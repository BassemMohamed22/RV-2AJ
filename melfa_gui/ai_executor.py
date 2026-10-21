"""
Runs a validated plan on the serial link. Called on the GUI's serial worker
thread, so it never interleaves with polling; STOP from the GUI still gets
through between transactions, and sets `abort` so the plan ends there.

Non-motion commands are sent exactly as validated. A non-Qo reply stops
the plan and is reported with the exact reply string. Motion steps (only
reachable when MOVE_SYNTAX_VERIFIED is True) re-check the joint limits
against a fresh JPOSF, go through serial_link's existing move functions,
and wait for the arm to settle before the next step; a rejected motion
command is re-raised so the GUI's link-error path turns the servo off.
"""

import time
from dataclasses import dataclass, field

import ai_validator as v
from ai_log import log_event
from serial_link import (
    OUT_BUSY,
    OUT_STATION_READY,
    CommandRejected,
    decode_order,
    error_code,
    bit,
    is_ok_reply,
    order_bits,
    parse_jposf,
    parse_pposf,
)

SETTLE_TOLERANCE_DEG = 0.1
SETTLE_POLL_S = 0.2
SETTLE_STABLE_POLLS = 5
SETTLE_TIMEOUT_S = 90.0


class PlanAborted(Exception):
    pass


@dataclass
class ExecResult:
    ok: bool = True
    lines: list = field(default_factory=list)   # (command, reply, summary)
    spoken: list = field(default_factory=list)  # sentences to say
    error: str = ""
    servo_off: bool = False


def _value(reply):
    return int(reply.strip().rsplit("=", 1)[1])


def summarize(command, reply):
    """Plain-language summary of a read reply (deterministic, no model)."""
    try:
        if command == "JPOSF":
            j = parse_jposf(reply)
            return "Joints: " + ", ".join(f"{k} {j[k]:.1f}" for k in ("J1", "J2", "J3", "J5", "J6")
                                           if j.get(k) is not None) + " degrees."
        if command == "PPOSF":
            p = parse_pposf(reply)
            return "Tool at " + ", ".join(f"{k} {p[k]:.1f}" for k in ("X", "Y", "Z") if p.get(k) is not None) \
                + " millimetres."
        if command == "STATE":
            return "Controller reports an alarm." if reply[2:3].islower() else "Controller OK, no alarm."
        if command == "VALM_INB(0)":
            inputs = _value(reply)
            b1, b0 = order_bits(inputs)
            return f"Order from the line: {decode_order(inputs).value} (IN7 {b1}, IN6 {b0})."
        if command == "VALM_OUTB(0)":
            outputs = _value(reply)
            return (f"Station ready {'on' if bit(outputs, OUT_STATION_READY) else 'off'}, "
                    f"busy {'on' if bit(outputs, OUT_BUSY) else 'off'}, "
                    f"error code {error_code(outputs)}.")
    except (ValueError, IndexError, KeyError, TypeError):
        return f"{command}: {reply.strip()}"
    return ""


def _check_abort(abort):
    if abort.is_set():
        raise PlanAborted("stopped")


def _current_joints(link):
    return parse_jposf(link.expect_ok("JPOSF"))


def _wait_settled(link, targets, abort, progress):
    last, stable = None, 0
    deadline = time.monotonic() + SETTLE_TIMEOUT_S
    while time.monotonic() < deadline:
        _check_abort(abort)
        joints = _current_joints(link)
        now = {k: joints.get(k) for k in targets}
        if all(x is not None for x in now.values()):
            if all(abs(now[k] - t) <= SETTLE_TOLERANCE_DEG for k, t in targets.items()):
                return now
            if last and all(abs(now[k] - last[k]) < 0.01 for k in now):
                stable += 1
                if stable >= SETTLE_STABLE_POLLS:
                    off = ", ".join(f"{k} {now[k]:.2f}° (target {t:.2f}°)" for k, t in targets.items())
                    raise PlanAborted(f"the arm stopped short: {off}")
            else:
                stable = 0
            last = now
            progress("moving… " + "  ".join(f"{k} {x:.1f}" for k, x in now.items()))
        time.sleep(SETTLE_POLL_S)
    raise PlanAborted("the move did not finish in time")


def _run_motion(link, step, override, abort, progress):
    if not v.MOVE_SYNTAX_VERIFIED:  # validated earlier; checked again here on purpose
        raise PlanAborted(v.MOTION_DISABLED_REASON)
    current = _current_joints(link)
    if step.motion["type"] == "rel":
        joint, delta = step.motion["joint"], step.motion["delta"]
        target = current[joint] + delta
        error = v._joint_limit_error(joint, target, current[joint])
        if error:
            raise PlanAborted(f"at execution time: {error}")
        progress(f"moving {joint} by {delta:+g}°")
        reached = link.send_step_move(int(joint[1]), delta, override)
        return _wait_settled(link, {joint: reached}, abort, progress)

    targets = step.motion["targets"]
    for joint, value in targets.items():
        error = v._joint_limit_error(joint, value, current.get(joint))
        if error:
            raise PlanAborted(f"at execution time: {error}")
    values = [targets.get(f"J{i}", current.get(f"J{i}")) for i in range(1, 7)]
    progress("moving to " + ", ".join(f"{j} {x:g}" for j, x in targets.items()))
    sent = link.send_joint_move(values, override)
    return _wait_settled(link, {j: sent[int(j[1]) - 1] for j in targets}, abort, progress)


def execute(link, plan, abort, default_override, progress=lambda text: None):
    """Run every step of a validated plan in order. Returns an ExecResult.
    Raises SerialLinkError for link failures and rejected motion commands
    (the GUI then shuts the link down with servo off)."""
    result = ExecResult()
    if not plan.ok:
        result.ok, result.error = False, "plan did not pass validation"
        return result
    override = min(int(default_override), v.MAX_OVERRIDE_PERCENT)
    for i, step in enumerate(plan.steps, 1):
        try:
            _check_abort(abort)
            progress(f"step {i}/{len(plan.steps)}: {step.command}")
            if step.kind == v.Kind.MOTION:
                reached = _run_motion(link, step, override, abort, progress)
                summary = "reached " + ", ".join(f"{k} {x:.2f}°" for k, x in reached.items())
                result.lines.append((step.command, "", summary))
                log_event("EXEC", step=step.command, result=summary)
                continue
            reply = link.send(step.command).strip()
            log_event("EXEC", step=step.command, reply=reply)
            if not is_ok_reply(reply):
                result.ok = False
                result.error = f"controller rejected {step.command} with {reply}"
                result.lines.append((step.command, reply, "rejected"))
                log_event("EXEC_REJECTED", step=step.command, reply=reply)
                return result
            summary = summarize(step.command, reply)
            result.lines.append((step.command, reply, summary))
            if summary:
                result.spoken.append(summary)
            if step.command == "SRVOFF":
                result.servo_off = True
            elif step.command.startswith("EXECJOVRD"):
                override = int(step.command.split()[1])
        except PlanAborted as e:
            result.ok, result.error = False, str(e)
            log_event("EXEC_ABORTED", step=step.command, reason=str(e))
            return result
        except CommandRejected as e:
            log_event("EXEC_REJECTED", step=step.command, reply=getattr(e, "reply", None), error=str(e))
            raise
    return result
