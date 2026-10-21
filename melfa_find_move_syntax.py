#!/usr/bin/env python3
"""
Empirically find the relative joint-move command string accepted by an
RV-2AJ / CR1-571 over the non-procedural serial protocol.

None of the candidate strings below are known to be correct. Each is a
plausible MELFA-BASIC IV form, tried one at a time, with the joint angle
read before and after to see whether it actually moved.

Default is DRY RUN: only OPEN / JPOSF (read-only) are sent and the
candidates are printed. --arm is required to send anything that can move.

Usage:
  python3 melfa_find_move_syntax.py                 # dry run, J1, +2.0 deg
  python3 melfa_find_move_syntax.py --arm           # live test, J1, +2.0 deg
  python3 melfa_find_move_syntax.py --arm --joint J5 --step -2
"""

import argparse
import sys
import time

import serial

PORT_DEFAULT = "/dev/ttyUSB0"
SHORT_TIMEOUT = 2.0
# EXEC motion replies may only arrive once the move finishes.
MOVE_REPLY_TIMEOUT = 15.0
MAX_ABS_STEP = 5.0
MOTION_THRESHOLD_DEG = 0.1
OTHER_JOINT_TOLERANCE_DEG = 0.5
OVERSHOOT_TOLERANCE_DEG = 1.0
SETTLE_TIMEOUT = 10.0
OVERRIDE_PERCENT = 5

JOINTS = ["J1", "J2", "J3", "J4", "J5", "J6"]
ABSENT_JOINTS = {"J4"}  # 5-axis RV-2AJ; JPOSF reports J4 as ****
MISCALIBRATED_JOINTS = {"J3"}


class Link:
    def __init__(self, port):
        self.ser = serial.Serial(
            port=port,
            baudrate=9600,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_EVEN,
            stopbits=serial.STOPBITS_TWO,
            timeout=0.2,
        )

    def send(self, command, timeout=SHORT_TIMEOUT):
        self.ser.reset_input_buffer()
        self.ser.write(f"1;1;{command}\r".encode("ascii"))
        self.ser.flush()
        deadline = time.monotonic() + timeout
        buf = bytearray()
        while time.monotonic() < deadline:
            chunk = self.ser.read(1)
            if chunk:
                buf += chunk
                if chunk == b"\r":
                    break
        return buf.decode("ascii", errors="replace")

    def close(self):
        self.ser.close()


def is_ok(reply):
    return reply.startswith("Qo")


def tx(link, command, timeout=SHORT_TIMEOUT):
    print(f"    -> 1;1;{command}")
    reply = link.send(command, timeout=timeout)
    print(f"    <- {reply!r}" if reply else "    <- <no reply>")
    return reply


def parse_jposf(reply):
    """Return {J1..J6: float or None}; None for **** (axis absent)."""
    body = reply[2:] if reply.startswith("Qo") else reply
    body = body.lstrip("Kk").strip("\r")
    parts = body.split(";")
    joints = {}
    for i in range(len(parts) - 1):
        if parts[i] in JOINTS and parts[i] not in joints:
            try:
                joints[parts[i]] = float(parts[i + 1])
            except ValueError:
                joints[parts[i]] = None
    for j in JOINTS:
        joints.setdefault(j, None)
    return joints


def read_joints(link):
    reply = tx(link, "JPOSF")
    if not is_ok(reply):
        raise RuntimeError(f"JPOSF failed: {reply!r}")
    return parse_jposf(reply)


def wait_settled(link):
    prev = read_joints(link)
    deadline = time.monotonic() + SETTLE_TIMEOUT
    while time.monotonic() < deadline:
        time.sleep(0.3)
        cur = read_joints(link)
        if all(
            prev[j] is None or cur[j] is None or abs(cur[j] - prev[j]) < 0.01
            for j in JOINTS
        ):
            return cur
        prev = cur
    return prev


def fmt_tuple(values):
    return "(" + ",".join(f"{v:.2f}" for v in values) + ")"


def build_candidates(current, joint, step):
    """Candidate command sequences. The last command in each is the move."""
    idx = JOINTS.index(joint)
    n = idx + 1
    cur6 = [current[j] if current[j] is not None else 0.0 for j in JOINTS]
    abs6 = list(cur6)
    abs6[idx] += step
    delta6 = [0.0] * 6
    delta6[idx] = step
    abs8 = abs6 + [0.0, 0.0]
    delta8 = delta6 + [0.0, 0.0]

    return [
        {
            "label": "EXECMOV J_CURR + 6-element offset tuple",
            "commands": [f"EXECMOV J_CURR+{fmt_tuple(delta6)}"],
        },
        {
            "label": "EXECMOV J_CURR + 8-element offset tuple (J7/J8 = 0)",
            "commands": [f"EXECMOV J_CURR+{fmt_tuple(delta8)}"],
        },
        {
            "label": "Joint variable = J_CURR + offset, then EXECMOV JTMP",
            "commands": [f"EXECJTMP=J_CURR+{fmt_tuple(delta6)}", "EXECMOV JTMP"],
        },
        {
            "label": "Joint variable = absolute 6-element tuple, then EXECMOV JTMP",
            "commands": [f"EXECJTMP={fmt_tuple(abs6)}", "EXECMOV JTMP"],
        },
        {
            "label": "Joint variable = absolute 8-element tuple, then EXECMOV JTMP",
            "commands": [f"EXECJTMP={fmt_tuple(abs8)}", "EXECMOV JTMP"],
        },
        {
            "label": f"JTMP = J_CURR, then JTMP.J{n} += RAD(step), then EXECMOV JTMP",
            "note": (
                "Joint components are in radians in MELFA-BASIC, hence RAD(). "
                "If components were actually degrees this moves only ~0.03 deg "
                "and reads as 'no motion' - the safe failure direction."
            ),
            "commands": [
                "EXECJTMP=J_CURR",
                f"EXECJTMP.J{n}=JTMP.J{n}+RAD({step:.2f})",
                "EXECMOV JTMP",
            ],
        },
        {
            "label": "'EXEC MOV' with a space, J_CURR + 6-element offset tuple",
            "commands": [f"EXEC MOV J_CURR+{fmt_tuple(delta6)}"],
        },
        {
            "label": "EXECMOV with an inline absolute tuple (no joint type marker)",
            "risky": True,
            "note": (
                "RISK: a bare tuple may be parsed as a Cartesian position "
                "(X,Y,Z,A,B,C), not joint angles. Your joint angles read as "
                "millimetres/degrees could be a reachable but completely "
                "different pose, i.e. a large move. Tried last, needs YES."
            ),
            "commands": [f"EXECMOV {fmt_tuple(abs6)}"],
        },
    ]


def print_candidates(candidates):
    for i, cand in enumerate(candidates, 1):
        tag = "  [RISKY]" if cand.get("risky") else ""
        print(f"\n  Candidate {i}: {cand['label']}{tag}")
        if cand.get("note"):
            print(f"    note: {cand['note']}")
        for cmd in cand["commands"]:
            print(f"    1;1;{cmd}")


def print_arm_header(joint, step):
    print()
    print("#" * 72)
    print("#  ARMED MODE: THE FOLLOWING COMMANDS CAN MOVE THE ROBOT")
    print("#" * 72)
    print("  Confirm ALL of the following before continuing:")
    print("    - The work area is clear of people and obstacles")
    print("    - Teaching pendant switch is DISABLE")
    print("    - Controller MODE switch is AUTO (Ext.)")
    print("    - Your hand is on or near the controller EMERGENCY STOP")
    print(f"  Joint {joint}, step {step:+.2f} deg, override forced to "
          f"{OVERRIDE_PERCENT}% before every candidate.")
    print("  Ctrl-C at any time sends STOP, SRVOFF, CNTLOFF, CLOSE.")
    print("#" * 72)


def validate_args(args):
    args.joint = args.joint.upper()
    if args.joint not in JOINTS:
        sys.exit(f"--joint must be one of {', '.join(JOINTS)}")
    if args.joint in ABSENT_JOINTS:
        sys.exit(f"{args.joint} is not fitted on this robot.")
    if not (0 < abs(args.step) <= MAX_ABS_STEP):
        sys.exit(f"--step must be non-zero and at most {MAX_ABS_STEP} deg in magnitude")


def run_armed(link, candidates, joint, step):
    """Returns the working candidate, or None."""
    results = []

    for cmd in ("CNTLON", "RSTALRM"):
        tx(link, cmd)
    if input("\n  Send SRVON now? Type YES: ").strip() != "YES":
        print("  Not confirmed. Stopping.")
        return None, results
    if not is_ok(tx(link, "SRVON")):
        print("  SRVON rejected. Stopping.")
        return None, results
    time.sleep(2.0)

    needs_reset = False
    for i, cand in enumerate(candidates, 1):
        print("\n" + "=" * 72)
        print(f"  Candidate {i}/{len(candidates)}: {cand['label']}")
        if cand.get("note"):
            print(f"  note: {cand['note']}")
        print("  About to send:")
        for cmd in cand["commands"]:
            print(f"    1;1;{cmd}")

        if cand.get("risky"):
            answer = input("  Type YES to send this RISKY candidate, anything else skips: ").strip()
            if answer != "YES":
                results.append((i, cand["label"], "skipped", None))
                continue
        else:
            answer = input("  Press Enter to send, 's' + Enter to skip: ").strip().lower()
            if answer == "s":
                results.append((i, cand["label"], "skipped", None))
                continue

        if needs_reset:
            print("  Previous candidate errored: clearing alarm and re-enabling servo.")
            tx(link, "RSTALRM")
            if not is_ok(tx(link, "SRVON")):
                print("  SRVON rejected after reset. Stopping.")
                break
            time.sleep(2.0)
            needs_reset = False

        if not is_ok(tx(link, f"EXECJOVRD {OVERRIDE_PERCENT}")):
            print(f"  Could not force {OVERRIDE_PERCENT}% override. Stopping for safety.")
            break

        before = read_joints(link)

        accepted = True
        for k, cmd in enumerate(cand["commands"]):
            is_move = k == len(cand["commands"]) - 1
            reply = tx(link, cmd, timeout=MOVE_REPLY_TIMEOUT if is_move else SHORT_TIMEOUT)
            if not is_ok(reply):
                accepted = False
                needs_reset = True
                break

        after = wait_settled(link)
        delta = after[joint] - before[joint]
        others_moved = [
            j for j in JOINTS
            if j != joint and before[j] is not None and after[j] is not None
            and abs(after[j] - before[j]) > OTHER_JOINT_TOLERANCE_DEG
        ]
        moved = abs(delta) > MOTION_THRESHOLD_DEG

        print(f"  {joint}: {before[joint]:.2f} -> {after[joint]:.2f} (delta {delta:+.2f} deg)")
        print(f"  reply accepted: {accepted}, joint moved: {moved}")

        if others_moved or abs(delta) > abs(step) + OVERSHOOT_TOLERANCE_DEG:
            print("\n  !!! UNEXPECTED MOTION: "
                  + (f"other joints moved: {', '.join(others_moved)}" if others_moved
                     else f"{joint} moved {delta:+.2f} deg, more than the {step:+.2f} step")
                  + ". Stopping.")
            tx(link, "STOP")
            tx(link, "SRVOFF")
            results.append((i, cand["label"], "UNEXPECTED MOTION", delta))
            return None, results

        if accepted and moved:
            results.append((i, cand["label"], "WORKS", delta))
            if (delta > 0) != (step > 0) or abs(abs(delta) - abs(step)) > 0.5:
                print(f"  NOTE: moved {delta:+.2f} deg but asked for {step:+.2f} deg; "
                      "check the direction/units before trusting this form.")
            return cand, results

        if moved and not accepted:
            print("  Joint moved even though the reply was not Qo. Stopping.")
            results.append((i, cand["label"], "moved without Qo", delta))
            return None, results

        results.append((i, cand["label"], "accepted, no motion" if accepted else "rejected", delta))

    return None, results


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=PORT_DEFAULT)
    parser.add_argument("--joint", default="J1")
    parser.add_argument("--step", type=float, default=2.0, help="degrees, default 2.0")
    parser.add_argument("--arm", action="store_true", help="actually send motion commands")
    args = parser.parse_args()
    validate_args(args)

    if args.joint in MISCALIBRATED_JOINTS:
        print(f"WARNING: {args.joint}'s origin is currently miscalibrated on this robot.")
        if args.arm and input(f"Type YES to test {args.joint} anyway: ").strip() != "YES":
            sys.exit("Aborted.")

    print(f"Mode: {'ARMED' if args.arm else 'DRY RUN (nothing that can move will be sent)'}")
    print(f"Port {args.port}, joint {args.joint}, step {args.step:+.2f} deg")

    link = None
    try:
        link = Link(args.port)
        tx(link, "OPEN")
        current = read_joints(link)
        print("  Current joints: " + ", ".join(
            f"{j}={'absent' if current[j] is None else f'{current[j]:.2f}'}" for j in JOINTS))
        if current[args.joint] is None:
            sys.exit(f"{args.joint} has no reading; cannot build candidates.")

        candidates = build_candidates(current, args.joint, args.step)
        print("\nCandidate command sequences (none verified):")
        print_candidates(candidates)

        if not args.arm:
            print("\nDry run complete. Re-run with --arm to test these on the robot.")
            return

        print_arm_header(args.joint, args.step)
        if input("  Type ARM to continue: ").strip() != "ARM":
            print("  Not armed. Nothing sent.")
            return

        working, results = run_armed(link, candidates, args.joint, args.step)

        print("\n" + "=" * 72)
        print("RESULTS")
        print("=" * 72)
        for i, label, outcome, delta in results:
            d = f" ({delta:+.2f} deg)" if delta is not None else ""
            print(f"  {i}. {outcome}{d}: {label}")
        if working:
            print("\n  CONFIRMED WORKING SYNTAX (accepted with Qo AND the joint moved):")
            for cmd in working["commands"]:
                print(f"    1;1;{cmd}")
        else:
            print("\n  No candidate was both accepted and produced motion.")

    except KeyboardInterrupt:
        print("\n  Interrupted.")
    finally:
        if link is not None:
            print("\nShutting down link:")
            cleanup = (["STOP"] if args.arm else []) + ["SRVOFF", "CNTLOFF", "CLOSE"]
            for cmd in cleanup:
                try:
                    tx(link, cmd)
                except Exception as e:
                    print(f"    {cmd} failed: {e}")
            link.close()


if __name__ == "__main__":
    main()
