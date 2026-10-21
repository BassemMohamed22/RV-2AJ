"""
Safety layer between the language model and the serial link.

The model may compose any sequence it likes; nothing reaches the robot
unless every command in the sequence matches one of the confirmed command
forms below and passes the range checks. A plan is all-or-nothing: one bad
command rejects the whole plan, and a rejected plan is never retried with a
modified command.

Pure logic: no serial, no Qt, so it can be tested on its own.

Confirmed on this controller by read-back (see serial_link.py):
    EXECM_OUT(n)=0|1   VALM_INB(0)   VALM_OUTB(0)   CNTLON   CNTLOFF
    SRVON   SRVOFF   EXECJOVRD <n>   RUN   STOP   JPOSF   PPOSF   STATE
    EXECHOPEN <hand>   EXECHCLOSE <hand>   (gripper)

Transfer-line shorthands, expanded here (deterministically) into the
confirmed EXECM_OUT writes, so a small model never has to do bit maths:
    LINE_READY ON|OFF      -> EXECM_OUT(7)=1|0
    LINE_BUSY ON|OFF       -> EXECM_OUT(6)=1|0
    LINE_ERROR_CODE 0..3   -> EXECM_OUT(5)=bit1, EXECM_OUT(4)=bit0 (clears before sets)

Motion is written by the model as abstract commands, never as controller
strings:
    MOVE_JOINT_REL J<n> <+/-degrees>
    MOVE_JOINT_ABS J1=<deg> J2=<deg> ...        (unlisted joints stay put)
All motion is rejected while MOVE_SYNTAX_VERIFIED is False.
"""

import math
import re
from dataclasses import dataclass, field
from enum import Enum

# The joint-move syntax (EXECJTMP / VALJTMP / EXECMOV JTMP, used by
# serial_link.send_step_move / send_joint_move) was verified on this robot on
# 2026-09-23. Motion enabled by the operator 2026-09-26; every move still needs
# Confirm. Set to False to reject all motion again.
MOVE_SYNTAX_VERIFIED = True
MOTION_DISABLED_REASON = (
    "Motion is disabled: the move syntax has not been verified on this robot "
    "(MOVE_SYNTAX_VERIFIED = False in ai_validator.py)."
)

# Joint limits the AI layer enforces on every motion target, in degrees.
JOINT_LIMITS_DEG = {
    "J1": (-150.0, 150.0),
    "J2": (-60.0, 120.0),
    "J3": (-110.0, 120.0),
    "J5": (-90.0, 90.0),
    "J6": (-200.0, 200.0),
}
ABSENT_JOINTS = {"J4"}  # 5-axis RV-2AJ

MAX_OVERRIDE_PERCENT = 30
HAND_NUMBER = 1  # gripper hand used with EXECHOPEN / EXECHCLOSE
MAX_PLAN_STEPS = 20

FORBIDDEN_OUTPUTS = frozenset(range(0, 4))  # not connected per the communication plan
ALLOWED_OUTPUTS = frozenset(range(4, 16))   # CR1 standard I/O has outputs 0-15
OUTPUT_NAMES = {
    4: "ERR CODE #0",
    5: "ERR CODE #1",
    6: "order active / busy",
    7: "station ready",
}

# Emergency stop words, matched on the raw transcript before any model sees
# it. Whole words only, so "stopper" or "durum" do not trigger.
STOP_WORDS = frozenset({"stop", "dur"})


def is_emergency_stop(raw_text):
    """True if the raw transcript contains a stop word ("stop", "dur",
    "stop robot", "STOP!" ...). Deliberately broad: a false stop is safe."""
    words = re.findall(r"[a-zçğıöşü]+", (raw_text or "").lower())
    return any(w in STOP_WORDS for w in words)


class Kind(str, Enum):
    READ = "read"          # queries only; may run without confirmation
    SAFE = "safe"          # STOP / SRVOFF; may run without confirmation
    ACTUATE = "actuate"    # outputs, gripper, control rights, override
    PROGRAM = "program"    # RUN: starts a robot program, may move the arm
    MOTION = "motion"      # joint moves


DIRECT_KINDS = frozenset({Kind.READ, Kind.SAFE})


@dataclass
class Step:
    source: str                 # exactly what the model produced
    command: str = ""           # normalized command sent to the controller
    kind: Kind = None
    ok: bool = False
    reason: str = ""            # rejection reason, or what the step does
    motion: dict = None         # {"type": "rel"/"abs", ...} for motion steps


@dataclass
class Plan:
    steps: list = field(default_factory=list)
    ok: bool = False
    reason: str = ""

    @property
    def needs_confirmation(self):
        return any(s.kind not in DIRECT_KINDS for s in self.steps)

    @property
    def has_motion(self):
        return any(s.kind in (Kind.MOTION, Kind.PROGRAM) for s in self.steps)


_NUM = r"[+-]?\d+(?:\.\d+)?"
_PATTERNS = [
    ("out", re.compile(r"^EXECM_OUT\((\d+)\)\s*=\s*([01])$")),
    ("inb", re.compile(r"^VALM_INB\(0\)$")),
    ("outb", re.compile(r"^VALM_OUTB\(0\)$")),
    ("read", re.compile(r"^(JPOSF|PPOSF|STATE)$")),
    ("cntlon", re.compile(r"^CNTLON$")),
    ("cntloff", re.compile(r"^CNTLOFF$")),
    ("srvon", re.compile(r"^SRVON$")),
    ("srvoff", re.compile(r"^SRVOFF$")),
    ("stop", re.compile(r"^STOP$")),
    ("ovrd", re.compile(r"^EXECJOVRD\s+(\d+)$")),
    ("run", re.compile(r"^RUN\s+(\d{1,4})$")),
    ("hand", re.compile(r"^EXECH(OPEN|CLOSE)\s+(\d+)$")),
    ("rel", re.compile(rf"^MOVE_JOINT_REL\s+(J\d)\s+({_NUM})$")),
    ("abs", re.compile(rf"^MOVE_JOINT_ABS((?:\s+J\d\s*=\s*{_NUM})+)$")),
]
_ABS_ITEM = re.compile(rf"(J\d)\s*=\s*({_NUM})")


def normalize(text):
    """Upper-case, trim and collapse spaces; drop a leading "1;1;" transport
    prefix if the model added one. Returns None if anything else looks like
    a second command smuggled into the string."""
    s = " ".join(str(text).strip().upper().split())
    if s.startswith("1;1;"):
        s = s[4:].strip()
    if any(c in s for c in ";\r\n"):
        return None
    return s


def _joint_limit_error(joint, value, current=None):
    """A target must be inside the limits, except that a joint already
    outside (the robot is often parked beyond the manual ranges) may move
    back toward the range, never further out."""
    lo, hi = JOINT_LIMITS_DEG[joint]
    if lo <= value <= hi:
        return None
    if current is not None:
        out_now = max(lo - current, current - hi, 0.0)
        if max(lo - value, value - hi) <= out_now + 1e-6:
            return None
    return f"{joint} target {value:.2f}° is outside the limit {lo:g}..{hi:g}°"



_LINE_MACRO = re.compile(r"^LINE_(READY|BUSY)\s+(ON|OFF|1|0)$|^LINE_ERROR_CODE\s+(\d+)$")


def expand(source):
    """Expand a transfer-line shorthand into confirmed commands; anything
    else passes through unchanged. Returns a list of command strings."""
    if not isinstance(source, str):
        return [source]
    s = normalize(source)
    m = _LINE_MACRO.match(s or "")
    if not m:
        return [source]
    if m[1]:
        n = 7 if m[1] == "READY" else 6
        return [f"EXECM_OUT({n})={1 if m[2] in ('ON', '1') else 0}"]
    code = int(m[3])
    if not 0 <= code <= 3:
        return [f"LINE_ERROR_CODE_OUT_OF_RANGE {code}"]  # rejected below as an unknown form
    bits = [(5, code >> 1 & 1), (4, code & 1)]
    bits.sort(key=lambda nv: nv[1])  # clears first, so the code never passes through a higher value
    return [f"EXECM_OUT({n})={b}" for n, b in bits]


def _check_step(source, pose, motion_verified, shown=None):
    """Validate one command. `pose` (dict J1..J6 or None) is advanced by
    motion steps so later relative moves are checked from the right place.
    `shown` is what the model wrote, if `source` came from a shorthand."""
    step = Step(source=shown or source)
    if not isinstance(source, str):
        step.reason = "not a text command"
        return step
    cmd = normalize(source)
    if not cmd:
        step.reason = "empty or contains more than one command"
        return step
    step.command = cmd

    for name, pattern in _PATTERNS:
        m = pattern.match(cmd)
        if m:
            break
    else:
        step.reason = "not one of the confirmed command forms"
        return step

    if name == "out":
        n, v = int(m[1]), int(m[2])
        if n in FORBIDDEN_OUTPUTS:
            step.reason = f"OUT{n} is not connected per the communication plan; OUT0-OUT3 are never allowed"
            return step
        if n not in ALLOWED_OUTPUTS:
            step.reason = f"OUT{n} does not exist on this controller (outputs 0-15)"
            return step
        label = OUTPUT_NAMES.get(n, "not in the communication plan")
        step.kind, step.ok, step.reason = Kind.ACTUATE, True, f"set OUT{n} ({label}) to {v}"
    elif name == "inb":
        step.kind, step.ok, step.reason = Kind.READ, True, "read inputs 0-7 (order bits IN6/IN7)"
    elif name == "outb":
        step.kind, step.ok, step.reason = Kind.READ, True, "read outputs 0-7"
    elif name == "read":
        what = {"JPOSF": "read joint angles", "PPOSF": "read tool position", "STATE": "read controller state"}
        step.kind, step.ok, step.reason = Kind.READ, True, what[m[1]]
    elif name == "cntlon":
        step.kind, step.ok, step.reason = Kind.ACTUATE, True, "take operation rights (no motion)"
    elif name == "cntloff":
        step.kind, step.ok, step.reason = Kind.ACTUATE, True, "release operation rights"
    elif name == "srvon":
        step.reason = "voice/text can never turn the servo on; use the Servo ON button"
    elif name == "srvoff":
        step.kind, step.ok, step.reason = Kind.SAFE, True, "servo off"
    elif name == "stop":
        step.kind, step.ok, step.reason = Kind.SAFE, True, "stop"
    elif name == "ovrd":
        pct = int(m[1])
        if not 1 <= pct <= MAX_OVERRIDE_PERCENT:
            step.reason = f"speed override {pct}% is outside 1-{MAX_OVERRIDE_PERCENT}%"
            return step
        step.kind, step.ok, step.reason = Kind.ACTUATE, True, f"set speed override to {pct}%"
    elif name == "run":
        step.kind, step.ok, step.reason = Kind.PROGRAM, True, f"run program {int(m[1])} (the arm may move)"
    elif name == "hand":
        action, hand = m[1], int(m[2])
        if hand != HAND_NUMBER:
            step.reason = f"hand {hand} is not the configured gripper (hand {HAND_NUMBER})"
            return step
        step.kind, step.ok = Kind.ACTUATE, True
        step.reason = f"{'open' if action == 'OPEN' else 'close'} the gripper (hand {hand}, up to 160 N)"
    elif name in ("rel", "abs"):
        step.kind = Kind.MOTION
        _check_motion(step, name, m, pose, motion_verified)
    return step


def _check_motion(step, name, m, pose, motion_verified):
    if not motion_verified:
        step.reason = MOTION_DISABLED_REASON
        return
    if pose is None or any(pose.get(j) is None for j in JOINT_LIMITS_DEG):
        step.reason = "current joint angles are unknown (connect first)"
        return

    if name == "rel":
        joint, delta = m[1], float(m[2])
        if joint in ABSENT_JOINTS or joint not in JOINT_LIMITS_DEG:
            step.reason = f"{joint} does not exist on this robot"
            return
        if not math.isfinite(delta) or delta == 0:
            step.reason = "the move distance must be a non-zero number"
            return
        target = pose[joint] + delta
        error = _joint_limit_error(joint, target, pose[joint])
        if error:
            step.reason = error
            return
        pose[joint] = target
        step.motion = {"type": "rel", "joint": joint, "delta": delta}
        step.ok, step.reason = True, f"move {joint} by {delta:+g}° (to {target:.2f}°)"
        return

    targets = {}
    for joint, value in _ABS_ITEM.findall(m[1]):
        if joint in ABSENT_JOINTS or joint not in JOINT_LIMITS_DEG:
            step.reason = f"{joint} does not exist on this robot"
            return
        if joint in targets:
            step.reason = f"{joint} is given twice"
            return
        targets[joint] = float(value)
    for joint, value in targets.items():
        error = _joint_limit_error(joint, value, pose[joint])
        if error:
            step.reason = error
            return
    pose.update(targets)
    step.motion = {"type": "abs", "targets": targets}
    step.ok = True
    step.reason = "move to " + ", ".join(f"{j} {v:g}°" for j, v in targets.items())


def validate(commands, current_joints=None, motion_verified=None):
    """Validate a model-produced command list. Returns a Plan; plan.ok is
    False if the list is empty, too long, or any step is rejected."""
    if motion_verified is None:
        motion_verified = MOVE_SYNTAX_VERIFIED
    plan = Plan()
    if not isinstance(commands, list):
        plan.reason = "the model did not return a command list"
        return plan
    if not commands:
        plan.reason = "no commands"
        return plan
    if len(commands) > MAX_PLAN_STEPS:
        plan.reason = f"{len(commands)} commands is more than the {MAX_PLAN_STEPS}-step limit"
        return plan

    pose = dict(current_joints) if current_joints else None
    for source in commands:
        for command in expand(source):
            shown = source if command != source else None
            plan.steps.append(_check_step(command, pose, motion_verified, shown))
    if len(plan.steps) > MAX_PLAN_STEPS:
        plan.ok = False
        plan.reason = f"{len(plan.steps)} commands is more than the {MAX_PLAN_STEPS}-step limit"
        return plan

    bad = [s for s in plan.steps if not s.ok]
    if bad:
        first = bad[0]
        shown = first.command or str(first.source)
        plan.reason = f"rejected “{shown}”: {first.reason}"
        if len(bad) > 1:
            plan.reason += f" (and {len(bad) - 1} more)"
        return plan
    plan.ok = True
    plan.reason = "all commands passed validation"
    return plan


_OUT_WRITE = re.compile(r"^EXECM_OUT\((\d+)\)\s*=\s*([01])$")


def describe_effects(plan, outputs=None):
    """What a validated plan will change, in plain words, so a wrong plan is
    obvious at Confirm time. `outputs` is the current output byte (bits 0-7)
    from the poll, or None if unknown."""
    effects = []
    writes = {}
    for s in plan.steps:
        m = _OUT_WRITE.match(s.command)
        if m:
            writes[int(m[1])] = int(m[2])
        elif s.command.startswith("EXECHOPEN"):
            effects.append("gripper opens")
        elif s.command.startswith("EXECHCLOSE"):
            effects.append("gripper closes")
        elif s.command.startswith("EXECJOVRD"):
            effects.append(f"speed override {s.command.split()[1]}%")
        elif s.kind in (Kind.MOTION, Kind.PROGRAM):
            effects.append(s.reason)
        elif s.command == "SRVOFF":
            effects.append("servo off")

    def before_bit(n):
        return None if outputs is None or n > 7 else (outputs >> n) & 1

    def change(label, old, new):
        return f"{label} {new}" if old is None or old == new else f"{label} {old}→{new}"

    onoff = {0: "off", 1: "on", None: "?"}
    for n, label in ((7, "station ready"), (6, "busy")):
        if n in writes:
            old = before_bit(n)
            effects.append(change(label, None if old is None else onoff[old], onoff[writes[n]]))
    if 4 in writes or 5 in writes:
        b0_old, b1_old = before_bit(4), before_bit(5)
        old = None if b0_old is None or b1_old is None else b1_old << 1 | b0_old
        new_b0 = writes.get(4, b0_old)
        new_b1 = writes.get(5, b1_old)
        if new_b0 is None or new_b1 is None:
            effects.append("error code bits " + ", ".join(f"OUT{n}={writes[n]}" for n in (5, 4) if n in writes))
        else:
            effects.append(change("error code", old, new_b1 << 1 | new_b0))
    for n in sorted(writes):
        if n not in (4, 5, 6, 7):
            effects.append(change(f"OUT{n}", before_bit(n), writes[n]))
    return effects
