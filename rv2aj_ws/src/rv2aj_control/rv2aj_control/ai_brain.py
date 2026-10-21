"""
AI brain for the RV-2AJ ROS stack: the command vocabulary the language
model writes, the safety validator, the teaching memory, and the prompt.

Pure Python (no rclpy), so it can be tested and evaluated on its own.

The model proposes; this module decides. Every command must be one of the
forms below. Motion is simulated with the robot's own kinematics from the
live pose, step after step, so a plan is checked against the joint limits at
every point it will pass, and a straight tool move that is out of reach is
rejected before MoveIt ever sees it. MoveIt then checks collisions when the
move is planned. Nothing here talks to the robot.

Vocabulary (one command per list item):
  WHERE | STATUS | LIST_POINTS | LIST_SKILLS          read, run immediately
  STOP | SERVO_OFF | WAIT <s>                          safe
  GRIPPER OPEN|CLOSE | SPEED <1-30>                    actuate (confirm)
  MOVE_JOINT_REL J<n> <+/-deg>                         motion  (confirm)
  MOVE_JOINT_ABS J1=<deg> J2=<deg> ...                 motion  (confirm)
  MOVE_TCP UP|DOWN|LEFT|RIGHT|FORWARD|BACK <mm>        motion  (confirm)
  MOVE_TCP X|Y|Z|R <+/-mm>                             motion  (confirm)
  GOTO_POINT <name> | GO_HOME                          motion  (confirm)
  RUN_SKILL <name>                                     expands to the skill's steps
  SAVE_POINT <name>                                    teach   (confirm)
  DEFINE_SKILL <name>: <cmd> | <cmd> | ...             teach   (confirm)
  FORGET_SKILL <name>                                  teach   (confirm)
SERVO_ON is never accepted: the operator turns the servo on in the GUI.
"""

import json
import math
import os
import re
import threading
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum

MOVE_JOINTS = ("J1", "J2", "J3", "J5", "J6")
ALL_JOINTS = ("J1", "J2", "J3", "J4", "J5", "J6")

# Joint limits the AI enforces on every pose a plan passes through (degrees).
# Overridable with the ai node's joint_limits_deg parameter.
DEFAULT_LIMITS = {  # measured on the robot 2026-09-28 (see rv2aj_description/config/joint_limits.yaml)
    "J1": (-327.5, -21.5),
    "J2": (-150.0, 0.5),
    "J3": (-43.0, 210.0),
    "J5": (-272.0, -84.0),
    "J6": (-425.0, 335.0),
}
MAX_OVERRIDE = 30
MAX_JOINT_REL_DEG = 90.0      # per command; the executor splits into planner-sized steps
MAX_TCP_MM = 200.0            # per command; split into <= 50 mm planner moves
MAX_WAIT_S = 10.0
MAX_PLAN_STEPS = 30           # after skills are expanded
MIN_RADIAL_RADIUS_MM = 50.0
NAME_RE = r"[A-Za-z][A-Za-z0-9_-]{0,23}"

DIRECTIONS = {  # same conventions as the GUI jog pad and voice_commands.py
    "UP": ("Z", +1), "DOWN": ("Z", -1),
    "LEFT": ("Y", +1), "RIGHT": ("Y", -1),
    "FORWARD": ("R", +1), "BACK": ("R", -1),
}
STOP_WORDS = frozenset({"stop", "dur", "durdur", "dursun", "durun"})


def is_emergency_stop(raw_text):
    """Stop words matched on the raw transcript, whole words only. A false
    positive only stops the robot, so the check is deliberately broad."""
    words = re.findall(r"[a-zçğıöşü]+", (raw_text or "").lower())
    return any(w in STOP_WORDS for w in words)


class Kind(str, Enum):
    READ = "read"
    SAFE = "safe"
    ACTUATE = "actuate"
    TEACH = "teach"
    MOTION = "motion"


DIRECT_KINDS = frozenset({Kind.READ, Kind.SAFE})


@dataclass
class World:
    """What the validator knows about the robot right now."""
    joints: dict = None          # J1..J6 degrees (J4 may be None/NaN)
    connected: bool = False
    servo_on: bool = False
    gripper: str = "unknown"
    points: dict = field(default_factory=dict)   # name -> {"joints": [6], "gripper": str}
    home: list = None                             # 6 values
    skills: dict = field(default_factory=dict)    # name -> [commands]


@dataclass
class Step:
    source: str
    command: str = ""
    kind: Kind = None
    ok: bool = False
    reason: str = ""
    action: dict = None          # what the executor does
    skill: str = ""              # set when the step came from RUN_SKILL


@dataclass
class Plan:
    steps: list = field(default_factory=list)
    ok: bool = False
    reason: str = ""
    effects: list = field(default_factory=list)

    @property
    def needs_confirmation(self):
        return any(s.kind not in DIRECT_KINDS for s in self.steps)

    @property
    def has_motion(self):
        return any(s.kind == Kind.MOTION for s in self.steps)

    def to_dict(self):
        return {"ok": self.ok, "reason": self.reason, "effects": self.effects,
                "needs_confirmation": self.needs_confirmation,
                "steps": [{"source": s.source, "command": s.command, "ok": s.ok, "reason": s.reason,
                           "kind": s.kind.value if s.kind else None, "skill": s.skill}
                          for s in self.steps]}


# -- parsing --------------------------------------------------------------------

_NUM = r"[+-]?\d+(?:\.\d+)?"
PATTERNS = [
    ("read", re.compile(r"^(WHERE|STATUS|LIST_POINTS|LIST_SKILLS|LINE_STATUS)$")),
    ("line", re.compile(r"^LINE_(READY|BUSY)\s+(ON|OFF|1|0)$|^LINE_ERROR_CODE\s+([0-3])$")),
    ("stop", re.compile(r"^STOP$")),
    ("servo_off", re.compile(r"^SERVO_OFF$")),
    ("servo_on", re.compile(r"^(SERVO_ON|SRVON|SERVO ON)$")),
    ("wait", re.compile(rf"^WAIT\s+({_NUM})$")),
    ("gripper", re.compile(r"^GRIPPER[\s_]+(OPEN|CLOSE)$")),
    ("speed", re.compile(rf"^SPEED\s+({_NUM})$")),
    ("jrel", re.compile(rf"^MOVE_JOINT_REL\s+(J\d)\s*=?\s*({_NUM})$")),
    ("jabs", re.compile(rf"^MOVE_JOINT_ABS((?:\s+J\d\s*=\s*{_NUM})+)$")),
    ("tcp", re.compile(rf"^MOVE_TCP\s+(UP|DOWN|LEFT|RIGHT|FORWARD|BACK|X|Y|Z|R)\s+({_NUM})(?:\s*MM)?$")),
    ("square", re.compile(rf"^DRAW_SQUARE\s+({_NUM})(?:\s*MM)?(?:\s+(HORIZONTAL|VERTICAL))?$")),
    ("rect", re.compile(rf"^DRAW_RECTANGLE\s+({_NUM})(?:\s*MM)?\s+({_NUM})(?:\s*MM)?(?:\s+(HORIZONTAL|VERTICAL))?$")),
    ("circle", re.compile(rf"^DRAW_CIRCLE\s+({_NUM})(?:\s*MM)?(?:\s+(HORIZONTAL|VERTICAL))?$")),
    ("arc", re.compile(rf"^DRAW_ARC\s+({_NUM})(?:\s*MM)?\s+({_NUM})(?:\s*DEG)?(?:\s+(HORIZONTAL|VERTICAL))?$")),
    ("goto", re.compile(rf"^GOTO_POINT\s+({NAME_RE})$", re.I)),
    ("home", re.compile(r"^GO_HOME$")),
    ("run_skill", re.compile(rf"^RUN_SKILL\s+({NAME_RE})$", re.I)),
    ("save_point", re.compile(rf"^SAVE_POINT\s+({NAME_RE})$", re.I)),
    ("define", re.compile(rf"^DEFINE_SKILL\s+({NAME_RE})\s*:\s*(.+)$", re.I)),
    ("forget", re.compile(rf"^FORGET_SKILL\s+({NAME_RE})$", re.I)),
]
_ABS_ITEM = re.compile(rf"(J\d)\s*=\s*({_NUM})")


_TILT = re.compile(rf"^TILT_WRIST\s+(UP|DOWN)\s+({_NUM})$", re.I)


def normalize(text):
    """Collapse spaces; upper-case the keyword part (names keep their case).
    Word-based shorthands become their canonical command here, in fixed code,
    so the model never has to remember a sign convention."""
    s = " ".join(str(text).strip().split())
    m = _TILT.match(s)
    if m:  # wrist down = +J5, wrist up = -J5
        deg = abs(float(m[2]))
        return f"MOVE_JOINT_REL J5 {'+' if m[1].upper() == 'DOWN' else '-'}{deg:g}"
    s = re.sub(r"^GRIPPER_(OPEN|CLOSE)$", r"GRIPPER \1", s, flags=re.I)
    if re.match(r"^DRAW_", s, flags=re.I):
        s = re.sub(r"\b(yatay|YATAY)\b", "HORIZONTAL", s)
        s = re.sub(r"\b(dikey|DİKEY|DIKEY|düşey|DÜŞEY)\b", "VERTICAL", s)
        # Arc direction as a word, sign fixed here: LEFT = +degrees, RIGHT = -degrees.
        m = re.match(rf"^DRAW_ARC\s+({_NUM})(?:\s*MM)?\s+({_NUM})(?:\s*DEG)?\s+(LEFT|RIGHT|SOLA|SAĞA|SAGA)\b(.*)$", s, flags=re.I)
        if m:
            deg = abs(float(m[2])) * (-1 if m[3].upper() in ("RIGHT", "SAĞA", "SAGA") else 1)
            s = f"DRAW_ARC {m[1]} {deg:+g}{m[4]}"
    s = re.sub(r"^MOVE_TCP\s+(OUT|OUTWARD|FORWARDS)\b", "MOVE_TCP FORWARD", s, flags=re.I)
    s = re.sub(r"^MOVE_TCP\s+(IN|INWARD|BACKWARD|BACKWARDS)\b", "MOVE_TCP BACK", s, flags=re.I)
    head, _, rest = s.partition(" ")
    head = head.upper()
    if head in ("GOTO_POINT", "RUN_SKILL", "SAVE_POINT", "FORGET_SKILL"):
        return f"{head} {rest}".strip()
    if head == "DEFINE_SKILL":
        name, colon, body = rest.partition(":")
        return f"{head} {name.strip()}: {body.strip()}" if colon else f"{head} {rest}"
    return s.upper()


def match(command):
    for name, pattern in PATTERNS:
        m = pattern.match(command)
        if m:
            return name, m
    return None, None


def split_skill_body(body):
    return [c.strip() for c in re.split(r"\s*\|\s*", body) if c.strip()]


def find_name(mapping, name):
    """Case-insensitive lookup; returns the stored key or None."""
    for key in mapping:
        if key.lower() == name.lower():
            return key
    return None


# -- validation -----------------------------------------------------------------

# Language of every message this module produces ("tr" or "en"); the ai node
# sets it. The operator asked for Turkish only.
LANG = "tr"


def L(en, tr):
    return tr if LANG == "tr" else en


JOINT_TR = {"J1": "taban (J1)", "J2": "omuz (J2)", "J3": "dirsek (J3)", "J5": "bilek eğimi (J5)",
            "J6": "bilek dönüşü (J6)"}
DIR_TR = {"UP": "yukarı", "DOWN": "aşağı", "LEFT": "sola", "RIGHT": "sağa", "FORWARD": "ileri", "BACK": "geri"}
MIN_USEFUL_MM = 2.0
MIN_USEFUL_DEG = 0.5


def jname(jn):
    return L(jn, JOINT_TR.get(jn, jn))


class _Sim:
    """Pose the plan will be at after each step (None if unknown)."""

    def __init__(self, world):
        j = world.joints or {}
        ok = all(isinstance(j.get(k), (int, float)) and math.isfinite(j[k]) for k in MOVE_JOINTS)
        self.pose = {k: float(j[k]) for k in MOVE_JOINTS} if ok else None


def _outside(value, lo, hi):
    return max(lo - value, value - hi, 0.0)


def _band(jn, current, limits):
    """Where a joint may go from `current`: inside its limits, or, if it is
    already outside (the robot is often parked beyond the manual ranges),
    anywhere that is not further out than it is now."""
    lo, hi = limits[jn]
    return min(lo, current), max(hi, current)


def _clamp_joint(jn, current, target, limits):
    lo, hi = _band(jn, current, limits)
    return min(max(target, lo), hi)


def _path_ok(path, before, limits):
    """(ok, joint that would leave its band first)."""
    for q in path:
        for jn in MOVE_JOINTS:
            lo, hi = _band(jn, before[jn], limits)
            if not lo - 1e-6 <= q[jn] <= hi + 1e-6:
                return False, jn
    return True, None


def _tcp_delta(pose, axis, mm, kinematics):
    if axis == "R":
        p = kinematics.fk(*(pose[j] for j in MOVE_JOINTS))
        radius = math.hypot(p["X"], p["Y"])
        if radius < MIN_RADIAL_RADIUS_MM:
            raise ValueError(L("the tool is too close to the base axis to define forward/back",
                               "araç tabanın eksenine çok yakın; ileri/geri tanımlanamıyor"))
        return (p["X"] / radius, p["Y"] / radius, 0.0)
    return tuple(1.0 if a == axis else 0.0 for a in "XYZ")


def _tcp_reach(before, unit, mm, limits, kinematics):
    """Longest distance (whole mm, up to |mm|) the tool can travel in a straight
    line from `before` without leaving the joint bands or the reachable space.
    Returns (distance, path, cause) where cause explains a shortfall."""

    def attempt(d):
        delta = tuple(u * d * (1 if mm > 0 else -1) for u in unit)
        try:
            path = kinematics.straight_line(before, delta, step_mm=5.0)
        except kinematics.IKError:
            return None, L("the arm's reach", "kolun erişim sınırı")
        ok, jn = _path_ok(path, before, limits)
        if not ok:
            return None, L(f"the {jn} limit", f"{jname(jn)} sınırı")
        return path, None

    full = abs(mm)
    path, cause = attempt(full)
    if path is not None:
        return full, path, None
    lo, hi, best = 0, int(full), None
    while lo < hi:
        mid = (lo + hi + 1) // 2
        p, _ = attempt(mid) if mid > 0 else (None, None)
        if p is not None:
            lo, best = mid, p
        else:
            hi = mid - 1
    _, first_block = attempt(lo + 1)  # what actually stops the motion just past the reachable part
    return float(lo), best, first_block or cause


def _note(step, text):
    """A step that cannot move at all: nothing to execute, just tell the operator."""
    step.kind, step.ok, step.reason = Kind.SAFE, True, text
    step.action = {"type": "note", "text": text}


def check_command(source, world, limits, sim, kinematics, allow_teach=True):
    """Validate one command (not RUN_SKILL). Advances sim.pose for motion."""
    step = Step(source=source)
    if not isinstance(source, str) or not source.strip():
        step.reason = L("not a text command", "metin komutu değil")
        return step
    cmd = normalize(source)
    step.command = cmd
    if any(c in cmd for c in ";\r\n"):
        step.reason = L("contains more than one command", "birden fazla komut içeriyor")
        return step
    name, m = match(cmd)
    if name is None:
        step.reason = L("not a known command", "bilinen bir komut değil")
        return step

    if name == "read":
        what = {"WHERE": L("report position", "konumu söyle"), "STATUS": L("report status", "durumu söyle"),
                "LINE_STATUS": L("read the transfer line (order, signals)", "transfer hattını oku (sipariş, sinyaller)"),
                "LIST_POINTS": L("list saved points", "kayıtlı noktaları listele"),
                "LIST_SKILLS": L("list taught skills", "becerileri listele")}[m[1]]
        step.kind, step.ok, step.reason, step.action = Kind.READ, True, what, {"type": "read", "what": m[1]}
    elif name == "stop":
        step.kind, step.ok, step.reason, step.action = Kind.SAFE, True, L("stop", "dur"), {"type": "stop"}
    elif name == "line":
        if m[1]:
            signal, value = ("ready" if m[1] == "READY" else "busy"), (1 if m[2] in ("ON", "1") else 0)
            what = L({"ready": "station ready", "busy": "busy"}[signal], {"ready": "hazır", "busy": "meşgul"}[signal])
            step.reason = L(f"tell Station 4: {what} {'on' if value else 'off'}",
                            f"4. istasyona {what} sinyali {'ver' if value else 'kapat'}")
        else:
            signal, value = "error_code", int(m[3])
            step.reason = L(f"send error code {value} to Station 4", f"4. istasyona hata kodu {value} gönder")
        step.kind, step.ok = Kind.ACTUATE, True
        step.action = {"type": "line", "signal": signal, "value": value}
    elif name == "servo_off":
        step.kind, step.ok, step.reason = Kind.SAFE, True, L("servo off", "servoyu kapat")
        step.action = {"type": "servo_off"}
    elif name == "servo_on":
        step.reason = L("the AI can never turn the servo on; use the Servo ON button",
                        "yapay zekâ servoyu asla açamaz; Servo ON düğmesini kullanın")
    elif name == "wait":
        sec = float(m[1])
        if not 0 < sec <= MAX_WAIT_S:
            step.reason = L(f"wait must be 0-{MAX_WAIT_S:g} s", f"bekleme 0-{MAX_WAIT_S:g} saniye olmalı")
        else:
            step.kind, step.ok, step.reason = Kind.SAFE, True, L(f"wait {sec:g} s", f"{sec:g} saniye bekle")
            step.action = {"type": "wait", "s": sec}
    elif name == "gripper":
        opening = m[1] == "OPEN"
        step.kind, step.ok = Kind.ACTUATE, True
        step.reason = L("open the gripper", "tutucuyu aç") if opening else \
            L("close the gripper (up to 160 N)", "tutucuyu kapat (160 N'a kadar)")
        step.action = {"type": "gripper", "open": opening}
    elif name == "speed":
        pct = float(m[1])
        if pct != int(pct) or not 1 <= pct <= MAX_OVERRIDE:
            step.reason = L(f"speed {m[1]}% is outside 1-{MAX_OVERRIDE}%",
                            f"hız yüzde {m[1]} izin verilen aralığın (1-{MAX_OVERRIDE}) dışında")
        else:
            step.kind, step.ok = Kind.ACTUATE, True
            step.reason = L(f"speed override {int(pct)}%", f"hız yüzde {int(pct)}")
            step.action = {"type": "speed", "percent": int(pct)}
    elif name in ("jrel", "jabs", "tcp", "goto", "home"):
        step.kind = Kind.MOTION
        _check_motion(step, name, m, world, limits, sim, kinematics)
    elif name in SHAPES:
        step.kind = Kind.MOTION
        _check_shape(step, name, m, limits, sim, kinematics)
    elif name in ("save_point", "define", "forget"):
        if not allow_teach:
            step.reason = L("teaching commands cannot be part of a skill", "öğretme komutları beceri içinde olamaz")
            return step
        step.kind = Kind.TEACH
        _check_teach(step, name, m, world, limits, kinematics)
    return step


def _frozen(kinematics):
    """Joints locked in joint_limits.yaml (hardware fault)."""
    return getattr(kinematics, "FROZEN_JOINTS", frozenset()) if kinematics is not None else frozenset()


def _check_motion(step, name, m, world, limits, sim, kinematics):
    """Motion is clamped, never refused for a limit: the robot goes as far as
    the limits allow (never further out) and the step says how far that is."""
    if sim.pose is None:
        step.reason = L("the robot's position is unknown (not connected?)", "robotun konumu bilinmiyor (bağlı değil mi?)")
        return
    before = dict(sim.pose)

    if name == "jrel":
        jn, delta = m[1], float(m[2])
        if jn not in MOVE_JOINTS:
            step.reason = L(f"{jn} does not exist on this robot", f"bu robotta {jn} yok")
            return
        if delta == 0:
            step.reason = L("a joint move must be non-zero", "eklem hareketi sıfır olamaz")
            return
        if jn in _frozen(kinematics):
            step.reason = L(f"{jn} is locked because of an encoder fault; it cannot move until it is repaired",
                            f"{jname(jn)} enkoder arızası nedeniyle kilitli; tamir edilene kadar hareket etmiyor")
            return
        step_delta = math.copysign(min(abs(delta), MAX_JOINT_REL_DEG), delta)  # one move is at most 90°
        target = _clamp_joint(jn, before[jn], before[jn] + step_delta, limits)
        got = target - before[jn]
        if abs(got) < MIN_USEFUL_DEG:
            _note(step, L(f"{jn} is already at its limit; it cannot turn that way",
                          f"{jname(jn)} zaten sınırda; o yöne dönemiyor"))
            return
        after = dict(before, **{jn: target})
        step.action = {"type": "joint_rel", "joint": jn, "delta": got, "requested": delta}
        step.reason = f"{jn} {before[jn]:.1f}° → {target:.1f}°"
        if abs(got - delta) > 0.05:
            step.reason += L(f" (limit: {got:+.1f}° instead of {delta:+g}°)",
                             f" (sınır: {delta:+g}° yerine {got:+.1f}°)")

    elif name == "jabs":
        wanted = {}
        for jn, value in _ABS_ITEM.findall(m[1]):
            if jn not in MOVE_JOINTS:
                step.reason = L(f"{jn} does not exist on this robot", f"bu robotta {jn} yok")
                return
            wanted[jn] = float(value)
        locked = [jn for jn in wanted if jn in _frozen(kinematics) and abs(wanted[jn] - before[jn]) > 0.05]
        if locked:
            step.reason = L(f"{', '.join(locked)} is locked because of an encoder fault",
                            f"{', '.join(jname(j) for j in locked)} enkoder arızası nedeniyle kilitli; hareket etmiyor")
            return
        after = dict(before)
        clamped = []
        for jn, value in wanted.items():
            after[jn] = _clamp_joint(jn, before[jn], value, limits)
            if abs(after[jn] - value) > 0.05:
                clamped.append(jn)
        if all(abs(after[j] - before[j]) < MIN_USEFUL_DEG for j in wanted):
            _note(step, L("the requested joints are already as close as the limits allow",
                          "istenen eklemler sınırların izin verdiği en yakın konumda zaten"))
            return
        step.action = {"type": "joint_abs", "targets": after, "requested": wanted}
        step.reason = ", ".join(f"{j} {before[j]:.1f}° → {after[j]:.1f}°" for j in wanted)
        if clamped:
            step.reason += L(" (limit: " + ", ".join(f"{j} stops at {after[j]:.1f}° instead of {wanted[j]:g}°" for j in clamped) + ")",
                             " (sınır: " + ", ".join(f"{j} {wanted[j]:g}° yerine {after[j]:.1f}°" for j in clamped) + ")")

    elif name == "tcp":
        word, mm = m[1], float(m[2])
        if word in DIRECTIONS:
            if mm <= 0:
                step.reason = L(f"use a positive distance with {word}", f"{word} ile pozitif bir mesafe kullanın")
                return
            axis, sign = DIRECTIONS[word]
            mm *= sign
        else:
            axis = word
            word = {"Z": ("UP", "DOWN"), "Y": ("LEFT", "RIGHT"), "R": ("FORWARD", "BACK")}.get(axis, (axis, axis))[0 if mm > 0 else 1]
        if mm == 0:
            step.reason = L("a tool move must be non-zero", "araç hareketi sıfır olamaz")
            return
        requested = mm
        mm = math.copysign(min(abs(mm), MAX_TCP_MM), mm)  # one move is at most MAX_TCP_MM
        try:
            unit = _tcp_delta(before, axis, 1.0, kinematics)
        except ValueError as e:
            step.reason = str(e)
            return
        got, path, cause = _tcp_reach(before, unit, mm, limits, kinematics)
        if got < abs(mm) - 0.5:
            p0 = kinematics.fk(*(before[j] for j in MOVE_JOINTS))
            target = [p0[k] + u * mm for k, u in zip("XYZ", unit)]
            q, pitch_change = find_manoeuvre(before, target, limits, kinematics)
            if q is not None:
                way = L(word.lower(), DIR_TR.get(word, word))
                step.action = {"type": "joint_abs", "targets": q, "label": "manoeuvre",
                               "manoeuvre": {"word": word, "mm": abs(mm), "pitch_change": pitch_change,
                                             "blocked_by": cause},
                               "points": [[p0["X"], p0["Y"], p0["Z"]], target]}
                tilt = L(f", tilting the wrist {pitch_change:+.0f}°", f", bileği {pitch_change:+.0f}° eğerek") \
                    if abs(pitch_change) >= 1 else ""
                step.reason = L(f"a straight line is blocked by {cause}, so reach the same point "
                                f"{abs(mm):g} mm {way} by another path{tilt}",
                                f"düz çizgide {cause} engel oluyor; aynı noktaya ({abs(mm):g} mm {way}) "
                                f"farklı bir yoldan gidiyorum{tilt}")
                sim.pose = dict(q)
                step.ok = True
                return
        if abs(requested) > MAX_TCP_MM and not cause:
            cause = L(f"the {MAX_TCP_MM / 10:g} cm single-move limit", f"tek hareket sınırı ({MAX_TCP_MM / 10:g} santim)")
        way = L(word.lower(), DIR_TR.get(word, word))
        if got < MIN_USEFUL_MM:
            _note(step, L(f"I cannot move {way} at all here: {cause} does not allow it",
                          f"Buradan {way} hiç gidemiyorum: {cause} izin vermiyor"))
            return
        after = {j: path[-1][j] for j in MOVE_JOINTS}
        signed = got if mm > 0 else -got
        p0 = kinematics.fk(*(before[j] for j in MOVE_JOINTS))
        p1 = kinematics.fk(*(after[j] for j in MOVE_JOINTS))
        step.action = {"type": "tcp", "axis": axis, "mm": signed, "requested": requested, "word": word,
                       "cause": cause, "points": [[p0["X"], p0["Y"], p0["Z"]], [p1["X"], p1["Y"], p1["Z"]]]}
        step.reason = (L(f"tool {way} {got:g} mm", f"araç {got:g} mm {way}") +
                       f": X {p0['X']:.0f}→{p1['X']:.0f}, Y {p0['Y']:.0f}→{p1['Y']:.0f}, Z {p0['Z']:.0f}→{p1['Z']:.0f}")
        if got < abs(requested) - 0.5:
            step.reason += L(f" ({cause}: {got:g} of {abs(requested):g} mm)",
                             f" ({cause} nedeniyle {abs(requested):g} yerine {got:g} mm)")

    else:  # goto / home: go as close as the limits allow
        if name == "goto":
            key = find_name(world.points, m[1])
            if key is None:
                known = ", ".join(world.points) or L("none", "yok")
                step.reason = L(f"there is no point named {m[1]!r} (saved points: {known})",
                                f"{m[1]!r} adında kayıtlı nokta yok (kayıtlı noktalar: {known})")
                return
            point = world.points[key]
            values, label = point["joints"], f"point {key}"
            gripper = point.get("gripper") or ""
        else:
            if not world.home or len(world.home) != 6:
                step.reason = L("the home pose is not known", "başlangıç konumu bilinmiyor")
                return
            values, label, gripper = world.home, "home", ""
        wanted = {j: float(values[ALL_JOINTS.index(j)]) for j in MOVE_JOINTS}
        for j in _frozen(kinematics):  # the planner keeps a frozen joint where it is
            if j in wanted:
                wanted[j] = before[j]
        after = {j: _clamp_joint(j, before[j], wanted[j], limits) for j in MOVE_JOINTS}
        clamped = [j for j in MOVE_JOINTS if abs(after[j] - wanted[j]) > 0.05]
        step.action = {"type": "joint_abs", "targets": after, "label": label, "gripper": gripper,
                       "requested": wanted if clamped else None}
        name_tr = "başlangıç konumu" if label == "home" else f"{label[6:]} noktası"
        step.reason = L(f"go to {label}", f"{name_tr}na git") + \
            (L(f", then gripper {gripper}", f", sonra tutucu {'açık' if gripper == 'open' else 'kapalı'}") if gripper else "")
        if clamped:
            step.reason += L(" (as close as the limits allow: " + ", ".join(f"{j} {after[j]:.1f}° instead of {wanted[j]:.1f}°" for j in clamped) + ")",
                             " (sınırların izin verdiği kadar: " + ", ".join(f"{jname(j)} {wanted[j]:.1f}° yerine {after[j]:.1f}°" for j in clamped) + ")")

    sim.pose = after
    step.ok = True


def _check_teach(step, name, m, world, limits, kinematics):
    if name == "save_point":
        exists = find_name(world.points, m[1])
        step.action = {"type": "save_point", "name": m[1]}
        step.reason = L(f"save the current position as {m[1]!r}", f"bu konumu {m[1]!r} olarak kaydet") + \
            (L(" (replaces it)", " (eskisinin yerine)") if exists else "")
        step.ok = True
    elif name == "forget":
        key = find_name(world.skills, m[1])
        if key is None:
            step.reason = L(f"there is no skill named {m[1]!r}", f"{m[1]!r} adında beceri yok")
            return
        step.action = {"type": "forget_skill", "name": key}
        step.reason, step.ok = L(f"forget skill {key!r}", f"{key!r} becerisini unut"), True
    else:
        body = split_skill_body(m[2])
        if not body:
            step.reason = L("a skill needs at least one step", "bir beceri en az bir adım içermeli")
            return
        # Check the form of every step now (pose-independent); positions are
        # checked each time the skill is run.
        blind = World(joints=None, points=world.points, home=world.home, skills={})
        for inner in body:
            inner_name, _ = match(normalize(inner))
            if inner_name in ("run_skill", "define", "forget", "save_point"):
                step.reason = L(f"a skill cannot contain {normalize(inner).split()[0]}",
                                f"beceri {normalize(inner).split()[0]} içeremez")
                return
            if inner_name is None:
                step.reason = L(f"skill step {inner!r} is not a known command", f"beceri adımı {inner!r} bilinen bir komut değil")
                return
            if inner_name == "servo_on":
                step.reason = L("a skill can never turn the servo on", "bir beceri servoyu asla açamaz")
                return
            if inner_name not in ("jrel", "jabs", "tcp", "goto", "home") + tuple(SHAPES):
                check = check_command(inner, blind, limits, _Sim(blind), kinematics, allow_teach=False)
                if not check.ok:
                    step.reason = L(f"skill step {inner!r}: {check.reason}", f"beceri adımı {inner!r}: {check.reason}")
                    return
        step.action = {"type": "define_skill", "name": m[1], "steps": [normalize(c) for c in body]}
        step.reason = L(f"learn skill {m[1]!r}: ", f"{m[1]!r} becerisini öğren: ") + " | ".join(normalize(c) for c in body)
        step.ok = True


# -- shapes ---------------------------------------------------------------------------
# The tool traces a square, rectangle, circle or arc starting at its current
# position. Curves become short straight pieces (the controller cannot do
# circular moves over this link); every piece is simulated like a tool move,
# and a shape that does not fit is scaled down as a whole - never distorted.

SHAPES = ("square", "rect", "circle", "arc")
MAX_SHAPE_MM = 200.0
MIN_SHAPE_MM = 10.0
CURVE_CHORD_MM = 10.0
PIECE_MM = 45.0  # under the planner's 50 mm limit, with margin for rounding


def _shape_axes(pose, plane, kinematics):
    """Unit vectors (a, b): a = forward along the reach (horizontal) or up
    (vertical); b = sideways to the left."""
    p = kinematics.fk(*(pose[j] for j in MOVE_JOINTS))
    radius = math.hypot(p["X"], p["Y"])
    heading = math.atan2(p["Y"], p["X"]) if radius > 1.0 else 0.0
    f = (math.cos(heading), math.sin(heading), 0.0)
    left = (-math.sin(heading), math.cos(heading), 0.0)
    a = (0.0, 0.0, 1.0) if plane == "VERTICAL" else f
    return a, left, p


def _shape_offsets(name, dims, a, b):
    """Offsets from the start point, in mm, start included."""
    def at(u, v):
        return tuple(u * a[i] + v * b[i] for i in range(3))
    if name in ("square", "rect"):
        h, w = (dims[0], dims[0]) if name == "square" else (dims[1], dims[0])
        return [at(0, 0), at(h, 0), at(h, w), at(0, w), at(0, 0)]
    r = dims[0] / 2 if name == "circle" else dims[0]
    sweep = 360.0 if name == "circle" else dims[1]
    n = max(8, min(72, math.ceil(abs(math.radians(sweep)) * r / CURVE_CHORD_MM)))
    pts = []
    for k in range(n + 1):
        t = math.radians(sweep) * k / n
        pts.append(at(r - r * math.cos(t), r * math.sin(t)))  # centre is r ahead (a) of the start
    return pts


def _pieces(offsets):
    """Consecutive moves, each at most PIECE_MM long."""
    out = []
    for p, q in zip(offsets, offsets[1:]):
        d = tuple(q[i] - p[i] for i in range(3))
        length = math.sqrt(sum(x * x for x in d))
        if length < 0.05:
            continue
        n = max(1, math.ceil(length / PIECE_MM - 1e-6))
        out.extend([tuple(x / n for x in d)] * n)
    return out


def _simulate_pieces(before, pieces, limits, kinematics):
    """(ok, end pose, cause) of tracing the pieces from `before`."""
    pose = dict(before)
    for d in pieces:
        try:
            path = kinematics.straight_line(pose, d, step_mm=5.0)
        except kinematics.IKError:
            return False, None, L("the arm's reach", "kolun erişim sınırı")
        ok, jn = _path_ok(path, before, limits)
        if not ok:
            return False, None, L(f"the {jn} limit", f"{jname(jn)} sınırı")
        pose = {j: path[-1][j] for j in MOVE_JOINTS}
    return True, pose, None


def shape_label(name, dims, plane, lang):
    tr = lang == "tr"
    plane_w = ("dikey" if plane == "VERTICAL" else "yatay") if tr else ("vertical" if plane == "VERTICAL" else "horizontal")
    def cm(mm):
        return f"{mm / 10:g} santim" if tr else f"{mm / 10:g} cm"
    if name == "square":
        return f"{cm(dims[0])} kenarlı {plane_w} kare" if tr else f"{plane_w} square with {cm(dims[0])} sides"
    if name == "rect":
        return f"{cm(dims[0])} x {cm(dims[1])} {plane_w} dikdörtgen" if tr else f"{plane_w} {cm(dims[0])} x {cm(dims[1])} rectangle"
    if name == "circle":
        return f"{cm(dims[0])} çaplı {plane_w} daire" if tr else f"{plane_w} circle, {cm(dims[0])} across"
    angle = abs(dims[1])
    side = ("sola" if dims[1] > 0 else "sağa") if tr else ("left" if dims[1] > 0 else "right")
    return f"{cm(dims[0])} yarıçaplı {angle:g} derecelik {plane_w} yay ({side})" if tr else \
        f"{plane_w} {angle:g}° arc, radius {cm(dims[0])}, curving {side}"


PITCH_TRIES = (0, 10, -10, 20, -20, 30, -30, 45, -45, 60, -60, 90, -90)


def _in_band(q, before, limits):
    return all(_band(j, before[j], limits)[0] - 1e-6 <= q[j] <= _band(j, before[j], limits)[1] + 1e-6
               for j in MOVE_JOINTS)


def find_manoeuvre(before, target, limits, kinematics):
    """A joint configuration that puts the tool exactly at `target` (mm, base
    frame) within the joint bands, trying other wrist pitches, the other elbow
    and reaching over the other side. Returns (joints, pitch change) or
    (None, 0). MoveIt then plans the (non-straight) path to it."""
    pitch0 = kinematics.fk(*(before[j] for j in MOVE_JOINTS))["B"]
    for dp in PITCH_TRIES:
        try:
            q = kinematics.ik(target[0], target[1], target[2], pitch0 + dp, before, ref=before)
        except kinematics.IKError:
            continue
        q = {j: float(q[j]) for j in MOVE_JOINTS}
        if _in_band(q, before, limits):
            return q, float(dp)
    return None, 0.0


SHIFT_DIRS = ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1),
              (1, 1, 0), (1, -1, 0), (-1, 1, 0), (-1, -1, 0))
SHIFT_STEPS_MM = (20, 40, 60, 80, 100, 120)


def _shape_start_shift(before, name, dims, a, b, limits, kinematics):
    """Smallest move (mm vector) to a starting point from which the full shape
    fits, reachable by a straight tool move. Returns (shift, pieces) or (None, None)."""
    offsets = _shape_offsets(name, dims, a, b)
    shape_pieces = _pieces(offsets)
    up = (0.0, 0.0, 1.0)
    best = None
    for dist in SHIFT_STEPS_MM:
        for da, db, dz in SHIFT_DIRS:
            n = math.sqrt(da * da + db * db + dz * dz)
            shift = tuple(dist / n * (da * a[i] + db * b[i] + dz * up[i]) for i in range(3))
            pre = _pieces([(0.0, 0.0, 0.0), shift])
            ok, _end, _cause = _simulate_pieces(before, pre + shape_pieces, limits, kinematics)
            if ok:
                best = (shift, pre)
                break
        if best:
            return best
    return None, None


def _check_shape(step, name, m, limits, sim, kinematics):
    if sim.pose is None:
        step.reason = L("the robot's position is unknown (not connected?)", "robotun konumu bilinmiyor (bağlı değil mi?)")
        return
    if name in ("square", "circle"):
        dims, plane = [abs(float(m[1]))], m[2] or "HORIZONTAL"
    elif name == "rect":
        dims, plane = [abs(float(m[1])), abs(float(m[2]))], m[3] or "HORIZONTAL"
    else:
        dims, plane = [abs(float(m[1])), float(m[2])], m[3] or "HORIZONTAL"
        if not 1 <= abs(dims[1]) <= 360:
            step.reason = L("an arc must be 1-360 degrees", "yay 1-360 derece olmalı")
            return
    size_dims = dims if name != "arc" else dims[:1]
    if min(size_dims) <= 0:
        step.reason = L("the shape needs a size", "şeklin bir boyutu olmalı")
        return
    requested = list(dims)
    over = max(size_dims) / MAX_SHAPE_MM
    if over > 1:  # one shape is at most MAX_SHAPE_MM across
        dims = [d / over for d in size_dims] + dims[len(size_dims):]
    before = dict(sim.pose)
    a0, b0, p0 = _shape_axes(before, plane, kinematics)
    n_size = len(size_dims)
    # No side was given, so a square/rectangle/circle may be drawn mirrored (to
    # the right, or from its far edge) when that is what fits. An arc keeps the
    # turning direction it was given.
    flips = [(1, 1), (1, -1), (-1, 1), (-1, -1)] if name != "arc" else [(1, 1), (-1, 1)]
    variants = [(tuple(fa * x for x in a0), tuple(fb * x for x in b0)) for fa, fb in flips]
    a, b = variants[0]

    def trial(sizes, axes=None):
        ax, bx = axes or (a, b)
        full = list(sizes) + dims[n_size:]
        offsets = _shape_offsets(name, full, ax, bx)
        ok, end, cause = _simulate_pieces(before, _pieces(offsets), limits, kinematics)
        return ok, end, cause, full, offsets

    ok, end, cause, final, offsets = trial(dims[:n_size])
    for axes in variants[1:]:
        if ok:
            break
        r = trial(dims[:n_size], axes)
        if r[0]:
            a, b = axes
            ok, end, _, final, offsets = r
    shift = None
    if not ok:
        for axes in variants:  # full size from a nearby start point?
            sh, pre = _shape_start_shift(before, name, dims, axes[0], axes[1], limits, kinematics)
            if sh is not None:
                a, b = axes
                offsets = [tuple(sh)] + [tuple(o[i] + sh[i] for i in range(3)) for o in _shape_offsets(name, dims, a, b)]
                ok_full, end, _ = _simulate_pieces(before, pre + _pieces(offsets[1:]), limits, kinematics)
                if ok_full:
                    ok, final, shift = True, list(dims), sh
                    offsets = [(0.0, 0.0, 0.0)] + offsets
                    break
    if not ok:
        best_scale, best_axes = 0.0, variants[0]
        for axes in variants:  # largest scale that fits, over every orientation
            lo, hi = 0.0, 1.0
            for _ in range(12):
                mid = (lo + hi) / 2
                if trial([d * mid for d in dims[:n_size]], axes)[0]:
                    lo = mid
                else:
                    hi = mid
            if lo > best_scale:
                best_scale, best_axes = lo, axes
        a, b = best_axes
        sizes = [math.floor(d * best_scale) for d in dims[:n_size]]
        if max(sizes) < MIN_SHAPE_MM:
            _note(step, L(f"there is no room here for that shape: {cause} does not allow it",
                          f"Burada bu şekil için yer yok: {cause} izin vermiyor"))
            return
        ok, end, _, final, offsets = trial(sizes)
        if not ok:  # rounding can only shrink it, but never trust that blindly
            _note(step, L("there is no room here for that shape", "Burada bu şekil için yer yok"))
            return
    elif over > 1:
        cause = L(f"the {MAX_SHAPE_MM / 10:g} cm shape limit", f"en fazla {MAX_SHAPE_MM / 10:g} santimlik şekil sınırı")
    pieces = _pieces(offsets)
    points = [[p0["X"] + o[0], p0["Y"] + o[1], p0["Z"] + o[2]] for o in offsets]
    step.action = {"type": "path", "shape": name, "dims": final, "requested": requested, "plane": plane,
                   "pieces": [list(d) for d in pieces], "points": points,
                   "shift_mm": [round(x, 1) for x in shift] if shift else None,
                   "cause": cause if final[:len(size_dims)] != requested[:len(size_dims)] else None}
    step.reason = shape_label(name, final, plane, LANG) + L(f" ({len(pieces)} straight pieces)", f" ({len(pieces)} düz parça)")
    if shift:
        d = math.sqrt(sum(x * x for x in shift))
        step.reason = L(f"first move {d:.0f} mm to where it fits, then ", f"önce sığacağı yere {d:.0f} mm gidip ") + step.reason
    if step.action["cause"]:
        step.reason += L(f" ({step.action['cause']}: {shape_label(name, requested, plane, 'en')} asked)",
                         f" ({step.action['cause']} nedeniyle; istenen: {shape_label(name, requested, plane, 'tr')})")
    sim.pose = end
    step.ok = True


def validate(commands, world, limits=None, kinematics=None):
    """Validate a model-produced command list against the live world. A
    command that is not allowed rejects the whole plan (never retried).
    Motion that would cross a joint limit or leave the reachable space is not
    rejected: it is shortened to what is allowed, and the step says so."""
    if kinematics is None:
        from rv2aj_control import kinematics
    limits = limits or DEFAULT_LIMITS
    plan = Plan()
    if not isinstance(commands, list) or not commands:
        plan.reason = L("no commands", "komut yok")
        return plan
    sim = _Sim(world)
    flat = []
    for source in commands:
        # "A | B" outside DEFINE_SKILL is two commands written on one line.
        if isinstance(source, str) and "|" in source and not source.strip().upper().startswith("DEFINE_SKILL"):
            flat.extend(split_skill_body(source))
        else:
            flat.append(source)
    for source in flat:
        cmd = normalize(source) if isinstance(source, str) else ""
        name, m = match(cmd)
        if name == "run_skill":
            key = find_name(world.skills, m[1])
            if key is None:
                known = ", ".join(world.skills) or L("none", "yok")
                plan.steps.append(Step(source=source, command=cmd, reason=L(
                    f"there is no skill named {m[1]!r} (known skills: {known})",
                    f"{m[1]!r} adında beceri yok (bilinen beceriler: {known})")))
                continue
            for inner in world.skills[key]:
                st = check_command(inner, world, limits, sim, kinematics, allow_teach=False)
                st.skill = key
                plan.steps.append(st)
        else:
            plan.steps.append(check_command(source, world, limits, sim, kinematics))
    if len(plan.steps) > MAX_PLAN_STEPS:
        plan.reason = L(f"{len(plan.steps)} steps is more than the {MAX_PLAN_STEPS}-step limit",
                        f"{len(plan.steps)} adım, {MAX_PLAN_STEPS} adım sınırından fazla")
        return plan
    bad = [st for st in plan.steps if not st.ok]
    if bad:
        first = bad[0]
        plan.reason = f"“{first.command or first.source}”: {first.reason}"
        if first.skill:
            plan.reason = L(f"skill {first.skill}: ", f"{first.skill} becerisi: ") + plan.reason
        if len(bad) > 1:
            plan.reason += L(f" (and {len(bad) - 1} more)", f" (ve {len(bad) - 1} tane daha)")
        return plan
    plan.ok = True
    plan.reason = L("all commands passed", "tüm komutlar uygun")
    plan.effects = [st.reason for st in plan.steps if st.kind in (Kind.MOTION, Kind.ACTUATE, Kind.TEACH)
                    or (st.action or {}).get("type") == "note"]
    return plan


# -- teaching memory ----------------------------------------------------------------

_STOP = set("a an the to of and then please robot arm now it by me my for with can you could".split())


def _tokens(text):
    words = re.findall(r"[a-z0-9çğıöşü]+", tr_lower(text))
    return {w for w in words if w not in _STOP}


def _sim_tokens(text):
    """Tokens for example matching: words, plus "#" for any number, so
    "ascend 35 mm" is close to "move up 20 mm"."""
    t = _tokens(text)
    return {("#" if w.isdigit() else w) for w in t}


def _overlap(q, t):
    """Shared words; a word that shares its first 4 letters with one in the
    other text counts partly (Turkish suffixes: "yukarıya" ~ "yukarı")."""
    exact = q & t
    score = float(len(exact))
    for w in q - exact:
        if len(w) >= 4 and any(len(x) >= 4 and x[:4] == w[:4] for x in t - exact):
            score += 0.6
    return score


class Memory:
    """Skills and examples the model learns from.

    seed examples: shipped with the package (written and checked by hand)
    learned examples: every plan the operator confirmed, and corrections
    skills: taught with DEFINE_SKILL
    """

    def __init__(self, path, seed_examples=()):
        self.path = os.path.expanduser(path)
        self.seed = [dict(e, source="seed") for e in seed_examples]
        self.skills = {}
        self.learned = []
        self._lock = threading.Lock()
        self.load()

    def load(self):
        try:
            with open(self.path) as f:
                data = json.load(f)
            self.skills = {k: list(v) for k, v in data.get("skills", {}).items()}
            self.learned = list(data.get("examples", []))
        except (OSError, ValueError):
            self.skills, self.learned = {}, []

    def save(self):
        with self._lock:
            os.makedirs(os.path.dirname(self.path) or ".", exist_ok=True)
            tmp = self.path + ".tmp"
            with open(tmp, "w") as f:
                json.dump({"skills": self.skills, "examples": self.learned}, f, indent=2)
            os.replace(tmp, self.path)

    def add_example(self, request, commands, source):
        request = request.strip()
        self.learned = [e for e in self.learned if e["request"].lower() != request.lower()]
        self.learned.append({"request": request, "commands": list(commands), "source": source,
                             "time": datetime.now().isoformat(timespec="seconds")})
        self.save()

    def define_skill(self, name, steps):
        key = find_name(self.skills, name) or name
        self.skills[key] = list(steps)
        self.save()
        return key

    def forget_skill(self, name):
        key = find_name(self.skills, name)
        if key:
            del self.skills[key]
            self.save()
        return key

    @staticmethod
    def _key(text):
        return " ".join(re.findall(r"[a-z0-9çğıöşü]+", tr_lower(text)))

    def recall(self, request):
        """Exact recall: a phrase the operator taught (or confirmed) always maps
        to the same commands, without asking the model."""
        key = self._key(request)
        # Only explicitly taught phrases and the hand-checked seed examples are
        # recalled verbatim. Confirmed plans are hints for the model, not rules:
        # a slightly wrong plan confirmed once must not be repeated forever.
        for e in reversed([x for x in self.learned if x.get("source") == "taught"] + self.seed):
            if self._key(e["request"]) == key and e["commands"]:  # chat is answered freshly, not canned
                return e
        return None

    def similar(self, request, k=8):
        """The k examples closest to the request (learned ones win ties)."""
        q = _sim_tokens(request)
        scored = []
        for i, e in enumerate(self.seed + self.learned):
            t = _sim_tokens(e["request"])
            if not t:
                continue
            score = _overlap(q, t) / math.sqrt(len(q) * len(t)) if q else 0.0
            if e.get("source") != "seed":
                score += 0.05
            scored.append((score, i, e))
        # Ties (and requests like nothing seen before) fall back to the first,
        # basic examples of the file, never to whatever was added last.
        scored.sort(key=lambda s: (-s[0], s[1]))
        return [e for _, _, e in scored[:k]]


# -- prompt ---------------------------------------------------------------------------

def system_prompt(limits):
    lim = ", ".join(f"{j} {lo:g}..{hi:g}" for j, (lo, hi) in limits.items())
    return f"""You are the friendly Turkish-speaking assistant of a Mitsubishi RV-2AJ robot arm. You chat naturally with the operator like a helpful colleague AND you control the arm.
Every message may be small talk, a question, a command, or a mix. Answer the human part in "say" and put any robot actions in "commands".

HOW TO ANSWER:
  "say": warm, natural, spoken Turkish, 1-3 short sentences (it is read aloud). Answer questions, explain, joke a little if the operator does.
  Your style is like JARVIS: calm, capable, polite and a little witty. ALWAYS address the operator as "hocam"
  (e.g. "Tabii hocam!", "Merhaba hocam, nasılsınız?", "Hemen bakıyorum hocam.").
  "commands": the robot actions the operator asked for, or [] for pure conversation.
  When you send commands, keep "say" to a short friendly remark ("Tabii!", "Hemen."). The robot itself announces exactly what it will do and what it did.
  NEVER claim you moved, opened, closed or finished anything in "say": actions only happen through commands, and the robot reports the result.
  You do NOT know the live robot state. For position, angles, servo, alarm or gripper questions send WHERE or STATUS; the robot answers with real data.
  To explain earlier results ("neden?", "ne oldu?", "az önce ne yaptın?") use the notes about what the robot really did.
  Answer the NEW message; never repeat an earlier answer word for word.
  Use WAIT only when the operator asks to wait a number of seconds; "dinlenelim"/"mola" is just conversation.

ABOUT THE ROBOT (for questions):
  Mitsubishi RV-2AJ, 5 joints: J1 base, J2 shoulder, J3 elbow, J5 wrist tilt, J6 wrist roll (there is NO J4). Controller CR1-571 over an RS-232 cable.
  Festo HGP-16 pneumatic parallel gripper, grips with up to 160 N. Works next to a Festo MPS 500 transfer line (station 4).
  Every move is planned and collision-checked with MoveIt, limited to safe joint ranges, and needs the operator's confirmation. Speed is limited to 30 percent.
  You can move the tool up/down/left/right/forward/back, turn joints, tilt the wrist, open/close the gripper, go to saved points or home,
  draw squares, rectangles, circles and arcs, save positions, and learn new skills. You cannot turn the servo on (the operator does that) or run the conveyor.

The operator speaks Turkish. ALWAYS write "say" in Turkish, never in English. Commands are always the English keywords below.
For robot requests never ask a question back: choose the most likely meaning and act on it.
If no distance is given use 20 mm (2 santim); if no angle is given use 10 degrees.
Always send motion even if it may reach a limit: the robot automatically stops at the limit and reports how far it went.
Shapes: kare = DRAW_SQUARE, dikdörtgen = DRAW_RECTANGLE, daire/çember/yuvarlak = DRAW_CIRCLE, yay/yarım daire/çeyrek daire = DRAW_ARC.
"dikey" = VERTICAL; otherwise horizontal. No size given = 50 mm. "10 santimlik daire" = diameter 100. "yarıçapı 5 santim" = diameter 100.
This is a conversation: "biraz daha" / "a bit more" = the same direction as the previous move, 10 mm or 5 degrees;
"tekrar" / "bir daha" / "again" = repeat the previous commands; "geri al" / "undo" = the opposite of the previous move.
Reply ONLY with JSON: {{"say": "<one short sentence>", "commands": ["<command>", ...]}}
Use an empty list when the request is a question you cannot answer with a command, or is not possible; explain in "say".

COMMANDS:
  WHERE                                report joint angles and tool position
  STATUS                               report connection, servo, alarm, gripper
  LIST_POINTS / LIST_SKILLS            list saved positions / taught skills
  STOP                                 stop
  SERVO_OFF                            turn the servo off
  WAIT <seconds>                       pause (max {MAX_WAIT_S:g})
  GRIPPER OPEN / GRIPPER CLOSE         open / close the gripper
  SPEED <1-{MAX_OVERRIDE}>                        speed override in percent
  MOVE_TCP <UP|DOWN|LEFT|RIGHT|FORWARD|BACK> <mm>   move the tool in a straight line (positive mm, max {MAX_TCP_MM:g})
  MOVE_JOINT_REL J<n> <+/-degrees>     turn one joint by an amount (max {MAX_JOINT_REL_DEG:g})
  TILT_WRIST <UP|DOWN> <degrees>       tilt the wrist (J5) up or down
  MOVE_JOINT_ABS J<n>=<deg> ...        turn joints to absolute angles
  GOTO_POINT <name>                    move to a saved position
  GO_HOME                              move to the home position
  SAVE_POINT <name>                    remember the current position under a name
  DRAW_SQUARE <side mm> [VERTICAL]               trace a square with the tool, starting here (default horizontal)
  DRAW_RECTANGLE <width mm> <height mm> [VERTICAL] trace a rectangle
  DRAW_CIRCLE <diameter mm> [VERTICAL]           trace a circle (a radius of r means diameter 2r)
  DRAW_ARC <radius mm> <degrees> LEFT|RIGHT [VERTICAL]   trace an arc curving left or right; half circle = 180, quarter = 90
  LINE_STATUS                          read the Festo transfer line: order from Station 4 and our signals
  LINE_READY ON|OFF                    tell Station 4 the robot station is ready / not ready
  LINE_BUSY ON|OFF                     tell Station 4 an order is being worked on / finished
  LINE_ERROR_CODE <0-3>                report an error code to Station 4 (0 = no error)
  RUN_SKILL <name>                     do a taught skill
  DEFINE_SKILL <name>: <cmd> | <cmd>   learn a new skill (steps separated by |)
  FORGET_SKILL <name>                  forget a skill

MEANING:
  UP = up, lift, raise, higher. DOWN = down, lower, drop, descend. LEFT/RIGHT = sideways (never used for forward/back).
  FORWARD = out along the arm's reach: forward, extend, reach out, outward, away from the base.
  BACK = back toward the base: back, retract, pull in, closer to the base, toward the base.
  "cm" / "santim" means 10 mm. "a bit" / "biraz" = 10 mm or 5 degrees. "a bit", "a little", "slightly" = 10 mm or 5 degrees. "a lot" = 50 mm or 30 degrees.
  "rotate/turn/spin the base" = J1. "shoulder" = J2. "elbow" = J3. "twist/roll/rotate the gripper or wrist" = J6.
  Up/down/left/right/forward/back on their own always move the TOOL with MOVE_TCP ("biraz yukarı" = MOVE_TCP UP 10).
  Only when the operator says wrist/bilek (or tilt/eğ) use TILT_WRIST UP/DOWN (never J5 or J6 directly).
  "turn/move a joint BY <angle>" = MOVE_JOINT_REL. "put/set a joint AT or TO <angle>" = MOVE_JOINT_ABS.
  Copy every number exactly as the operator said it (only convert cm to mm). Never invent a skill or point name.
  The conveyor/belt is not controlled by the robot: answer with an empty list.
  Orders: sipariş/istasyon/hat questions = LINE_STATUS. Orders are: none, assemble (montaj), sorting out (ayıklama),
  take part from pallet (paletten parça al).
  "save / store / record / remember this position or pose as <name>" = SAVE_POINT <name>.
  Only use DEFINE_SKILL when the operator says "skill", "learn", or "when I say".
  One command per list item; "twice" means repeat the command.
  "grab", "grip", "pick", "hold" = GRIPPER CLOSE. "release", "drop", "let go", "open the jaws" = GRIPPER OPEN.
  "speed", "percent", "run at N percent", "faster", "slower" = SPEED (not a skill).
  You do NOT know the robot's current state. Questions about position, angles, status, alarm, servo or gripper
  are answered ONLY by WHERE or STATUS. Never claim something is "already" done; just send the command.
  Use exactly the numbers the operator says. Do only what was asked, in the order asked.
  Use the saved point and skill names exactly as listed in the status.

NEVER: turn the servo on (the operator does that), exceed speed {MAX_OVERRIDE}.
Joint limits (degrees): {lim}."""


_KNOWLEDGE = None


def knowledge_text():
    """config/ai_knowledge.txt: facts about the robot, the cell and this
    software, read by the fine-tuned model to answer questions. Editable
    without retraining."""
    global _KNOWLEDGE
    if _KNOWLEDGE is None:
        paths = [os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config", "ai_knowledge.txt")]
        try:
            from ament_index_python.packages import get_package_share_directory
            paths.insert(0, os.path.join(get_package_share_directory("rv2aj_control"), "config", "ai_knowledge.txt"))
        except Exception:
            pass
        _KNOWLEDGE = ""
        for path in paths:
            if os.path.exists(path):
                with open(path) as f:
                    _KNOWLEDGE = f.read().strip()
                break
    return _KNOWLEDGE


_QUESTION = re.compile(r"\b(ne|neden|niye|niçin|nasıl|kaç|nedir|hangi|kim|nerede|ne zaman|anlat|açıkla|bilgi|"
                       r"what|why|how|which|who|explain)\b", re.I)
_COMMAND_HINT = re.compile(
    r"\d|\b(git|gid\w*|çık\w*|in|in[ie]\w*|kay\w*|getir\w*|çiz\w*|kapat\w*|aç|açar\w*|tut|tutar\w*|bırak\w*|"
    r"döndür\w*|çevir\w*|eğ|eğer\w*|kaydet\w*|bekle\w*|yap|yapar\w*|dön|kaldır\w*|indir\w*|uzat\w*|uzan\w*|"
    r"ilerle\w*|götür\w*|oynat\w*|ayarla\w*|koy|sık|yakala\w*|çek\w*|gel|al|yukarı\w*|aşağı\w*|sola|sağa|ileri\w*|"
    r"geri|santim\w*|derece\w*|yüzde|hız\w*|move|go|draw|open|close|turn|rotate|lift|lower|grab|release)\b", re.I)


def is_question(request):
    """A question or small talk, not a command ("tutucun ne?", "ara verelim", "görüşürüz")."""
    t = tr_lower(request)
    return bool(_QUESTION.search(t) or re.search(r"\?|\bm[ıiuü](s[ıiuü]n|y[ıiuü]z)?\b", t)) and \
        not _COMMAND_HINT.search(re.sub(r"\b(nasıl|ne zaman)\b.*", "", t)) or not _COMMAND_HINT.search(t)


def wants_knowledge(request):
    """Questions and small talk get the fact sheet; commands do not need it
    (a polite "gider misin?" is still a command)."""
    t = tr_lower(request)
    return bool(_QUESTION.search(t)) or not _COMMAND_HINT.search(t)


def system_prompt_compact(limits, knowledge=True):
    """Short prompt for the fine-tuned model (rv2aj-qwen): it learned the
    commands and the operator's language from thousands of examples, so it
    needs only the format and the command list. Must match finetune/ data."""
    lim = ", ".join(f"{j} {lo:g}..{hi:g}" for j, (lo, hi) in limits.items())
    return f"""You are the Turkish-speaking assistant of a Mitsubishi RV-2AJ robot arm (5 joints: J1 base, J2 shoulder, J3 elbow, J5 wrist tilt, J6 wrist roll; no J4; Festo gripper; Festo MPS 500 station 4).
Reply ONLY with JSON {{"say": "...", "commands": [...]}}. "say": short natural Turkish, address the operator as "hocam", never claim an action is done. "commands": robot actions or [].
Commands: WHERE, STATUS, LIST_POINTS, LIST_SKILLS, LINE_STATUS, SERVO_OFF, WAIT <s>, GRIPPER OPEN|CLOSE, SPEED <1-{MAX_OVERRIDE}>, MOVE_TCP UP|DOWN|LEFT|RIGHT|FORWARD|BACK <mm>, MOVE_JOINT_REL J<n> <+-deg>, TILT_WRIST UP|DOWN <deg>, MOVE_JOINT_ABS J<n>=<deg> ..., GOTO_POINT <name>, GO_HOME, SAVE_POINT <name>, DRAW_SQUARE <mm> [VERTICAL], DRAW_RECTANGLE <w> <h> [VERTICAL], DRAW_CIRCLE <diameter> [VERTICAL], DRAW_ARC <radius> <deg> LEFT|RIGHT [VERTICAL], LINE_READY ON|OFF, LINE_BUSY ON|OFF, LINE_ERROR_CODE <0-3>, RUN_SKILL <name>, DEFINE_SKILL <name>: <cmd> | <cmd>, FORGET_SKILL <name>.
Never turn the servo on. The conveyor is not controlled by the robot. Joint limits: {lim}.""" + (
        "\nAnswer questions about the robot, the cell and yourself from these facts (explain simply, like an expert "
        "colleague; if something is not in the facts and you are not sure, say so):\n" + knowledge_text()
        if knowledge and knowledge_text() else "")


def status_text(world, tcp=None):
    # Deliberately no joint angles, servo or gripper state: a small model
    # answers from whatever it is shown instead of reading the robot. The
    # validator has the live pose; the model must use WHERE / STATUS.
    head = "Robot: connected." if world.connected else "Robot: not connected."
    points = ", ".join(world.points) or "none"
    skills = "; ".join(f"{k}: {' | '.join(v)}" for k, v in world.skills.items()) or "none"
    return f"{head}\nSaved points: {points}.\nTaught skills: {skills}."


CONTINUATION_DEMO = [
    ("yukarı çık", {"say": "20 milimetre yukarı çıkıyorum.", "commands": ["MOVE_TCP UP 20"]}),
    ("biraz daha", {"say": "Biraz daha yukarı çıkıyorum.", "commands": ["MOVE_TCP UP 10"]}),
    ("tabanı 15 derece döndür", {"say": "Tabanı 15 derece döndürüyorum.", "commands": ["MOVE_JOINT_REL J1 +15"]}),
    ("tekrar", {"say": "Tabanı yine 15 derece döndürüyorum.", "commands": ["MOVE_JOINT_REL J1 +15"]}),
    ("3 santim sola git", {"say": "30 milimetre sola gidiyorum.", "commands": ["MOVE_TCP LEFT 30"]}),
    ("geri al", {"say": "Son hareketi geri alıyorum: 30 milimetre sağa.", "commands": ["MOVE_TCP RIGHT 30"]}),
]


def build_messages(request, world, limits, memory, tcp=None, k_examples=8, history=(), compact=False):
    """System prompt, then the closest examples as earlier turns (small models
    follow worked examples far better than rules), then the request.
    compact: the fine-tuned model's format (short prompt, no examples)."""
    messages = [{"role": "system", "content": system_prompt_compact(limits, knowledge=wants_knowledge(request))
                 if compact else system_prompt(limits)}]
    for e in [] if compact else reversed(memory.similar(request, k_examples)):  # most similar last, nearest the request
        say = e.get("say", "Tamam.")
        if LANG == "tr" and not is_turkish(say):
            say = "Tamam."  # keep the model's replies Turkish
        messages.append({"role": "user", "content": e["request"]})
        messages.append({"role": "assistant", "content": json.dumps(
            {"say": say, "commands": e["commands"]}, ensure_ascii=False)})
    if history and is_continuation(request) and not compact:
        # Show how "biraz daha" / "tekrar" / "geri al" follow from the turn before.
        for req, answer in CONTINUATION_DEMO:
            messages.append({"role": "user", "content": req})
            messages.append({"role": "assistant", "content": json.dumps(answer, ensure_ascii=False)})
    for turn in history:  # the real conversation so far, oldest first
        messages.append({"role": "user", "content": turn["request"]})
        messages.append({"role": "assistant", "content": json.dumps(
            {"say": turn.get("say") or "Tamam.", "commands": turn["commands"]}, ensure_ascii=False)})
    # What the robot really did in those turns, so "neden?" / "ne oldu?" get true answers.
    results = [f"- “{t['request']}” → {t['result']}" for t in history
               if t.get("result") and t.get("commands") and t["result"] != t.get("say")]
    notes = ("\nWhat the robot really did earlier (use it only to answer questions about it):\n" + "\n".join(results[-3:])
             if results else "")
    messages.append({"role": "user", "content": f"{status_text(world, tcp)}{notes}\nRequest: {request}"})
    return messages


RESPONSE_SCHEMA = {
    "type": "object",
    "properties": {"say": {"type": "string"}, "commands": {"type": "array", "items": {"type": "string"}}},
    "required": ["say", "commands"],
}


class LLMError(Exception):
    pass


class Ollama:
    def __init__(self, model="rv2aj-qwen", url="http://127.0.0.1:11434", timeout=120):
        self.model, self.url, self.timeout = model, url.rstrip("/"), timeout

    def check(self):
        import requests
        try:
            r = requests.get(f"{self.url}/api/tags", timeout=3)
            names = {m.get("name") for m in r.json().get("models", [])}
        except Exception:
            return False, f"Ollama not reachable at {self.url}"
        if self.model not in names and f"{self.model}:latest" not in names:  # Ollama lists "rv2aj-qwen" as "rv2aj-qwen:latest"
            return False, f"model {self.model} not pulled"
        return True, f"{self.model} ready"

    def warm_up(self):
        import requests
        try:
            requests.post(f"{self.url}/api/generate", json={"model": self.model, "keep_alive": "30m"},
                          timeout=self.timeout)
        except Exception:
            pass

    def chat(self, messages):
        """Return (raw, parsed). Raises LLMError; never retries."""
        import requests
        # A JSON-schema grammar makes Ollama emit the keys alphabetically ("commands" before
        # "say"). The fine-tuned model was trained to answer first and then act, so it gets
        # plain JSON mode and keeps its own order (measured: stray commands on questions vanish).
        fmt = "json" if self.model.startswith("rv2aj") else RESPONSE_SCHEMA
        body = {"model": self.model, "messages": messages, "stream": False, "format": fmt,
                "keep_alive": "30m", "options": {"temperature": 0, "num_ctx": 4096, "num_predict": 300}}
        if self.model.startswith(("qwen3", "deepseek-r1")) and "instruct" not in self.model:
            body["think"] = False  # no hidden reasoning: fast, and the JSON is the whole answer
        try:
            r = requests.post(f"{self.url}/api/chat", json=body, timeout=self.timeout)
        except Exception as e:
            raise LLMError(f"Ollama not reachable: {e}") from e
        if r.status_code != 200:
            raise LLMError(f"Ollama error {r.status_code}: {r.text[:200]}")
        raw = (r.json().get("message") or {}).get("content", "")
        try:
            parsed = json.loads(raw)
        except ValueError as e:
            raise LLMError(f"model output was not JSON: {raw[:200]!r}") from e
        if isinstance(parsed, dict) and "commands" not in parsed and "say" in parsed:
            parsed["commands"] = []  # plain JSON mode: a pure answer may leave the list out
        if not isinstance(parsed, dict) or not isinstance(parsed.get("commands"), list):
            raise LLMError(f"model output has no command list: {raw[:200]!r}")
        parsed["say"] = str(parsed.get("say", "")).strip()
        return raw, parsed


# -- intent guards ------------------------------------------------------------------
# Deterministic cross-checks between the operator's words and the plan. A small
# model sometimes turns "10" into 100 or picks a skill nobody asked for; these
# reject such plans instead of letting them reach the confirmation dialog.

_UNITS = {"zero": 0, "one": 1, "two": 2, "three": 3, "four": 4, "five": 5, "six": 6, "seven": 7,
          "eight": 8, "nine": 9, "ten": 10, "eleven": 11, "twelve": 12, "thirteen": 13, "fourteen": 14,
          "fifteen": 15, "sixteen": 16, "seventeen": 17, "eighteen": 18, "nineteen": 19}
_TENS = {"twenty": 20, "thirty": 30, "forty": 40, "fifty": 50, "sixty": 60, "seventy": 70,
         "eighty": 80, "ninety": 90}
VAGUE_WORDS = {"bit", "little", "slightly", "tiny", "touch", "nudge", "lot", "much", "far", "some"}
VAGUE_AMOUNTS = {5.0, 10.0, 20.0, 30.0, 50.0}  # incl. the 20 mm / 10° defaults
MAX_WORDS = {"sonuna", "maksimum", "max", "maximum", "possible", "far", "sınıra", "sınırına", "gidebildiğin",
             "gidebildiğince", "all"}  # "all the way" / "sonuna kadar": the longest allowed move
FULL_SPEED_WORDS = {"full", "max", "maximum", "fastest", "top"}
SPEED_DEFAULTS = {30.0}
DEFAULT_WAITS = {0.5, 1.0}


TR_HINTS = {"derece", "santim", "santimetre", "milimetre", "yukarı", "aşağı", "sola", "sağa", "ileri", "geri",
            "bilek", "bileği", "taban", "tabanı", "döndür", "çevir", "kaldır", "indir", "aç", "kapat", "tut",
            "bırak", "git", "hareket", "eğ", "hız", "hızı", "yüzde", "kaydet", "nerede", "konum", "dirsek", "omuz",
            "biraz", "sonra", "ve", "ile", "bekle", "eve", "noktaya", "beceri", "öğren", "servo", "tutucu"}
_TR_UNITS = {"sıfır": 0, "bir": 1, "iki": 2, "üç": 3, "dört": 4, "beş": 5, "altı": 6, "yedi": 7, "sekiz": 8,
             "dokuz": 9}
_TR_TENS = {"on": 10, "yirmi": 20, "otuz": 30, "kırk": 40, "elli": 50, "altmış": 60, "yetmiş": 70,
            "seksen": 80, "doksan": 90}


def tr_lower(text):
    return text.replace("I", "ı").replace("İ", "i").lower()


TR_STEMS = ("kare", "daire", "çember", "yuvarlak", "dikdörtgen", "çiz", "derece", "santim", "milim", "eklem", "getir", "yukar", "aşağ", "asag", "ileri", "geri", "sola",
            "sağa", "bilek", "taban", "döndür", "çevir", "kaldır", "indir", "kapat", "bırak", "hareket", "konum",
            "nokta", "beceri", "öğren", "tutucu", "yüzde", "hız", "dirsek", "omuz", "omz", "bekle", "kaydet")


def is_turkish(text):
    t = tr_lower(text)
    if re.search(r"[çğıöşü]", t):
        return True
    words = set(re.findall(r"\w+", t))
    return bool(words & TR_HINTS) or any(w.startswith(stem) for w in words for stem in TR_STEMS)


def _words(text):
    return re.findall(r"[a-zçğıöşü]+", tr_lower(text))


_FOLD = str.maketrans("çğıöşüâîû", "cgiosuaiu")


def fold(text):
    """Turkish letters to ASCII: speech recognition sometimes drops them ("bes bucuk", "basa don")."""
    return tr_lower(text).translate(_FOLD)


_TR_UNITS_F = {fold(k): v for k, v in _TR_UNITS.items()}
_TR_TENS_F = {fold(k): v for k, v in _TR_TENS.items()}
_TR_ORDINALS_F = {fold(k): v for k, v in {
    "birinci": 1, "ikinci": 2, "üçüncü": 3, "dördüncü": 4, "beşinci": 5, "altıncı": 6, "yedinci": 7,
    "sekizinci": 8, "dokuzuncu": 9, "onuncu": 10}.items()}
# Joint names are not amounts: "2. eklemi", "J2", "ikinci eklem", "joint two".
_JOINT_REF = re.compile(r"\bj\s?\d\b|\b\d\s*\.\s*(?=eklem|eksen|motor)|"
                        r"\b(birinci|ikinci|üçüncü|ucuncu|dördüncü|dorduncu|beşinci|besinci|altıncı|altinci)\s+(?=eklem|eksen)|"
                        r"\bjoint\s+(\d|one|two|three|four|five|six)\b|\bp\s?\d\b")


def request_numbers(text, keep_joint_refs=False):
    """Numbers the operator said, digits or words ("twenty five", "half",
    "yirmi beş", "bir buçuk", "yüz otuz beş", ASCII "bes bucuk"). Joint
    names ("2. eklem", "J3") and point names ("P3") are not amounts."""
    t = tr_lower(text)
    if not keep_joint_refs:
        t = _JOINT_REF.sub(" ", t)
    nums = {abs(float(x.replace(",", "."))) for x in re.findall(r"\d+(?:[.,]\d+)?", t)}
    turkish = is_turkish(text)
    words = re.findall(r"[a-z]+", fold(t))
    i = 0
    while i < len(words):
        w = words[i]
        value = None
        if w == "yuz" or (w in _TR_UNITS_F and i + 1 < len(words) and words[i + 1] == "yuz"):
            value = 100
            if w != "yuz":
                value = 100 * _TR_UNITS_F[w]
                i += 1
            if i + 1 < len(words) and words[i + 1] in _TR_TENS_F:
                value += _TR_TENS_F[words[i + 1]]
                i += 1
            if i + 1 < len(words) and words[i + 1] in _TR_UNITS_F:
                value += _TR_UNITS_F[words[i + 1]]
                i += 1
        elif w in _TR_TENS_F and (turkish or w != "on"):
            value = _TR_TENS_F[w]
            if i + 1 < len(words) and words[i + 1] in _TR_UNITS_F:
                value += _TR_UNITS_F[words[i + 1]]
                i += 1
        elif w in _TR_UNITS_F:
            value = _TR_UNITS_F[w]
        elif w in _TR_ORDINALS_F:
            value = _TR_ORDINALS_F[w]
        elif w == "yarim":
            nums.add(0.5)
        if value is not None:
            if i + 1 < len(words) and words[i + 1] == "bucuk":
                nums.add(value + 0.5)
            nums.add(float(value))
        i += 1
    words = re.findall(r"[a-z]+", t)
    i = 0
    while i < len(words):
        w = words[i]
        if w in _TENS:
            value = _TENS[w]
            if i + 1 < len(words) and words[i + 1] in _UNITS and _UNITS[words[i + 1]] < 10:
                value += _UNITS[words[i + 1]]
                i += 1
            nums.add(float(value))
        elif w in _UNITS:
            nums.add(float(_UNITS[w]))
        elif w == "hundred":
            nums.add(100.0)
        elif w == "half":
            nums.add(0.5)
        i += 1
    return nums


def _command_numbers(cmd):
    """(kind, [numbers]) of one canonical command, for the number guard."""
    name, m = match(cmd)
    if name == "tcp":
        return "mm", [abs(float(m[2]))]
    if name == "jrel":
        return "deg", [abs(float(m[2]))]
    if name == "jabs":
        return "deg", [abs(float(v)) for _, v in _ABS_ITEM.findall(m[1])]
    if name in ("square", "circle"):
        return "shape", [abs(float(m[1]))]
    if name == "rect":
        return "shape", [abs(float(m[1])), abs(float(m[2]))]
    if name == "arc":
        return "arc", [abs(float(m[1])), abs(float(m[2]))]
    if name == "line" and m[3] is not None and int(m[3]) != 0:
        return "code", [float(m[3])]
    if name == "speed":
        return "speed", [abs(float(m[1]))]
    if name == "wait":
        return "wait", [abs(float(m[1]))]
    return None, []


# Words that must appear in the request for a command to be grounded in it.
GROUNDING = {
    "UP": {"up", "lift", "raise", "higher", "ascend", "rise", "upward", "upwards", "top"},
    "DOWN": {"down", "lower", "drop", "descend", "downward", "downwards", "sink"},
    "LEFT": {"left"},
    "RIGHT": {"right"},
    "FORWARD": {"forward", "forwards", "out", "outward", "outwards", "extend", "reach", "away", "push",
                "ahead", "front", "further"},
    "BACK": {"back", "backward", "backwards", "retract", "pull", "closer", "toward", "towards", "in",
             "inward", "withdraw", "off"},
    "J1": {"base", "waist", "j1", "one", "1", "swivel", "pan"},
    "J2": {"shoulder", "j2", "two", "2"},
    "J3": {"elbow", "j3", "three", "3"},
    "J5": {"tilt", "up", "down", "raise", "lower", "angle", "point", "aim", "pitch", "j5", "five", "5",
           "nod", "bow", "upward", "downward", "tip"},
    "J6": {"roll", "twist", "rotate", "spin", "turn", "j6", "six", "6"},
    "GRIPPER CLOSE": {"close", "grab", "grip", "pick", "hold", "clamp", "grasp", "take", "shut"},
    "GRIPPER OPEN": {"open", "release", "drop", "let", "free", "ungrip"},
    "GO_HOME": {"home", "start"},
    "SPEED": {"speed", "percent", "fast", "faster", "slow", "slower", "quick", "quickly", "%", "velocity",
              "pace", "run"},
    "SERVO_OFF": {"off", "disable", "deactivate"},  # an "off" word is required, not just "servo"
    "WAIT": {"wait", "pause", "second", "seconds", "sec", "half", "delay", "hold"},
    "LINE_READY": {"ready", "station"},
    "LINE_BUSY": {"busy", "working", "done", "finished", "free"},
    "LINE_ERROR": {"error", "fault"},
    "LINE_STATUS": {"line", "order", "orders", "station", "conveyor", "transfer", "festo", "belt"},
    "SQUARE": {"square", "box"},
    "RECTANGLE": {"rectangle", "rect"},
    "CIRCLE": {"circle", "round", "ring"},
    "ARC": {"arc", "curve", "half", "quarter", "semicircle"},
    "VERTICAL": {"vertical", "vertically", "upright", "standing"},
}


# Turkish word stems (matched as prefixes, so suffixes like "yukarıya",
# "sola", "bileği" still count). Only used when the request is Turkish.
GROUNDING_TR = {
    "UP": ("yukarı", "kaldır", "yüksel", "yukari", "çık", "kalk"),
    "DOWN": ("aşağı", "indir", "alçal", "düşür", "asagi", "=in", "=inin", "=insin"),
    "LEFT": ("sol",),
    "RIGHT": ("sağ", "sag"),
    "FORWARD": ("ileri", "öne", "ön", "uzat", "uzan", "dışarı"),
    "BACK": ("geri", "çek", "yaklaş", "içeri", "arka", "tabana"),
    "J1": ("taban", "gövde", "=bel", "=beli", "j1", "=bir", "birinci", "1"),
    "J2": ("omuz", "omz", "j2", "=iki", "ikinci", "2"),
    "J3": ("dirsek", "dirseğ", "j3", "=üç", "üçüncü", "3"),
    "J5": ("eğ", "yukarı", "aşağı", "kaldır", "indir", "bük", "j5", "=beş", "beşinci", "5"),
    "J6": ("döndür", "çevir", "dön", "bük", "j6", "=altı", "altıncı", "6", "düzelt", "=düz"),
    "GRIPPER CLOSE": ("kapat", "tut", "kavra", "sık", "yakala", "=al", "alıp", "alır", "taşı"),
    "GRIPPER OPEN": ("aç", "bırak", "serbest", "koy", "taşı", "götür"),
    "GO_HOME": ("ev", "başlangıç", "home", "=başa", "başa dön"),
    "SPEED": ("hız", "yüzde", "yavaş", "%"),
    "SERVO_OFF": ("kapat", "kapa", "kes", "durdur", "off"),
    "WAIT": ("bekle", "saniye"),
    "LINE_READY": ("hazır", "hazir", "istasyon"),
    "LINE_BUSY": ("meşgul", "mesgul", "çalış", "bitti", "bitir", "tamamla", "boş"),
    "LINE_ERROR": ("hata", "arıza", "ariza"),
    "LINE_STATUS": ("sipariş", "siparis", "istasyon", "=iş", "=işi", "=hat", "hattı", "hatta", "hattın", "bant", "konveyör",
                    "konveyor", "transfer", "festo"),
    "SQUARE": ("kare",),
    "RECTANGLE": ("dikdörtgen", "dikdortgen"),
    "CIRCLE": ("daire", "çember", "yuvarlak", "halka"),
    "ARC": ("yay", "kavis", "yarım", "çeyrek", "eğri"),
    "VERTICAL": ("dikey", "düşey", "dik"),
}
VAGUE_WORDS_TR = {"biraz", "azıcık", "hafif", "hafifçe", "çok", "epey", "azcık"}
FULL_SPEED_WORDS_TR = {"tam", "maksimum", "en", "hızlı"}


def _grounding_key(cmd):
    name, m = match(cmd)
    if name == "tcp":
        word = m[1]
        if word in DIRECTIONS:
            return word
        axis_word = {"Z": "UP" if float(m[2]) > 0 else "DOWN", "Y": "LEFT" if float(m[2]) > 0 else "RIGHT",
                     "R": "FORWARD" if float(m[2]) > 0 else "BACK"}.get(word)
        return axis_word
    if name == "jrel":
        return m[1]
    if name == "jabs":
        return [jn for jn, _ in _ABS_ITEM.findall(m[1])]
    if name == "gripper":
        return f"GRIPPER {m[1]}"
    if name == "read" and m[1] == "LINE_STATUS":
        return "LINE_STATUS"
    if name == "line":
        return "LINE_READY" if m[1] == "READY" else "LINE_BUSY" if m[1] == "BUSY" else "LINE_ERROR"
    if name in SHAPES:
        plane = m.lastindex and m[m.lastindex] if name != "rect" else m[3]
        keys = [{"square": "SQUARE", "rect": "RECTANGLE", "circle": "CIRCLE", "arc": "ARC"}[name]]
        if plane == "VERTICAL":
            keys.append("VERTICAL")
        return keys
    if name == "wait" and float(m[1]) in DEFAULT_WAITS:
        return None  # a short settling pause (e.g. after gripping) needs no words
    if name in ("home", "speed", "servo_off", "wait"):
        return {"home": "GO_HOME", "speed": "SPEED", "servo_off": "SERVO_OFF", "wait": "WAIT"}[name]
    return None


CONTINUATION_WORDS = {"daha", "tekrar", "tekrarla", "again", "more", "aynı", "aynısını", "same", "devam",
                      "continue", "undo", "further"}
OPPOSITE = {"UP": "DOWN", "DOWN": "UP", "LEFT": "RIGHT", "RIGHT": "LEFT", "FORWARD": "BACK", "BACK": "FORWARD",
            "GRIPPER OPEN": "GRIPPER CLOSE", "GRIPPER CLOSE": "GRIPPER OPEN"}


def is_continuation(text):
    """'biraz daha', 'tekrar', 'bir daha', 'geri al', 'again', 'a bit more', 'undo' ..."""
    t = tr_lower(text)
    return bool(set(_words(text)) & CONTINUATION_WORDS) or "geri al" in t or "bir daha" in t


def _keys(commands):
    keys = set()
    for c in commands:
        k = _grounding_key(normalize(c))
        keys.update([k] if isinstance(k, str) else (k or []))
    return keys


SHAPE_DEFAULT_MM = {50.0, 100.0}           # "bir kare çiz" / "küçük daire": 5 cm, "büyük": 10 cm
RADIUS_WORDS = {"yarıçap", "yarıçapı", "yarıçaplı", "radius"}
ARC_DEFAULT_DEG = {90.0, 180.0}            # çeyrek / yarım


def _shape_numbers_ok(cmd, kind, numbers, said, cm, mm_said, words):
    """Sizes of a shape must come from the operator's words: centimetres x 10,
    a radius x 2 for a circle's diameter, or the defaults when no size was said."""
    sizes = numbers if kind == "shape" else numbers[:1]
    said_all = set(said)
    if kind == "arc":
        said = {n for n in said if abs(n - numbers[1]) > 1e-6}  # "60 derecelik yay": 60 is the angle
    base = set(said) if not cm or mm_said else set()
    base |= {n * 10 for n in said} if cm else set()
    ok_sizes = set(base)
    if words & RADIUS_WORDS or any(w.startswith("yarıçap") for w in words):
        ok_sizes |= {n * 2 for n in base}
    # "bir kare" / "a half circle": "bir" (a/one) and "half" are not sizes.
    real_sizes = {n for n in said if n > 1}
    if not real_sizes or words & (VAGUE_WORDS | VAGUE_WORDS_TR | {"küçük", "büyük", "small", "big", "large"}):
        ok_sizes |= SHAPE_DEFAULT_MM
    for n in sizes:
        if not any(abs(n - x) < 1e-6 for x in ok_sizes):
            if cm and any(abs(n - x) < 1e-6 for x in said):
                return L(f"“{cmd}” uses {n:g} mm, but you said {n:g} cm = {n * 10:g} mm",
                         f"“{cmd}” {n:g} mm diyor ama siz {n:g} santim dediniz, yani {n * 10:g} mm")
            return L(f"“{cmd}” uses {n:g}, but you did not say that size", f"“{cmd}” içinde {n:g} var ama siz bu boyutu söylemediniz")
    if kind == "arc":
        deg = numbers[1]
        said = said_all
        if not (any(abs(deg - x) < 1e-6 for x in said) or (deg in ARC_DEFAULT_DEG and (
                deg == 180 and words & {"yarım", "half", "semicircle"} or deg == 90 and words & {"çeyrek", "quarter"}
                or len({n for n in said if n > 1}) <= 1))):
            return L(f"“{cmd}” turns {deg:g} degrees, but you did not say that", f"“{cmd}” {deg:g} derece diyor ama siz bunu söylemediniz")
    return None


RESIZE_UP = ("büyüt", "büyük", "genişlet", "uzat", "bigger", "larger", "enlarge", "grow", "wider")
RESIZE_DOWN = ("küçült", "küçük", "daralt", "kısalt", "smaller", "shrink", "narrower")
REFERENCE_WORDS = {"onu", "bunu", "şunu", "ona", "buna", "aynısı", "aynısını", "it", "that", "this"}


RESIZE_ADJECTIVES = ("büyük", "küçük", "uzat", "kısalt")  # resize only with "daha" ("daha büyük yap")
DRAW_WORDS = ("çiz", "draw", "yeni", "new")


def resize_direction(text):
    """+1 for "büyüt / daha büyük / bigger", -1 for "küçült / daha küçük /
    smaller", 0 otherwise ("küçük bir kare çiz" is a new shape, not a resize)."""
    words = _words(text)
    if any(w.startswith(DRAW_WORDS) for w in words):
        return 0
    more = bool(set(words) & {"daha", "more"})

    def hit(stems):
        return any(w.startswith(stems) and (more or not w.startswith(RESIZE_ADJECTIVES)) for w in words)
    up, down = hit(RESIZE_UP), hit(RESIZE_DOWN)
    return 0 if up == down else (1 if up else -1)


def refers_back(text):
    """The request builds on the previous exchange ("biraz daha", "tekrar",
    "büyüt", "onu sola götür" ...)."""
    return is_continuation(text) or bool(resize_direction(text)) or bool(set(_words(text)) & REFERENCE_WORDS)


def intent_check(request, commands, previous=None):
    """previous: {"request", "commands"} of the last plan the operator saw.
    When the request refers back to it ("biraz daha", "tekrar", "geri al",
    "büyüt"), its words, directions (and their opposites, for undo) and
    numbers count as said; "büyüt / küçült" allows a bigger / smaller shape."""
    if previous and previous.get("commands") and refers_back(request):
        extra_keys = _keys(previous["commands"])
        extra_keys |= {OPPOSITE[k] for k in extra_keys if k in OPPOSITE}
        numbers, shape_sizes = set(), []
        for c in previous["commands"]:
            kind, n = _command_numbers(normalize(c))
            numbers.update(n)
            if kind in ("shape", "arc"):
                shape_sizes.extend(n if kind == "shape" else n[:1])
        resize = (resize_direction(request), shape_sizes) if shape_sizes and resize_direction(request) else None
        vague = "" if request_numbers(request) else " biraz"  # "daha" alone: a small step is fine
        return _intent_check(request + vague + " " + previous["request"] + " " + " ".join(f"{n:g}" for n in numbers),
                             commands, extra_keys, resize)
    return _intent_check(request, commands, set())


def plan_unrelated(request, commands, previous=None):
    """True when no command of the plan fits the request at all (e.g. SPEED 5
    for "ascend 35 mm"): worth asking the model again before asking the
    operator. A plan that fits partly ("5 santim" right, direction unclear)
    is asked about directly."""
    return bool(commands) and all(intent_check(request, [c], previous) for c in commands)


def _lev(a, b):
    """Edit distance (short words only)."""
    prev = list(range(len(b) + 1))
    for i, ca in enumerate(a, 1):
        cur = [i]
        for j, cb in enumerate(b, 1):
            cur.append(min(prev[j] + 1, cur[j - 1] + 1, prev[j - 1] + (ca != cb)))
        prev = cur
    return prev[-1]


def _stem_hit(word, stem):
    """Turkish stem match: "=x" exact, otherwise a prefix. Stems of 4+ letters
    also match with one wrong, missing or extra letter, because speech
    recognition writes "illeri", "yukarıa", "aşşağı"."""
    word, stem = fold(word), fold(stem)
    if stem.startswith("="):
        return word == stem[1:]
    if word.startswith(stem):
        return True
    if len(stem) < 4 or _known_word(word):
        return False  # a real word ("ileri") is never read as a typo of another ("içeri")
    return any(_lev(word[:k], stem) <= 1 for k in (len(stem) - 1, len(stem), len(stem) + 1) if k <= len(word))


def _known_word(word):
    return any(fold(word).startswith(fold(st)) for stems in GROUNDING_TR.values() for st in stems if not st.startswith("=")) \
        or any(word == w for ws in GROUNDING.values() for w in ws)


def _word_hit(words, needed):
    """English grounding: exact word, or one letter off for words of 5+ letters."""
    return bool(words & needed) or any(len(n) >= 5 and len(w) >= 4 and not _known_word(w) and _lev(w, n) <= 1
                                       for w in words for n in needed)


def _intent_check(request, commands, extra_keys, resize=None):
    """Return a reason string if the plan uses numbers or names the operator
    did not say, else None. Runs only on model output, never on typed '>'
    commands."""
    text = tr_lower(request)
    turkish = is_turkish(request)
    words = set(_words(request))
    said = request_numbers(request)
    cm = bool(re.search(r"\bcm\b|centimet|santim", text))
    mm_said = bool(re.search(r"\bmm\b|milim|millim", text))
    allowed = set(said) | ({n * 10 for n in said} if cm else set())
    # "100 santim" is 1000 mm, never 100 mm: with only centimetres spoken, a
    # millimetre value must be the spoken number x 10.
    allowed_mm = {n * 10 for n in said} if cm and not mm_said else allowed
    vague = bool(words & (VAGUE_WORDS | VAGUE_WORDS_TR)) or not said
    if turkish and words & FULL_SPEED_WORDS_TR:
        words = words | FULL_SPEED_WORDS
    flat = []
    for c in commands:
        c = normalize(c)
        if c.startswith("DEFINE_SKILL"):
            flat.extend(normalize(x) for x in split_skill_body(c.partition(":")[2]))
        flat.append(c)
    for i, c in enumerate(flat):
        after_gripper = i > 0 and flat[i - 1].startswith("GRIPPER")
        if after_gripper and match(c)[0] == "wait" and float(match(c)[1][1]) <= 2.0:
            continue  # a short pause after gripping lets the pneumatics finish; always harmless
        kind, numbers = _command_numbers(c)
        if kind in ("shape", "arc"):
            others = set()
            for o in flat:
                if o is not c:
                    k2, n2 = _command_numbers(o)
                    if k2 in ("mm", "deg", "wait", "speed"):
                        others.update(n2)
                        others.update(x / 10 for x in n2)
            reason = _shape_numbers_ok(c, kind, numbers, said - others, cm, mm_said, words)
            if reason and resize:
                sign, before = resize
                sizes = numbers if kind == "shape" else numbers[:1]
                if all((n > max(before)) if sign > 0 else (n < min(before)) for n in sizes):
                    reason = None  # "biraz büyüt": any bigger size is what was asked
            if reason:
                return reason
            if kind == "arc":
                right = bool(words & {"right", "clockwise"}) or any(w.startswith(("sağ", "sag")) for w in words)
                left = bool(words & {"left", "counterclockwise", "anticlockwise"}) or any(w.startswith("sol") for w in words)
                deg = float(match(c)[1][2])
                # No side said: either way is fine. A side that was said must be kept.
                if (deg < 0 and left and not right) or (deg > 0 and right and not left):
                    return L(f"“{c}” curves {'right' if deg < 0 else 'left'}, but you did not say that",
                             f"“{c}” {'sağa' if deg < 0 else 'sola'} kıvrılıyor ama siz bunu söylemediniz")
            vertical_said = bool(words & GROUNDING["VERTICAL"]) or (
                turkish and any(w.startswith(("dikey", "düşey")) for w in words))
            if vertical_said and not c.rstrip().endswith("VERTICAL"):
                return L(f"you said vertical, but “{c}” is horizontal",
                         f"dikey dediniz ama “{c}” yatay bir şekil")
            numbers = []
        for n in numbers:
            ok = any(abs(n - a) < 1e-6 for a in (allowed_mm if kind == "mm" else allowed))
            if not ok and kind in ("mm", "deg") and vague:
                ok = n in VAGUE_AMOUNTS
            if not ok and kind == "mm" and {fold(w) for w in words} & {fold(w) for w in MAX_WORDS}:
                ok = n == MAX_TCP_MM
            if not ok and kind == "speed" and (words & FULL_SPEED_WORDS or any(x > MAX_OVERRIDE for x in said)
                                               or any(fold(w).startswith(("maksimum", "hizlan")) for w in words)):
                ok = n == MAX_OVERRIDE  # "yüzde 60" / "tam hız" -> the 30 % cap
            if not ok and kind == "speed" and any(fold(w).startswith(("yavas", "dusur", "azalt")) for w in words) \
                    or not ok and kind == "speed" and words & {"slow", "slower"}:
                ok = n in (5.0, 10.0)
            if not ok and kind == "wait" and any(x > MAX_WAIT_S for x in said):
                ok = n == MAX_WAIT_S
            if not ok and kind == "wait":
                ok = n in DEFAULT_WAITS
            if not ok and kind == "deg" and n == 0 and ({"zero", "straight", "straighten", "reset"} & words or any(
                    fold(w).startswith(("sifir", "duzelt", "duz")) for w in words)):
                ok = True
            if not ok and kind == "mm" and cm and not mm_said and any(abs(n - a) < 1e-6 for a in said):
                return L(f"“{c}” uses {n:g} mm, but you said {n:g} cm = {n * 10:g} mm",
                         f"“{c}” {n:g} mm diyor ama siz {n:g} santim dediniz, yani {n * 10:g} mm")
            if not ok:
                return L(f"“{c}” uses {n:g}, but you did not say {n:g}",
                         f"“{c}” içinde {n:g} var ama siz {n:g} demediniz")
        keys = _grounding_key(c)
        for key in ([keys] if isinstance(keys, str) else keys or []):
            if key in extra_keys:
                continue  # grounded by the previous turn the operator is referring to
            words_needed = GROUNDING.get(key)
            grounded = bool(words_needed) and (_word_hit(words, words_needed) or bool(set(re.findall(r"\d+|%", text)) & words_needed))
            if not grounded:  # Turkish words always count: short sentences are not always detected as Turkish
                stems = GROUNDING_TR.get(key, ())
                grounded = any(_stem_hit(w, stem) for w in words for stem in stems) or \
                    bool(set(re.findall(r"\d+|%", text)) & set(stems))
            if words_needed and not grounded:
                what = L({"J1": "the base", "J2": "the shoulder", "J3": "the elbow", "J5": "a wrist tilt",
                          "J6": "a wrist roll"}.get(key, key.lower().replace("_", " ")),
                         {"J1": "taban", "J2": "omuz", "J3": "dirsek", "J5": "bilek eğme", "J6": "bilek çevirme",
                          "UP": "yukarı", "DOWN": "aşağı", "LEFT": "sol", "RIGHT": "sağ", "FORWARD": "ileri",
                          "BACK": "geri", "GRIPPER CLOSE": "tutucuyu kapatma", "GRIPPER OPEN": "tutucuyu açma",
                          "GO_HOME": "eve dönme", "SPEED": "hız", "SERVO_OFF": "servo kapatma",
                          "WAIT": "bekleme", "SQUARE": "kare", "RECTANGLE": "dikdörtgen", "CIRCLE": "daire",
                          "ARC": "yay", "VERTICAL": "dikey", "LINE_READY": "hazır sinyali",
                          "LINE_BUSY": "meşgul sinyali", "LINE_ERROR": "hata kodu",
                          "LINE_STATUS": "transfer hattı"}.get(key, key))
                return L(f"“{c}” means {what}, but your words do not say that",
                         f"“{c}” {what} anlamına geliyor ama sözlerinizde bu yok")
        name, m = match(c)
        if name in ("run_skill", "forget", "save_point", "define"):
            label = m[1].lower()
            if label not in text:
                return L(f"“{c}” names {m[1]!r}, which you did not mention", f"“{c}” {m[1]!r} adını kullanıyor ama siz bunu söylemediniz")
        if name == "goto":
            label = m[1].lower()
            digits = re.sub(r"\D", "", label)
            if label not in text and label not in re.sub(r"\s+", "", text) and not (digits and float(digits) in said):
                return L(f"“{c}” names point {m[1]!r}, which you did not mention", f"“{c}” {m[1]!r} noktasını kullanıyor ama siz bunu söylemediniz")
    return None


def _said_directions(request):
    """TCP directions the request names (UP, LEFT ...)."""
    words = set(_words(request))
    turkish = is_turkish(request)
    out = set()
    for key in DIRECTIONS:
        if any(_stem_hit(w, st) for w in words for st in GROUNDING_TR.get(key, ())) if turkish \
                else _word_hit(words, GROUNDING.get(key, set())):
            out.add(key)
    return out


def repair_directions(request, commands):
    """The model wrote the opposite of the one direction the operator said
    ("sola git" -> MOVE_TCP RIGHT): use the operator's. Returns (commands, changes)."""
    said = _said_directions(request)
    if len(said) != 1:
        return list(commands), []
    want = next(iter(said))
    out, changes = list(commands), []
    for i, c in enumerate(commands):
        c = normalize(c)
        name, m = match(c)
        if name == "tcp" and m[1] == OPPOSITE.get(want):
            out[i] = f"MOVE_TCP {want} {m[2]}"
            changes.append((c, out[i]))
    return out, changes


def repair_numbers(request, commands):
    """When the operator said one amount with a unit and the model used a
    different one for its single move or shape, use the operator's amount
    ("5 cm sola" -> MOVE_TCP LEFT 50, not 10). Returns (commands, changes)."""
    text = fold(request)
    words = _words(request)
    said = request_numbers(request)
    if "bir" in words and not re.search(r"\b1\b|\bbir\s+(santim|cm|milim|mm|derece|bucuk)", text):
        said.discard(1.0)  # "bir kare" = "a square"
    if "bucuk" in text:
        said = {n for n in said if n != int(n)} or said  # "bir buçuk" = 1.5 only
    cm = bool(re.search(r"\bcm\b|centimet|santim", text))
    mm = bool(re.search(r"\bmm\b|milim|millim", text))
    deg = bool(re.search(r"derece|degree|°", text))
    if len(said) != 1 or (cm and mm) or set(words) & RADIUS_WORDS or "yaricap" in text or refers_back(request):
        return list(commands), []
    n = next(iter(said))
    canon = [normalize(c) for c in commands]
    targets = [i for i, c in enumerate(canon) if match(c)[0] in ("tcp", "jrel", "square", "circle")]
    numbered = [c for c in canon if _command_numbers(c)[1]]
    if len(targets) != 1 or len(numbered) != 1:
        return list(commands), []  # another number in the plan (a wait, a speed): not sure which the amount belongs to
    i = targets[0]
    c = canon[i]
    name, m = match(c)
    if name in ("tcp", "square", "circle"):
        if not (cm or mm):
            return list(commands), []
        want = n * 10 if cm else n
    else:
        if cm or mm or not deg:
            return list(commands), []  # only an amount said in degrees fixes a joint move
        want = n
    old = abs(float(m[2] if name in ("tcp", "jrel") else m[1]))
    if abs(old - want) < 1e-6:
        return list(commands), []
    if name == "tcp":
        new = (f"MOVE_TCP {m[1]} {want:g}" if m[1] in DIRECTIONS
               else f"MOVE_TCP {m[1]} {math.copysign(want, float(m[2])):+g}")
    elif name == "jrel":
        new = f"MOVE_JOINT_REL {m[1]} {math.copysign(want, float(m[2])):+g}"
    else:
        new = re.sub(r"-?\d+(?:\.\d+)?", f"{want:g}", c, count=1)
    if match(new)[0] != name:
        return list(commands), []
    out = list(commands)
    out[i] = new
    return out, [(c, new)]


# -- follow-ups resolved by rule, not by the model --------------------------------------
# "biraz daha", "tekrar", "geri al" have one exact meaning given the previous
# move, so they are worked out here deterministically.

_REPEAT = {"tekrar", "tekrarla", "again", "aynısını", "same", "yine"}
_UNDO_WORDS = {"undo"}
_MORE = {"daha", "more", "devam", "continue", "further"}
_FILLER = {"biraz", "bir", "az", "azıcık", "yap", "et", "lütfen", "hadi", "aynı", "şeyi", "şekilde", "kez", "kere",
           "a", "bit", "little", "do", "it", "please", "that", "once", "one", "the", "same", "thing", "santim", "cm",
           "santimetre", "milimetre", "milim", "mm", "derece", "degree", "degrees", "tane", "sonra", "şimdi",
           "tamam", "ok", "okay", "and", "ve"}


_OTHER_SHAPE_WORDS = {"square": ("daire", "çember", "yuvarlak", "dikdörtgen", "yay", "circle", "rectangle", "arc"),
                      "circle": ("kare", "dikdörtgen", "yay", "square", "rectangle", "arc"),
                      "rect": ("kare", "daire", "çember", "yuvarlak", "yay", "square", "circle", "arc"),
                      "arc": ("kare", "daire", "çember", "dikdörtgen", "square", "circle", "rectangle")}


def _resize_followup(request, previous):
    """"biraz büyüt" / "2 santim küçült" / "make it bigger" right after a
    shape: the same shape, bigger or smaller. Said amount in cm/mm, else
    1 cm for "biraz", else 2 cm."""
    sign = resize_direction(request)
    if not sign:
        return None
    shapes = [normalize(c) for c in previous["commands"] if match(normalize(c))[0] in SHAPES]
    if len(shapes) != 1:
        return None
    c = shapes[0]
    name, m = match(c)
    words = _words(request)
    if any(w.startswith(_OTHER_SHAPE_WORDS[name]) for w in words):
        return None  # a different shape was named: the model reads it
    t = tr_lower(request)
    said = request_numbers(request)
    if "bir" in words and not re.search(r"\b1\b|\bbir\s+(santim|cm|milim|mm|derece)", t):
        said.discard(1.0)  # "bir" = "a", not 1 (but "bir santim" is 1 cm)
    if len(said) > 1:
        return None
    if said:
        n = next(iter(said))
        delta = n * 10 if re.search(r"\bcm\b|santim|centimet", t) else n
    else:
        delta = 10.0 if set(words) & (VAGUE_WORDS | VAGUE_WORDS_TR) else 20.0

    def size(v):
        return max(MIN_SHAPE_MM, min(MAX_SHAPE_MM, float(v) + sign * delta))
    if name in ("square", "circle"):
        new = re.sub(r"-?\d+(?:\.\d+)?", f"{size(m[1]):g}", c, count=1)
    elif name == "rect":
        a, b = size(m[1]), size(m[2])
        new = f"DRAW_RECTANGLE {a:g} {b:g}" + (f" {m[3]}" if m[3] else "")
    else:
        new = re.sub(r"-?\d+(?:\.\d+)?", f"{size(m[1]):g}", c, count=1)
    return None if new == c else [new]


def resolve_followup(request, previous):
    """Return the commands a pure follow-up means, or None if the request says
    something new (then the model handles it). previous: {"request", "commands"}."""
    if not previous:
        return None
    resized = _resize_followup(request, previous)
    if resized:
        return resized
    t = tr_lower(request)
    words = _words(request)
    undo = "geri al" in t or bool(set(words) & _UNDO_WORDS)
    repeat = "bir daha" in t or bool(set(words) & _REPEAT)
    more = bool(set(words) & _MORE) and not repeat
    if not (undo or repeat or more):
        return None
    known = _REPEAT | _UNDO_WORDS | _MORE | _FILLER | {"geri", "al"}
    rest = [w for w in words if w not in known and w not in _TR_UNITS and w not in _TR_TENS
            and w not in _UNITS and w not in _TENS and w != "yarım" and w != "buçuk"]
    if rest:
        return None  # a new direction or object was mentioned: let the model read it
    prev = [normalize(c) for c in previous["commands"]]
    motion = [c for c in prev if match(c)[0] in ("tcp", "jrel", "gripper")]

    if repeat:
        return prev
    if undo:
        inverse = []
        for c in reversed(motion):
            name, m = match(c)
            if name == "tcp":
                word = m[1]
                inverse.append(f"MOVE_TCP {OPPOSITE.get(word, word)} {m[2]}" if word in DIRECTIONS
                               else f"MOVE_TCP {word} {-float(m[2]):+g}")
            elif name == "jrel":
                inverse.append(f"MOVE_JOINT_REL {m[1]} {-float(m[2]):+g}")
            elif name == "gripper":
                inverse.append("GRIPPER CLOSE" if m[1] == "OPEN" else "GRIPPER OPEN")
        return inverse or None
    # more: same direction as the last motion command, a small or the stated amount
    moves = [c for c in motion if match(c)[0] in ("tcp", "jrel")]
    if not moves:
        return None
    name, m = match(moves[-1])
    said = request_numbers(request)
    said.discard(1.0) if "bir" in words and len(said) > 1 else None
    amount = max(said) if said else None
    if name == "tcp":
        cm = bool(re.search(r"\bcm\b|santim|centimet", t))
        mm = (amount * 10 if cm else amount) if amount else 10.0
        word = m[1]
        if word in DIRECTIONS:
            return [f"MOVE_TCP {word} {mm:g}"]
        return [f"MOVE_TCP {word} {math.copysign(mm, float(m[2])):+g}"]
    deg = amount or 5.0
    return [f"MOVE_JOINT_REL {m[1]} {math.copysign(deg, float(m[2])):+g}"]


# -- honesty about robot state ---------------------------------------------------------
# The model is never trusted to state the robot's position or status: questions
# about them are answered by a real reading, and model answers that contain
# made-up angles or coordinates are replaced by one.

_POSITION_Q = re.compile(r"\b(nerede|neredesin|neresi|konum\w*|pozisyon\w*|koordinat\w*|açı\w*|acı\w*|kaç derece|"
                         r"where|position|coordinates?|angles?|pose)\b", re.I)
_STATUS_Q = re.compile(r"\b(servo\w*\s+(açık|kapalı|acik|kapali)\s*mı|alarm\w*|durum\w*|status|"
                       r"tutucu\w*\s+(açık|kapalı)\s*mı|bağlı\s*mı|is the servo|any alarm)\b", re.I)
_STATE_CLAIM = re.compile(r"(J[1-6]\s*[=:]?\s*-?\d|\b[XYZ]\s*[=:]?\s*-?\d|\d+\s*°\s*,|-?\d+(?:\.\d+)?\s*,\s*-?\d+(?:\.\d+)?\s*,\s*-?\d+)")


def state_question(text):
    """WHERE / STATUS if the operator asks about the robot's state, else None."""
    t = tr_lower(text)
    if not re.search(r"\?|mısın|misin|musun|müsün|söyle|nedir|ne\b|mi\b|mı\b|nerede|neresi|rapor\w*|report|what|where|tell|give", t):
        return None
    if re.search(r"\b(sipariş\w*|siparis\w*|order\w*|istasyon\w*|hat\b|hattı\w*)", t):
        return "LINE_STATUS"
    if _STATUS_Q.search(t):
        return "STATUS"
    if _POSITION_Q.search(t):
        return "WHERE"
    return None


def claims_state(say):
    """True if the model's reply states joint angles or coordinates by itself."""
    return bool(_STATE_CLAIM.search(say or ""))


def drop_ungrounded_waits(request, commands):
    """A pause the operator did not ask for ("mola verelim") is harmless: remove
    it rather than reject the whole plan. Nothing else is ever removed."""
    words = set(_words(request))
    asked = bool(words & GROUNDING["WAIT"]) or any(w.startswith(("bekle", "saniye")) for w in words)
    if asked:
        return commands, []
    kept = [c for c in commands if match(normalize(c))[0] != "wait" or
            (commands.index(c) > 0 and normalize(commands[commands.index(c) - 1]).startswith("GRIPPER"))]
    return kept, [c for c in commands if c not in kept]



# -- speech-recognition clean-up ---------------------------------------------------------
# Whisper often writes Turkish commands in the past tense ("sola gitti") or splits
# words ("bir razı daha"). Fix the typical cases before anything else reads the
# text. Past-tense fixes are skipped for questions ("indi mi?").

WHISPER_PROMPT_TR = ("Robot kol komutları: yukarı, aşağı, sola, sağa, ileri, geri, santim, milimetre, derece, "
                     "biraz, biraz daha, tekrar, geri al, tabanı döndür, bileği eğ, tutucuyu aç, tutucuyu kapat, "
                     "parçayı tut, kare çiz, daire çiz, yarım daire, eve dön, dur, onayla, iptal, hocam.")

_FIXES = [
    (r"\bbir\s*razı\b", "biraz"), (r"\bbi\s*raz\b", "biraz"), (r"\bbiraz\s*ı\b", "biraz"),
    (r"\bda+\b", "daha"), (r"\bdaa+\b", "daha"),
    (r"\by[ıi]kar[ıi]\b", "yukarı"), (r"\byukari\b", "yukarı"), (r"\basa[gğ][ıi]\b", "aşağı"),
    (r"\bsol'?a\b", "sola"), (r"\bsa[gğ]'?a\b", "sağa"), (r"\bileri'?ye\b", "ileri"),
    (r"\bsan\s*tim\b", "santim"), (r"\bsantm\b", "santim"),
    (r"\bill+eri\b", "ileri"), (r"\bil+eri'?ye\b", "ileri"), (r"\ba[sş]+a[gğ]+[ıi]\b", "aşağı"),
]
_PAST_TO_COMMAND = {"gitti": "git", "çıktı": "çık", "indi": "in", "kaydı": "kay", "döndürdü": "döndür",
                    "kaldırdı": "kaldır", "indirdi": "indir", "eğdi": "eğ", "açtı": "aç", "kapattı": "kapat",
                    "çizdi": "çiz", "döndü": "dön", "geldi": "gel", "tuttu": "tut", "bıraktı": "bırak"}


def fix_transcript(text):
    """Return the cleaned-up text (unchanged if nothing matched)."""
    t = text.strip()
    for pat, rep in _FIXES:
        t = re.sub(pat, rep, t, flags=re.I)
    question = bool(re.search(r"\?|\bm[iıuü]\b|\bm[iıuü]s[iıuü]n\b", tr_lower(t)))
    if not question:
        t = re.sub(r"\b(" + "|".join(_PAST_TO_COMMAND) + r")\b",
                   lambda m: _PAST_TO_COMMAND[m.group(1).lower()], t, flags=re.I)
    return re.sub(r"\s+", " ", t).strip(" .")


_CONFIRM_WORDS = {"onayla", "onaylıyorum", "onaylıyor", "onaylı", "evet", "tamam", "yap", "olur", "confirm", "yes"}
_CANCEL_WORDS = {"iptal", "vazgeç", "vazgeçtim", "hayır", "yapma", "cancel", "no"}


def is_confirm_word(text):
    words = _words(text)
    return 0 < len(words) <= 3 and all(w in _CONFIRM_WORDS | {"hocam", "et", "lütfen", "ediyorum"} for w in words) \
        and bool(set(words) & _CONFIRM_WORDS)


def is_cancel_word(text):
    words = _words(text)
    return 0 < len(words) <= 3 and bool(set(words) & _CANCEL_WORDS)



def is_hard_rule(commands):
    """Plans that break a fixed rule (servo on) are never re-planned around."""
    return any(match(normalize(c))[0] == "servo_on" for c in commands if isinstance(c, str))
