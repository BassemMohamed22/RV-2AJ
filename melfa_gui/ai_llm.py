"""
Local language model (Ollama) that turns natural language into a command
list for the validator. The model is free to compose any sequence; the
validator decides what may run.

Sized for an RTX 3050 4 GB: a 3B model (~2 GB at Q4) stays fully in VRAM.
A 7B model would spill into the 8 GB of system RAM and crawl.
"""

import json
import os

import requests

import ai_validator as v

OLLAMA_URL = os.environ.get("OLLAMA_HOST", "http://127.0.0.1:11434").rstrip("/")
if not OLLAMA_URL.startswith("http"):
    OLLAMA_URL = "http://" + OLLAMA_URL
DEFAULT_MODEL = os.environ.get("MELFA_AI_MODEL", "rv2aj-qwen")  # qwen2.5:3b was removed (2026-10-01)
REQUEST_TIMEOUT_S = 120  # the first request also loads the model into VRAM

# Ollama structured output: the reply must be this JSON object.
RESPONSE_SCHEMA = {
    "type": "object",
    "properties": {
        "say": {"type": "string"},
        "commands": {"type": "array", "items": {"type": "string"}},
    },
    "required": ["say", "commands"],
}


class LLMError(Exception):
    pass


def system_prompt():
    limits = ", ".join(f"{j} {lo:g}..{hi:g}" for j, (lo, hi) in v.JOINT_LIMITS_DEG.items())
    motion_state = "ENABLED" if v.MOVE_SYNTAX_VERIFIED else \
        "currently DISABLED (still write motion commands when asked; the safety layer will explain)"
    return f"""You control a Mitsubishi RV-2AJ robot arm (5 axes: J1 J2 J3 J5 J6, there is NO J4) with a CR1-571 controller.
You turn the operator's request into controller commands. Every command you write is checked by a safety layer before anything runs, and the operator confirms anything that changes the robot.

Answer ONLY with JSON: {{"say": "<one short sentence for the operator>", "commands": ["<command>", ...]}}
Use an empty commands list when you are only answering, cannot help, or the request is unsafe; explain why in "say".

COMMANDS (write them exactly like this, one per list item, no "1;1;" prefix):
  STATE                 read controller state / alarm
  JPOSF                 read joint angles
  PPOSF                 read tool position X Y Z
  VALM_INB(0)           read inputs 0-7 (transfer line order bits)
  VALM_OUTB(0)          read outputs 0-7
  CNTLON                take operation rights; put it FIRST in any plan that uses an EXEC command or motion
  CNTLOFF               release operation rights
  SRVOFF                servo off
  STOP                  stop
  EXECJOVRD <n>         speed override in percent, 1..{v.MAX_OVERRIDE_PERCENT} only
  EXECM_OUT(<n>)=1 / EXECM_OUT(<n>)=0   set another output n on or off
  EXECHOPEN {v.HAND_NUMBER} / EXECHCLOSE {v.HAND_NUMBER}   open / close the pneumatic gripper (hand {v.HAND_NUMBER}, Festo HGP-16, grips with up to 160 N)
  RUN <program number>  run a stored robot program
  MOVE_JOINT_REL J<n> <+/-degrees>          move one joint relative to where it is
  MOVE_JOINT_ABS J1=<deg> J2=<deg> ...      move listed joints to absolute angles
Motion is {motion_state}.

NEVER: turn the servo on (no SRVON: the operator does that with a button), use outputs OUT0-OUT3 (not connected), or exceed override {v.MAX_OVERRIDE_PERCENT}.
Joint limits in degrees: {limits}.

TRANSFER LINE (Festo MPS 500, transfer system station 4). For these signals ALWAYS use:
  LINE_READY ON / LINE_READY OFF        station ready (OUT7)
  LINE_BUSY ON / LINE_BUSY OFF          order active / busy (OUT6)
  LINE_ERROR_CODE <0-3>                 error code on OUT5/OUT4 (0 = no error)
  "not ready", "no longer ready" = LINE_READY OFF.  "done", "finished", "not busy", "clear busy" = LINE_BUSY OFF.
  Only change the signals the operator asked for.
  IN6 = order bit 0, IN7 = order bit 1: 00 no order, 01 assemble, 10 sorting out, 11 take part on pallet
  The conveyor belt is NOT controlled by the robot.

The current robot status is given with each request, for answering questions only: always do what the operator asks, even if the status already shows that value. Positive joint angles tilt forward.

EXAMPLES
"where is the arm" -> {{"say": "Reading the joint angles.", "commands": ["JPOSF"]}}
"is there an order" -> {{"say": "Reading the order bits.", "commands": ["VALM_INB(0)"]}}
"tell the line we are ready" -> {{"say": "Setting station ready.", "commands": ["CNTLON", "LINE_READY ON"]}}
"we are working on the order" -> {{"say": "Setting busy.", "commands": ["CNTLON", "LINE_BUSY ON"]}}
"order finished, clear busy and set ready" -> {{"say": "Clearing busy and setting ready.", "commands": ["CNTLON", "LINE_BUSY OFF", "LINE_READY ON"]}}
"the station is not ready" -> {{"say": "Clearing station ready.", "commands": ["CNTLON", "LINE_READY OFF"]}}
"report error code 3" -> {{"say": "Reporting error code 3.", "commands": ["CNTLON", "LINE_ERROR_CODE 3"]}}
"send error 1 to the station" -> {{"say": "Reporting error code 1.", "commands": ["CNTLON", "LINE_ERROR_CODE 1"]}}
"we are no longer busy" -> {{"say": "Clearing busy.", "commands": ["CNTLON", "LINE_BUSY OFF"]}}
"clear the error" -> {{"say": "Clearing the error code.", "commands": ["CNTLON", "LINE_ERROR_CODE 0"]}}
"close the gripper" -> {{"say": "Closing the gripper.", "commands": ["CNTLON", "EXECHCLOSE {v.HAND_NUMBER}"]}}
"raise joint 2 by 10 degrees slowly" -> {{"say": "Moving J2 by +10 degrees at 10 percent.", "commands": ["CNTLON", "EXECJOVRD 10", "MOVE_JOINT_REL J2 +10"]}}
"turn the servo on" -> {{"say": "I cannot turn the servo on. Please use the Servo ON button.", "commands": []}}
"""


def status_text(context):
    """One-paragraph robot status for the model, from the GUI's poll data."""
    if not context.get("connected"):
        return "Robot status: not connected."
    joints = context.get("joints") or {}
    j = " ".join(f"{k}={joints[k]:.2f}" for k in ("J1", "J2", "J3", "J5", "J6") if joints.get(k) is not None)
    parts = [
        "connected",
        f"servo {'ON' if context.get('servo_on') else 'OFF'}",
        "ALARM pending" if context.get("alarm") else "no alarm",
        f"joints {j or 'unknown'}",
        f"override slider {context.get('override')}%",
    ]
    if context.get("line"):
        parts.append(context["line"])
    return "Robot status: " + ", ".join(parts) + "."


class OllamaClient:
    def __init__(self, model=DEFAULT_MODEL, url=OLLAMA_URL):
        self.model = model
        self.url = url

    def check(self):
        """(ok, message) — whether the server is up and the model is pulled."""
        try:
            r = requests.get(f"{self.url}/api/tags", timeout=3)
            r.raise_for_status()
        except requests.RequestException:
            return False, f"Ollama not reachable at {self.url} (run ./setup_ai.sh)"
        names = {m.get("name", "") for m in r.json().get("models", [])}
        if self.model not in names and f"{self.model}:latest" not in names:
            return False, f"model {self.model} not pulled (ollama pull {self.model})"
        return True, f"{self.model} ready"

    def warm_up(self):
        """Load the model into VRAM now so the first request is fast."""
        try:
            requests.post(f"{self.url}/api/generate", json={"model": self.model, "keep_alive": "30m"},
                          timeout=REQUEST_TIMEOUT_S)
        except requests.RequestException:
            pass

    def plan(self, request_text, context):
        """Return (raw_text, parsed_dict). Raises LLMError; never retries."""
        body = {
            "model": self.model,
            "messages": [
                {"role": "system", "content": system_prompt()},
                {"role": "user", "content": f"{status_text(context)}\nRequest: {request_text}"},
            ],
            "stream": False,
            "format": RESPONSE_SCHEMA,
            "keep_alive": "30m",
            "options": {"temperature": 0, "num_ctx": 4096, "num_predict": 400},
        }
        try:
            r = requests.post(f"{self.url}/api/chat", json=body, timeout=REQUEST_TIMEOUT_S)
        except requests.RequestException as e:
            raise LLMError(f"Ollama not reachable at {self.url}: {e}") from e
        if r.status_code != 200:
            raise LLMError(f"Ollama error {r.status_code}: {r.text.strip()[:200]}")
        raw = (r.json().get("message") or {}).get("content", "")
        try:
            parsed = json.loads(raw)
        except json.JSONDecodeError as e:
            raise LLMError(f"model output was not valid JSON: {raw[:200]!r}") from e
        if not isinstance(parsed, dict) or not isinstance(parsed.get("commands"), list):
            raise LLMError(f"model output has no command list: {raw[:200]!r}")
        parsed["say"] = str(parsed.get("say", "")).strip()
        return raw, parsed
