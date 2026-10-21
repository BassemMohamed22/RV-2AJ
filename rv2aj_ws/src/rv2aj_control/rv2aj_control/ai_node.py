"""
AI operator for the RV-2AJ stack: natural language -> local LLM -> validator
and intent guards -> operator confirmation -> MoveIt-planned motion.

Topics / services (namespace /rv2aj):
  ai/request   std_msgs/String   text from the GUI (typed or push-to-talk)
  ai/state     std_msgs/String   JSON: phase, request, plan, response (latched)
  ai/confirm   std_srvs/Trigger  run the plan that is waiting for confirmation
  ai/cancel    std_srvs/Trigger  drop the pending plan / abort the running one

Safety:
  - Stop words on the raw text call the driver's STOP before anything else
    and never reach the model.
  - The model's commands must pass ai_brain.validate (forms, limits, a
    kinematic simulation of every move) and ai_brain.intent_check (every
    number, direction, joint and name must come from the operator's words).
    A failed plan is reported and never retried or modified.
  - Only read / stop plans run without Confirm. Motion also needs the servo
    to be on already; the AI can never turn it on.
  - All motion goes through the planner's MoveIt-checked actions.

Teaching:
  - Seed examples ship in config/ai_examples.json.
  - Every confirmed plan that ran successfully becomes an example.
  - "teach: <request> => <cmd>, <cmd>" stores a correction directly.
  - DEFINE_SKILL / SAVE_POINT teach skills and named positions.
"""

import json
import math
import os
import re
import shutil
import subprocess
import threading
import time
from datetime import datetime

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker
from std_srvs.srv import SetBool, Trigger

from rv2aj_interfaces.action import JointMove, TcpMove
from rv2aj_interfaces.msg import RobotStatus, SequencerStatus
from rv2aj_interfaces.srv import LineSignal, SavePoint, SetOverride

from . import ai_brain as brain
from . import ai_voice_fx as fx
from . import kinematics

DIR_WORDS = {"Z": ("up", "down", "yukarı", "aşağı"), "Y": ("left", "right", "sola", "sağa"),
             "R": ("forward", "back", "ileri", "geri")}


def joint_phrase(joint, delta, lang, done=False):
    """Natural words for a joint move: 'tilt the wrist down 10 degrees' /
    'bileği 10 derece aşağı eğ' instead of 'J5 +10'."""
    amount = f"{round(abs(delta), 1):g}"
    if lang == "tr":
        if joint == "J5":
            way = "aşağı" if delta > 0 else "yukarı"
            return f"bilek {amount} derece {way} eğildi" if done else f"bileği {amount} derece {way} eğ"
        names = {"J1": ("taban", "tabanı"), "J2": ("omuz", "omzu"), "J3": ("dirsek", "dirseği"),
                 "J6": ("bilek", "bileği")}
        subject, obj = names.get(joint, (joint, joint))
        sign = "" if delta > 0 else "eksi "
        verb_done = "çevrildi" if joint == "J6" else "döndürüldü"
        verb = "çevir" if joint == "J6" else "döndür"
        return f"{subject} {sign}{amount} derece {verb_done}" if done else f"{obj} {sign}{amount} derece {verb}"
    if joint == "J5":
        way = "down" if delta > 0 else "up"
        return f"wrist tilted {way} {amount} degrees" if done else f"tilt the wrist {way} {amount} degrees"
    names = {"J1": "the base", "J2": "the shoulder", "J3": "the elbow", "J6": "the wrist roll"}
    sign = "" if delta > 0 else "minus "
    name = names.get(joint, joint)
    return f"{name} turned {sign}{amount} degrees" if done else f"turn {name} {sign}{amount} degrees"


def dist(mm, lang):
    """Distance the way people say it: 8 santim / 25 milimetre."""
    if mm >= 10 and abs(mm / 10 - round(mm / 10)) < 1e-6:
        return f"{mm / 10:g} " + ("santim" if lang == "tr" else "centimetres")
    return f"{mm:g} " + ("milimetre" if lang == "tr" else "millimetres")


def num(value, lang):
    """Number for speech/display: '20', 'minus 20' / 'eksi 20'."""
    text = f"{abs(value):g}"
    return text if value >= 0 else ("eksi " if lang == "tr" else "minus ") + text


PIPER_LENGTH_SCALE = 1.05  # the offline voice's pace, as in the sample the operator chose

class Speaker:
    """Offline neural text-to-speech with Piper, optionally with the Jarvis-style
    processing (ai_voice_fx), one voice per language, played with aplay. An
    interface sound can lead each sentence. Falls back to spd-say if Piper or
    the voice files are missing. Speech is queued; stop() flushes it.
    on_speaking(bool) tells the node when audio is playing (the GUI's stop-word
    listener ignores the robot's own voice)."""

    def __init__(self, engine, voices, logger, style="jarvis", on_speaking=lambda on: None):
        self.engine, self.voices, self.log, self.style = engine, voices, logger, style
        self.edge_voices = {"tr": "tr-TR-AhmetNeural", "en": "en-GB-RyanNeural"}
        self._edge_warned = False
        self.on_speaking = on_speaking
        self.loaded = {}
        self.queue = []
        self.cond = threading.Condition()
        self.player = None
        threading.Thread(target=self._run, daemon=True).start()

    def say(self, text, lang, sfx=None):
        if self.engine == "none" or not (text or sfx):
            return
        with self.cond:
            self.queue.append((text, lang, sfx))
            self.cond.notify()

    def sound(self, name):
        self.say("", "tr", sfx=name)

    def stop(self):
        with self.cond:
            self.queue.clear()
        player = self.player
        if player is not None and player.poll() is None:
            player.kill()

    def preload(self, lang):
        """Load the offline voice in the background so the first reply is not delayed."""
        if self.engine in ("piper", "edge"):
            threading.Thread(target=self._voice, args=(lang,), daemon=True).start()

    def _voice(self, lang):
        if lang not in self.loaded:
            path = os.path.expanduser(self.voices.get(lang, ""))
            try:
                from piper import PiperVoice
                self.loaded[lang] = PiperVoice.load(path) if os.path.exists(path) else None
            except Exception as e:  # piper not installed or bad voice file
                self.log.warn(f"Piper voice for {lang} unavailable ({e}); using spd-say")
                self.loaded[lang] = None
        return self.loaded[lang]

    def _play(self, data):
        self.player = subprocess.Popen(["aplay", "-q", "-"], stdin=subprocess.PIPE,
                                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.player.communicate(data, timeout=60)

    def _edge_wav(self, text, lang):
        voice = self.edge_voices.get(lang)
        if not voice:
            return None
        try:
            samples, sr = fx.edge_speech(text, voice=voice)
        except Exception as e:  # offline / service unavailable: Piper takes over
            if not self._edge_warned:
                self.log.warn(f"online voice unavailable ({e}); using the offline Piper voice")
                self._edge_warned = True
            return None
        if self.style in ("light", "jarvis"):
            samples = fx.light(samples, sr)
        return fx.wav_bytes(samples, sr)

    def _speech_wav(self, voice, text):
        import io
        import wave
        import numpy as np
        from piper import SynthesisConfig
        jarvis = self.style == "jarvis-heavy"
        config = SynthesisConfig(length_scale=fx.synth_length_scale() if jarvis else PIPER_LENGTH_SCALE,
                                 noise_scale=0.5 if jarvis else None, noise_w_scale=0.6 if jarvis else None)
        buf = io.BytesIO()
        with wave.open(buf, "wb") as w:
            voice.synthesize_wav(text, w, syn_config=config)
        if not jarvis:
            return buf.getvalue()
        buf.seek(0)
        with wave.open(buf) as w:
            sr = w.getframerate()
            raw = np.frombuffer(w.readframes(w.getnframes()), dtype=np.int16)
        return fx.wav_bytes(fx.jarvis(raw, sr), sr)

    def _run(self):
        while True:
            with self.cond:
                while not self.queue:
                    self.cond.wait()
                text, lang, sfx = self.queue.pop(0)
            self.on_speaking(True)
            try:
                if sfx:
                    with open(fx.sound_file(sfx), "rb") as f:
                        self._play(f.read())
                if not text:
                    continue
                wav = self._edge_wav(text, lang) if self.engine == "edge" else None
                voice = self._voice(lang) if self.engine in ("piper", "edge") and wav is None else None
                if wav is not None:
                    self._play(wav)
                elif voice is not None:
                    self._play(self._speech_wav(voice, text))
                elif shutil.which("spd-say"):
                    args = ["spd-say", "-w"] + (["-l", "tr"] if lang == "tr" else []) + ["--", text]
                    self.player = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    self.player.wait(timeout=60)
            except Exception as e:
                self.log.warn(f"speech failed: {e}")
            finally:
                with self.cond:
                    idle = not self.queue
                if idle:
                    self.on_speaking(False)


PLANNER_MAX_JOINT_DEG = 45.0
PLANNER_MAX_TCP_MM = 50.0
ACTION_TIMEOUT_S = 120.0


class Aborted(Exception):
    pass


class AINode(Node):
    def __init__(self):
        super().__init__("ai")
        p = self.declare_parameter
        self.model = p("model", "rv2aj-qwen").value
        self.llm = brain.Ollama(model=self.model, url=p("ollama_url", "http://127.0.0.1:11434").value)
        self.k_examples = int(p("k_examples", 8).value)
        # "guesses": a plan that clearly matches the operator's words runs at once; only
        # guesses (and auto-corrected plans) wait for "Onayla" / "evet". "all": every
        # motion waits (the original behaviour). Operator's choice on 2026-09-27: guesses.
        self.confirm_mode = str(p("confirm_mode", "guesses").value)
        # The fine-tuned model (rv2aj-*) learned the task: it gets the short prompt
        # (plus the fact sheet for questions) it was trained with, no examples.
        self.compact_prompt = bool(p("compact_prompt", self.model.startswith("rv2aj")).value)
        self.plan_ttl_s = float(p("plan_ttl_s", 120.0).value)
        self.language = p("language", "auto").value          # auto: answer in the request's language
        self.speaker = Speaker(p("tts", "edge").value if p("speak", True).value else "none", {
            "tr": p("voice_tr", "~/.local/share/piper-voices/tr_TR-dfki-medium.onnx").value,
            "en": p("voice_en", "~/.local/share/piper-voices/en_US-lessac-medium.onnx").value,
        }, self.get_logger(), style=p("voice_style", "plain").value,
            on_speaking=lambda on: self._update(speaking=on))
        self.sounds = bool(p("sounds", True).value)
        fx.prebuild()
        self.lang = "tr" if self.language != "en" else "en"
        self.speaker.preload(self.lang)
        brain.LANG = self.lang
        limits = list(p("joint_limits_deg", [v for j in brain.MOVE_JOINTS for v in brain.DEFAULT_LIMITS[j]]).value)
        self.limits = {j: (float(limits[2 * i]), float(limits[2 * i + 1])) for i, j in enumerate(brain.MOVE_JOINTS)}
        self.home_fallback = list(p("home_joints_deg", [-118.49, -101.71, 186.64, 0.00, -127.75, -279.42]).value)
        memory_file = p("memory_file", "~/.ros/rv2aj_ai_memory.json").value
        self.log_file = os.path.expanduser(p("log_file", "~/.ros/rv2aj_ai_log.txt").value)
        examples_file = p("examples_file", "").value or os.path.join(
            get_package_share_directory("rv2aj_control"), "config", "ai_examples.json")
        with open(examples_file) as f:
            self.memory = brain.Memory(memory_file, json.load(f))

        group = ReentrantCallbackGroup()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, "ai/state", latched)
        self.preview_pub = self.create_publisher(Marker, "ai/path_preview", latched)
        self.create_subscription(RobotStatus, "status", self._on_status, 10, callback_group=group)
        self.create_subscription(SequencerStatus, "sequencer/status", self._on_sequencer, latched,
                                 callback_group=group)
        self.create_subscription(String, "ai/request", self._on_request, 10, callback_group=group)
        self.create_service(Trigger, "ai/confirm", self._confirm, callback_group=group)
        self.create_service(Trigger, "ai/cancel", self._cancel, callback_group=group)

        self.srv_clients = {
            "stop": self.create_client(Trigger, "stop", callback_group=group),
            "servo": self.create_client(SetBool, "servo", callback_group=group),
            "gripper/open": self.create_client(Trigger, "gripper/open", callback_group=group),
            "gripper/close": self.create_client(Trigger, "gripper/close", callback_group=group),
            "set_override": self.create_client(SetOverride, "set_override", callback_group=group),
            "points/save": self.create_client(SavePoint, "points/save", callback_group=group),
            "line/set": self.create_client(LineSignal, "line/set", callback_group=group),
        }
        self.joint_action = ActionClient(self, JointMove, p("joint_action", "planned/joint_move").value,
                                         callback_group=group)
        self.tcp_action = ActionClient(self, TcpMove, p("tcp_action", "planned/tcp_move").value,
                                       callback_group=group)
        self.sequencer_params = AsyncParameterClient(self, "sequencer")

        self.robot = None
        self.points = {}
        self.home = None
        self.lock = threading.Lock()
        self.generation = 0
        self.pending = None          # {"plan", "commands", "request", "source", "time"}
        self.executing = False
        self.abort = threading.Event()
        self.goal_handle = None
        self.state = {"phase": "idle", "llm": "checking…"}
        self.history = []            # moves the operator had executed recently (for "biraz daha" etc.)
        self.last_plan = None        # the last plan the operator saw (shown or run): what "daha" refers to
        self.conversation = []       # recent exchanges incl. what really happened (for the model)
        self.chat_log = []           # [{"who": "you"/"robot", "text"}] shown in the GUI
        self.turn = None             # the exchange in progress
        self.history_ttl_s = float(p("history_ttl_s", 900.0).value)
        self._publish()

        threading.Thread(target=self._startup, daemon=True).start()
        self.get_logger().info(f"AI operator ready (model {self.model}); motion goes through MoveIt")

    # -- bookkeeping -----------------------------------------------------------------

    def _startup(self):
        ok, message = self.llm.check()
        if not ok and "not reachable" in message:
            self._start_ollama()
            ok, message = self.llm.check()
        self._update(llm=message, llm_ok=ok)
        if ok:
            self.llm.warm_up()
            greeting = self.T("Systems online. At your service.", "Sistemler çevrimiçi hocam. Emrinizdeyim.")
            self.speaker.say(greeting, self.lang, sfx="startup" if self.sounds else None)
            self._chat("robot", greeting)
        self._fetch_home()

    def _start_ollama(self):
        """Start the per-user Ollama server (~/.local/ollama) if it is installed
        but not running, limited to one model / one request for 4 GB VRAM."""
        binary = shutil.which("ollama") or os.path.expanduser("~/.local/ollama/bin/ollama")
        if not os.path.exists(binary):
            return
        settings = dict(OLLAMA_MAX_LOADED_MODELS="1", OLLAMA_NUM_PARALLEL="1", OLLAMA_HOST="127.0.0.1:11434",
                        OLLAMA_FLASH_ATTENTION="1", OLLAMA_KV_CACHE_TYPE="q8_0", OLLAMA_CONTEXT_LENGTH="4096",
                        # The runner (llama.cpp llama-server) keeps a RAM prompt cache of up to 8 GB
                        # by default: it grew ~30 MB per request until the OOM killer struck. Off.
                        LLAMA_ARG_CACHE_RAM="0")
        if shutil.which("systemd-run"):
            # Its own systemd unit with a memory cap, as a second safety net: if the runner ever
            # grows again, only Ollama is killed (and restarted), not VS Code or the robot stack.
            subprocess.run(["systemctl", "--user", "reset-failed", "rv2aj-ollama"], capture_output=True)
            cmd = ["systemd-run", "--user", "--unit=rv2aj-ollama", "--collect", "-p", "MemoryMax=1536M",
                   "-p", "MemorySwapMax=256M", "-p", "Restart=on-failure", "-p", "RestartSec=2"]
            cmd += [f"--setenv={k}={v}" for k, v in settings.items()] + [binary, "serve"]
            subprocess.run(cmd, capture_output=True)
        else:
            log = open(os.path.expanduser("~/.local/ollama/serve.log"), "a") if os.path.isdir(
                os.path.expanduser("~/.local/ollama")) else subprocess.DEVNULL
            subprocess.Popen([binary, "serve"], env=dict(os.environ, **settings), stdout=log, stderr=log,
                             start_new_session=True)
        self.get_logger().info(f"started {binary} serve")
        for _ in range(40):
            time.sleep(0.25)
            if self.llm.check()[0] or "not pulled" in self.llm.check()[1]:
                break

    def _fetch_home(self):
        try:
            if self.sequencer_params.wait_for_services(timeout_sec=5.0):
                future = self.sequencer_params.get_parameters(["home_joints_deg"])
                deadline = time.monotonic() + 5.0
                while not future.done() and time.monotonic() < deadline:
                    time.sleep(0.05)
                if future.done() and future.result() is not None:
                    values = list(future.result().values[0].double_array_value)
                    if len(values) == 6:
                        self.home = values
                        return
        except Exception as e:  # fall back to our own parameter
            self.get_logger().warn(f"could not read the sequencer's home pose: {e}")
        self.home = self.home_fallback

    def _log(self, event, **data):
        line = f"{datetime.now():%Y-%m-%d %H:%M:%S.%f}"[:-3] + f"  {event:<14} " + json.dumps(
            data, ensure_ascii=False, default=str)
        try:
            os.makedirs(os.path.dirname(self.log_file), exist_ok=True)
            with open(self.log_file, "a", encoding="utf-8") as f:
                f.write(line + "\n")
        except OSError:
            pass

    def _preview(self, plan, state="pending"):
        """Draw the tool path of a plan in RViz: orange while it waits for
        confirmation, green once done. Joint moves are not straight lines and
        are not drawn."""
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns, marker.id = "ai_path", 0
        points = []
        for step in (plan.steps if plan else []):
            pts = (step.action or {}).get("points")
            if pts:
                points.extend(pts[1:] if points and points[-1] == pts[0] else pts)
        if len(points) < 2:
            marker.action = Marker.DELETE
        else:
            marker.type, marker.action = Marker.LINE_STRIP, Marker.ADD
            marker.scale.x = 0.004
            marker.pose.orientation.w = 1.0
            marker.color = ColorRGBA(r=1.0, g=0.55, b=0.0, a=0.95) if state == "pending" else \
                ColorRGBA(r=0.2, g=0.8, b=0.35, a=0.95)
            marker.points = [Point(x=p[0] / 1000.0, y=p[1] / 1000.0, z=p[2] / 1000.0) for p in points]
        self.preview_pub.publish(marker)

    def _previous(self):
        """What a follow-up ("daha", "tekrar", "büyüt") refers to: the last plan
        the operator saw, unless they cancelled it; else the last executed move.
        The model sees the same conversation, so it and the guards agree."""
        now = time.monotonic()
        last = self.last_plan
        if last and not last.get("cancelled") and now - last["time"] < self.history_ttl_s:
            return {"request": last["request"], "commands": last["commands"]}
        recent = self._recent()
        return recent[-1] if recent else None

    def _remember_plan(self, request, commands):
        self.last_plan = {"request": request, "commands": list(commands), "time": time.monotonic()}

    def _recent(self):
        """The last few things the operator had done (for "biraz daha", "tekrar", "geri al")."""
        now = time.monotonic()
        self.history = [t for t in self.history if now - t["time"] < self.history_ttl_s]
        return [{"request": t["request"], "commands": t["commands"]} for t in self.history]

    def _update(self, **fields):
        with self.lock:
            self.state.update(fields)
        self._publish()

    def _publish(self):
        with self.lock:
            text = json.dumps(self.state, default=str)
        self.state_pub.publish(String(data=text))

    def _chat(self, who, text):
        self.chat_log = (self.chat_log + [{"who": who, "text": text}])[-40:]
        self._update(chat=self.chat_log)

    def _close_turn(self, result):
        turn, self.turn = self.turn, None
        if turn is not None:
            turn["result"] = result
            self.conversation = (self.conversation + [turn])[-6:]

    SFX = {"warn": "confirm", "ok": "done", "error": "error"}

    def _respond(self, text, kind="info", spoken=None, sfx=None, **fields):
        """Show `text` and speak `spoken` (defaults to text) in the current language,
        led by an interface sound that matches the kind of response."""
        self.chat_log = (self.chat_log + [{"who": "robot", "text": text}])[-40:]
        fields["chat"] = self.chat_log
        if fields.get("phase") == "idle":
            self._close_turn(text)
        self._update(response=text, kind=kind, **fields)
        self._log("RESPONSE", text=text, spoken=spoken, kind=kind, lang=self.lang)
        cue = (sfx or self.SFX.get(kind)) if self.sounds else None
        self.speaker.say(text if spoken is None else spoken, self.lang, sfx=cue)

    def T(self, en, tr):
        return tr if self.lang == "tr" else en

    def _set_lang(self, text):
        self.lang = self.language if self.language in ("tr", "en") else ("tr" if brain.is_turkish(text) else "en")
        brain.LANG = self.lang

    def _describe(self, step):
        """Short, speakable description of what a step will do."""
        a, L = step.action or {}, self.lang
        kind = a.get("type")
        if kind == "tcp":
            w = DIR_WORDS.get(a["axis"])
            d = (w[0] if a["mm"] > 0 else w[1]) if w else a["axis"]
            d_tr = (w[2] if a["mm"] > 0 else w[3]) if w else a["axis"]
            got, req = abs(a["mm"]), abs(a.get("requested", a["mm"]))
            if got < req - 0.5:
                return self.T(f"move {d} {dist(got, 'en')} instead of {dist(req, 'en')} because of the limit",
                              f"sınır nedeniyle {dist(req, 'tr')} yerine {dist(got, 'tr')} {d_tr} git")
            return self.T(f"move {d} {dist(got, 'en')}", f"{dist(got, 'tr')} {d_tr} git")
        if kind == "joint_rel":
            text = joint_phrase(a["joint"], a["delta"], L)
            req = a.get("requested", a["delta"])
            if abs(req - a["delta"]) > 0.05:
                text = self.T(f"because of the limit, {text} instead of {abs(req):g}",
                              f"sınır nedeniyle {abs(req):g} yerine {text}")
            return text
        if kind == "note":
            return a["text"]
        if kind == "path":
            text = brain.shape_label(a["shape"], a["dims"], a["plane"], self.lang) + self.T(" draw", " çiz")
            if a.get("shift_mm"):
                d = math.sqrt(sum(x * x for x in a["shift_mm"]))
                text = self.T(f"first move {dist(round(d), 'en')} to where it fits, then ",
                              f"önce sığacağı yere {dist(round(d), 'tr')} gidip ") + text
            if a.get("cause"):
                text = self.T(f"because of {a['cause']}, ", f"{a['cause']} nedeniyle ") + text
            return text
        if kind == "joint_abs" and a.get("label") == "manoeuvre":
            mv = a["manoeuvre"]
            w = {"UP": "yukarı", "DOWN": "aşağı", "LEFT": "sola", "RIGHT": "sağa", "FORWARD": "ileri", "BACK": "geri"}
            tilt = self.T(f", tilting the wrist {mv['pitch_change']:+.0f} degrees",
                          f", bileği {abs(mv['pitch_change']):.0f} derece eğerek") if abs(mv["pitch_change"]) >= 1 else ""
            return self.T(f"a straight line is blocked, so I reach the point {dist(mv['mm'], 'en')} "
                          f"{mv['word'].lower()} by another path{tilt}",
                          f"düz gidemediğim için aynı noktaya, {dist(mv['mm'], 'tr')} {w.get(mv['word'], mv['word'])}, "
                          f"farklı bir yoldan gideyim{tilt}")
        if kind == "joint_abs":
            label = a.get("label", "")
            if label == "home":
                return self.T("go home", "başlangıç konumuna git")
            if label.startswith("point "):
                return self.T(f"go to {label}", f"{label[6:]} noktasına git")
            return self.T("move the joints to the target", "eklemleri hedefe getir")
        if kind == "gripper":
            return self.T("open the gripper" if a["open"] else "close the gripper",
                          "tutucuyu aç" if a["open"] else "tutucuyu kapat")
        if kind == "speed":
            return self.T(f"set speed to {a['percent']} percent", f"hızı yüzde {a['percent']} yap")
        if kind == "line":
            return step.reason
        if kind == "wait":
            return self.T(f"wait {a['s']:g} seconds", f"{a['s']:g} saniye bekle")
        if kind == "save_point":
            return self.T(f"save this position as {a['name']}", f"bu konumu {a['name']} olarak kaydet")
        if kind == "define_skill":
            return self.T(f"learn the skill {a['name']}", f"{a['name']} becerisini öğren")
        if kind == "forget_skill":
            return self.T(f"forget the skill {a['name']}", f"{a['name']} becerisini unut")
        return ""

    def _on_status(self, msg):
        self.robot = msg

    def _on_sequencer(self, msg):
        self.points = {p.name: {"joints": list(p.joints_deg), "gripper": p.gripper} for p in msg.points}

    def world(self):
        r = self.robot
        joints = None
        if r is not None and r.connected:
            joints = {j: (None if math.isnan(v) else float(v)) for j, v in zip(brain.ALL_JOINTS, r.joints_deg)}
        return brain.World(joints=joints, connected=bool(r and r.connected), servo_on=bool(r and r.servo_on),
                           gripper=r.gripper if r else "unknown", points=dict(self.points),
                           home=self.home, skills=dict(self.memory.skills))

    # -- requests --------------------------------------------------------------------

    def _on_request(self, msg):
        raw = msg.data.strip()
        if not raw:
            return
        self._log("REQUEST", text=raw)
        # 1. Emergency stop on the RAW text, before any clean-up or model.
        if brain.is_emergency_stop(raw):
            self._set_lang(raw)
            self._chat("you", raw)
            self._emergency_stop(raw)
            return
        text = raw if raw.startswith((">", "teach:")) else brain.fix_transcript(raw)
        if text != raw:
            self._log("FIXED_TEXT", raw=raw, text=text)
        self._set_lang(text)
        # A spoken "onaylıyorum" must not throw the waiting plan away: confirming
        # stays a click (the operator's rule), so say where to click. "iptal" cancels.
        if self.pending is not None and brain.is_confirm_word(text):
            self._chat("you", raw)
            self._respond(self.T("Please press Confirm on the screen to run it.",
                                 "Çalıştırmak için ekrandaki Onayla düğmesine basın hocam."), "warn",
                          phase="awaiting_confirmation")
            return
        if self.pending is not None and brain.is_cancel_word(text):
            self._chat("you", raw)
            self._cancel(Trigger.Request(), Trigger.Response())
            return
        self._chat("you", text)
        self.turn = {"request": text, "commands": [], "say": "", "result": "", "time": time.monotonic()}
        # 1. Emergency stop on the RAW text, before any model sees it.
        if brain.is_emergency_stop(text):
            self._emergency_stop(text)
            return
        with self.lock:
            self.generation += 1
            generation = self.generation
            self.pending = None
        self._update(phase="thinking", request=text, plan=None, response="", kind="info")
        self._preview(None)
        threading.Thread(target=self._handle, args=(text, generation), daemon=True).start()

    def _emergency_stop(self, text):
        self._stop_everything()
        with self.lock:
            self.generation += 1
            self.pending = None
        self._log("EMERGENCY_STOP", text=text)
        self._respond(self.T("STOP sent.", "Durdum hocam."), "error", phase="idle", request=text, plan=None, sfx="stop")

    def _stop_everything(self):
        self.abort.set()
        self.speaker.stop()
        client = self.srv_clients["stop"]
        if client.service_is_ready():
            client.call_async(Trigger.Request())
        handle = self.goal_handle
        if handle is not None:
            handle.cancel_goal_async()

    def _handle(self, text, generation):
        world = self.world()
        source, say = "llm", ""
        if text.lower().startswith("teach:"):
            self._teach(text[6:])
            return
        previous = self._previous()
        followup = None if text.startswith(">") else brain.resolve_followup(text, previous)
        recalled = None if text.startswith(">") else self.memory.recall(text)
        skill = brain.find_name(self.memory.skills, " ".join(re.findall(r"[A-Za-z0-9_-]+", text)))
        if text.startswith(">"):
            commands, source = [c.strip() for c in text[1:].split(",") if c.strip()], "direct"
        elif followup:
            commands, source, say = followup, "context", ""
            self._log("FOLLOWUP", request=text, previous=previous, commands=commands)
        elif skill:
            commands, source, say = [f"RUN_SKILL {skill}"], "recall", f"Doing {skill}."
        elif recalled:
            # Taught phrases are the operator's own mapping: used as-is, still validated.
            commands, source, say = list(recalled["commands"]), "recall", recalled.get("say", "")
            self._log("RECALL", request=text, commands=commands, from_source=recalled.get("source"))
        else:
            now = time.monotonic()
            recent = [t for t in self.conversation if now - t["time"] < self.history_ttl_s][-4:]
            if brain.is_continuation(text):
                recent = recent[-1:]  # a follow-up refers to the last exchange only
            messages = brain.build_messages(text, world, self.limits, self.memory, k_examples=self.k_examples,
                                            compact=self.compact_prompt,
                                            history=recent)
            if self.sounds:
                self.speaker.sound("think")
            try:
                raw, parsed = self.llm.chat(messages)
            except brain.LLMError as e:
                if generation == self.generation:
                    self._log("LLM_ERROR", error=str(e))
                    self._respond(self.T(f"The language model is not available: {e}",
                                         f"Dil modeli kullanılamıyor: {e}"), "error", phase="idle",
                                  spoken=self.T("The language model is not available.", "Dil modeli şu an kullanılamıyor hocam."))
                return
            self._log("LLM_OUTPUT", model=self.model, raw=raw)
            commands, say = parsed["commands"], parsed["say"]
            if self.lang == "tr" and say and not brain.is_turkish(say):
                say = ""  # the operator wants Turkish only; our own Turkish text is used instead
        if source == "llm":
            asked = brain.state_question(text)
            if (asked and not commands) or (not commands and brain.claims_state(say)):
                # Never let the model describe the robot's state from imagination: read it.
                self._log("STATE_FROM_ROBOT", request=text, model_say=say, forced=asked or "WHERE")
                commands, say = [asked or "WHERE"], self.T("Let me check.", "Hemen bakıyorum.")
            elif commands and brain.claims_state(say):
                say = ""  # a move plan whose reply invents numbers: drop the reply, keep the plan
            elif commands and re.search(r"bakıyorum|kontrol ediyorum|let me check", say, re.I) and \
                    not any(brain.match(brain.normalize(c))[0] == "read" for c in commands):
                say = ""  # "Hemen bakıyorum" copied from an example in front of a move
            had_commands = bool(commands)
            commands, dropped = brain.drop_ungrounded_waits(text, commands)
            if dropped:
                self._log("DROPPED_WAIT", request=text, dropped=dropped)
            if commands and say and not asked and brain.is_question(text):
                # A question ("tutucun ne?", "servoyu nasıl açarım?") answered in "say": commands its
                # words do not ask for are dropped (the fine-tuned model sometimes adds one).
                kept = [c for c in commands if brain.intent_check(text, [c], self._previous()) is None]
                if len(kept) < len(commands):
                    self._log("DROPPED_ON_QUESTION", request=text, dropped=[c for c in commands if c not in kept])
                    commands = kept
            if (dropped or had_commands) and not commands and re.match(
                    r"^(hemen|tamam|tabii|peki|olur|emredersiniz|anlaşıldı)\b[^.!?]{0,25}[.!]?$", say.strip(), re.I):
                # "Hemen yapıyorum hocam." belonged to the dropped action: say something that fits.
                say = self.T("Sure. Tell me whenever you're ready.", "Tabii hocam. Hazır olduğunuzda söylemeniz yeterli.")
        if self.turn is not None:
            self.turn.update(commands=list(commands), say=say)
        if generation != self.generation:
            return  # superseded by a newer request or a stop

        if not commands:
            self._respond(say or self.T("Tell me which way and how far, e.g. 'move left 2 cm'.",
                                        "Hangi yöne ve ne kadar gideyim hocam? Örneğin: “2 santim sola git”."),
                          "info", phase="idle")
            return
        if source == "llm":
            # The model understood the request but wrote the wrong amount or the
            # opposite direction: correct it to the operator's words, do not refuse.
            commands, fixed_dir = brain.repair_directions(text, commands)
            commands, fixed_num = brain.repair_numbers(text, commands)
            repaired = bool(fixed_dir or fixed_num)
            if fixed_dir or fixed_num:
                self._log("REPAIRED", request=text, changes=fixed_dir + fixed_num)
                say = ""  # the model's own sentence named the wrong value
                if self.turn is not None:
                    self.turn.update(commands=list(commands), say=say)
        plan = brain.validate(commands, world, self.limits, kinematics)
        guard = brain.intent_check(text, commands, previous) if source == "llm" and plan.ok else None
        self._log("VALIDATOR", source=source, commands=commands, ok=plan.ok, reason=plan.reason, guard=guard,
                  steps=plan.to_dict()["steps"])
        plan_reason = None
        # Only a validator (safety/limits) rejection makes the model think again. A plan
        # that is safe but not literally in the operator's words is shown and asked about.
        read_only = all(st.kind == brain.Kind.READ for st in plan.steps)
        rethink = guard and (read_only or brain.plan_unrelated(text, commands, previous))
        if (not plan.ok or rethink) and source == "llm" and not brain.is_hard_rule(commands) \
                and generation == self.generation:
            # A read nobody asked for (e.g. LINE_STATUS for "yukarı çık") is re-thought, not run.
            first_reason = plan.reason if not plan.ok else guard
            self._log("REPLAN", request=text, rejected=commands, reason=first_reason)
            feedback = messages + [
                {"role": "assistant", "content": json.dumps({"say": say, "commands": commands}, ensure_ascii=False)},
                {"role": "user", "content": (
                    f"That plan cannot be used: {first_reason}. Think of another way to do what the operator asked: "
                    "different commands, a different order, or an intermediate move. Use only the operator's numbers "
                    "and directions. If there is truly no way, send no commands and say in Turkish what the operator "
                    "could do instead.")}]
            try:
                raw2, parsed2 = self.llm.chat(feedback)
                self._log("LLM_REPLAN", raw=raw2)
                commands2, say2 = parsed2["commands"], parsed2["say"]
                if commands2:
                    commands2, _ = brain.drop_ungrounded_waits(text, commands2)
                    commands2, _ = brain.repair_directions(text, commands2)
                    commands2, _ = brain.repair_numbers(text, commands2)
                    plan2 = brain.validate(commands2, world, self.limits, kinematics)
                    guard2 = brain.intent_check(text, commands2, previous) if plan2.ok else None
                    # Take the second idea if it is safe and not worse than the first.
                    if plan2.ok and (not plan.ok or not guard2):
                        blocked_first = not plan.ok
                        commands, plan, guard = commands2, plan2, guard2
                        say = (say2 if self.lang != "tr" or brain.is_turkish(say2) else "") or (self.T(
                            "My first idea did not work, so here is another way.",
                            "İlk planım olmadı hocam, başka bir yol buldum.") if blocked_first else "")
                        if self.turn is not None:
                            self.turn.update(commands=list(commands), say=say)
                elif say2 and (self.lang != "tr" or brain.is_turkish(say2)):
                    first_reason = f"{first_reason}. {say2}"
            except brain.LLMError:
                pass
            if not plan.ok:
                plan_reason = first_reason
        if not plan.ok:
            if self.turn is not None:
                self.turn.update(commands=[])  # nothing was done: an invalid plan must not be copied later
            reason = plan_reason or plan.reason
            self._respond(self.T(f"I won't do that: {reason.rstrip('.')}.", f"Bunun için güvenli bir yol bulamadım hocam: {reason.rstrip('.')}."),
                          "error", phase="idle", plan=plan.to_dict(),
                          spoken=self.T("I won't do that. The reason is on the screen.",
                                        "Bunun için güvenli bir yol bulamadım hocam. Ayrıntılar ekranda."))
            return
        uncertain = bool(guard) and any(st.kind != brain.Kind.READ for st in plan.steps)
        if guard:
            self._log("UNCERTAIN", request=text, commands=commands, guard=guard)
        sure = not uncertain and not (source == "llm" and repaired)
        run_now = not plan.needs_confirmation and not uncertain
        if self.confirm_mode == "guesses" and sure and plan.needs_confirmation and \
                (world.servo_on or not plan.has_motion):
            run_now = True  # clear command: no "Onaylıyor musunuz?" (a motion with the servo off still waits)
        if run_now:
            self._remember_plan(text, commands)
            self._update(plan=plan.to_dict())
            self._execute(plan, text, commands, source, confirmed=False, lead=say if source == "llm" else "")
            return
        self._preview(plan, "pending")
        self._remember_plan(text, commands)
        with self.lock:
            self.pending = {"plan": plan, "commands": commands, "request": text, "source": source,
                            "time": time.monotonic()}
        warn = "" if world.servo_on or not plan.has_motion else self.T(
            " The servo is off: turn it on first.", " Servo kapalı hocam, önce servoyu açın.")
        steps = [d for d in (self._describe(st) for st in plan.steps) if d]
        spoken = self.T("I will " + ", then ".join(steps) + ". Please confirm." + warn,
                        ", sonra ".join(steps).capitalize() + ". Onaylıyor musunuz hocam?" + warn)
        shown = self.T("This will: " + "; ".join(plan.effects) + ". Confirm?" + warn,
                       "Şunu yapacağım: " + ", sonra ".join(steps) + ". Onaylıyor musunuz hocam?" + warn)
        lead = (say.rstrip() + " ") if say and source == "llm" and len(say) < 120 else ""
        if uncertain:
            # Not literally what was said: say how it was understood and let the operator decide.
            lead = self.T("You did not say it exactly, so this is my best guess. ",
                          "Tam olarak böyle demediniz hocam, ben şöyle anladım. ")
            shown = shown.replace(self.T("Confirm?", "Onaylıyor musunuz hocam?"),
                                  self.T("Is that right? Confirm or cancel.", "Doğru mu? Onaylayın ya da iptal edin."))
            spoken = spoken.replace(self.T("Please confirm.", "Onaylıyor musunuz hocam?"),
                                    self.T("Is that right?", "Doğru mu hocam?"))
        self._respond(lead + shown, "warn", spoken=lead + spoken, phase="awaiting_confirmation", plan=plan.to_dict())

    def _teach(self, body):
        request, sep, rest = body.partition("=>")
        commands = [c.strip() for c in rest.split(",") if c.strip()]
        if not sep or not request.strip() or not commands:
            self._respond(self.T("Use: teach: <what you say> => <command>, <command>",
                                 "Kullanım: teach: <söylediğiniz> => <komut>, <komut>"), "warn", phase="idle")
            return
        for c in commands:
            if brain.match(brain.normalize(c))[0] is None:
                self._respond(self.T(f"I won't learn that: {c!r} is not a known command.",
                                     f"Bunu öğrenmeyeceğim: {c!r} bilinen bir komut değil."), "error", phase="idle")
                return
        self.memory.add_example(request.strip(), commands, "taught")
        self._log("TAUGHT", request=request.strip(), commands=commands)
        self._respond(self.T(f"Learned: “{request.strip()}” means {', '.join(commands)}.",
                             f"Öğrendim: “{request.strip()}” = {', '.join(commands)}."), "ok", phase="idle",
                      spoken=self.T("Learned.", "Öğrendim."))

    # -- confirmation ----------------------------------------------------------------

    def _confirm(self, request, response):
        with self.lock:
            pending, self.pending = self.pending, None
        if pending is None:
            response.success, response.message = False, "nothing is waiting for confirmation"
            return response
        if time.monotonic() - pending["time"] > self.plan_ttl_s:
            response.success, response.message = False, "that plan expired; ask again"
            self._respond(self.T("That plan expired; ask again.", "Bu planın süresi doldu; tekrar isteyin."),
                          "warn", phase="idle")
            return response
        world = self.world()
        plan = brain.validate(pending["commands"], world, self.limits, kinematics)  # from the pose now
        self._log("CONFIRM", request=pending["request"], ok=plan.ok, reason=plan.reason)
        if not plan.ok:
            response.success, response.message = False, f"no longer valid: {plan.reason}"
            self._respond(self.T(f"No longer valid: {plan.reason}.", f"Artık geçerli değil: {plan.reason}."),
                          "error", phase="idle", plan=plan.to_dict(),
                          spoken=self.T("The plan is no longer valid.", "Plan artık geçerli değil."))
            return response
        if plan.has_motion and not world.servo_on:
            with self.lock:
                self.pending = pending  # keep it: the operator can turn the servo on and confirm again
            response.success, response.message = False, "the servo is off; turn it on with the Servo ON button"
            self._respond(self.T("The servo is off. Turn it on with the Servo ON button, then confirm.",
                                 "Servo kapalı hocam. Servo ON düğmesiyle açın, sonra onaylayın."), "warn")
            return response
        if self.executing:
            response.success, response.message = False, "another plan is running"
            return response
        threading.Thread(target=self._execute, args=(plan, pending["request"], pending["commands"],
                                                     pending["source"], True), daemon=True).start()
        response.success, response.message = True, "executing"
        return response

    def _cancel(self, request, response):
        with self.lock:
            had = self.pending is not None
            self.pending = None
            self.generation += 1
        if had and self.last_plan is not None:
            self.last_plan["cancelled"] = True
        if self.executing:
            self._stop_everything()
        self._log("CANCEL", pending=had, executing=self.executing)
        self._preview(None)
        if not self.executing:
            if had:
                self._respond(self.T("Cancelled.", "Tamam hocam, iptal ettim."), "info", phase="idle")
            else:
                self._update(phase="idle")
        response.success, response.message = True, "cancelled"
        return response

    # -- execution -------------------------------------------------------------------

    def _execute(self, plan, request, commands, source, confirmed, lead=""):
        self.executing = True
        self.abort.clear()
        self._update(phase="executing")
        self._log("EXECUTE", request=request, confirmed=confirmed, commands=commands, lang=self.lang)
        shown, spoken = [], []
        try:
            for i, step in enumerate(plan.steps, 1):
                if self.abort.is_set():
                    raise Aborted("stopped")
                self._update(response=self.T(f"Step {i}/{len(plan.steps)}: {step.command}",
                                             f"Adım {i}/{len(plan.steps)}: {step.command}"), kind="info")
                report = self._run_step(step)
                if report:
                    shown.append(report[0])
                    spoken.append(report[1])
                self._log("STEP_DONE", step=step.command, report=report)
            self._preview(plan, "done")
            done = self.T("Done.", "Tamam hocam.")
            lead = (lead.rstrip() + " ") if lead else ""
            spoken_lead = lead or (self.T("", "Tamam hocam. ") if spoken and confirmed else "")
            self._respond(lead + (" ".join(shown) or done), "ok", phase="idle",
                          spoken=spoken_lead + (" ".join(spoken) or done))
            if confirmed and source == "llm":
                self.memory.add_example(request, commands, "confirmed")  # a hint for the model, not a rule
            self.history.append({"request": request, "commands": list(commands), "time": time.monotonic()})
            self.history = self.history[-4:]
        except Aborted:
            self._respond(self.T("Stopped.", "Durdum hocam."), "error", phase="idle")
        except Exception as e:  # service/action failure: report exactly, stop the plan
            self._log("EXEC_ERROR", error=str(e))
            self._respond(self.T(f"Stopped: {e}.", f"Hareket durdu hocam; planlayıcının mesajı: {e}."), "error", phase="idle",
                          spoken=self.T("The move failed. The reason is on the screen.",
                                        "Hareket yapılamadı hocam. Nedeni ekranda yazıyor."))
        finally:
            self.goal_handle = None
            self.executing = False

    def _wait(self, future, timeout, what):
        deadline = time.monotonic() + timeout
        while not future.done():
            if self.abort.is_set():
                raise Aborted("stopped")
            if time.monotonic() > deadline:
                raise RuntimeError(f"{what} timed out")
            time.sleep(0.02)
        return future.result()

    def _call(self, name, request):
        client = self.srv_clients[name]
        if not client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError(f"service {name} is not available")
        result = self._wait(client.call_async(request), 30.0, name)
        if result is None or not result.success:
            raise RuntimeError(f"{name} failed: {getattr(result, 'message', 'no response')}")
        return result

    def _goal(self, client, goal, what):
        if not client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(f"{what}: MoveIt planner not available")
        handle = self._wait(client.send_goal_async(goal), 10.0, what)
        if handle is None or not handle.accepted:
            raise RuntimeError(f"{what}: goal rejected (another move running?)")
        self.goal_handle = handle
        result = self._wait(handle.get_result_async(), ACTION_TIMEOUT_S, what).result
        self.goal_handle = None
        time.sleep(0.3)  # let the next joint poll arrive before reporting
        if not result.success:
            raise RuntimeError(f"{what}: {result.message}")
        return result

    def _pose_text(self):
        """Tool position from the joint angles (JPOSF is polled every cycle;
        the controller's own X/Y/Z is only refreshed about once a second)."""
        r = self.robot
        if r is None:
            return ""
        j = dict(zip(brain.ALL_JOINTS, r.joints_deg))
        p = kinematics.fk(*(j[k] for k in brain.MOVE_JOINTS))
        return self.T("tool", "araç") + f" X {p['X']:.0f}, Y {p['Y']:.0f}, Z {p['Z']:.0f} mm"

    def _run_step(self, step):
        """Run one validated step. Returns (shown, spoken) or None."""
        a, L = step.action, self.lang
        kind = a["type"]
        if kind == "read":
            text = self._read(a["what"])
            return text, text
        if kind == "note":
            return a["text"] + ".", a["text"] + "."
        if kind == "stop":
            self._call("stop", Trigger.Request())
            return (self.T("Stopped.", "Durdum hocam."),) * 2
        if kind == "servo_off":
            self._call("servo", SetBool.Request(data=False))
            return (self.T("Servo off.", "Servo kapatıldı."),) * 2
        if kind == "wait":
            end = time.monotonic() + a["s"]
            while time.monotonic() < end:
                if self.abort.is_set():
                    raise Aborted("stopped")
                time.sleep(0.05)
            return None
        if kind == "gripper":
            self._call("gripper/open" if a["open"] else "gripper/close", Trigger.Request())
            time.sleep(0.6)  # let the pneumatics finish
            text = self.T("Gripper opened." if a["open"] else "Gripper closed.",
                          "Tutucu açıldı." if a["open"] else "Tutucu kapatıldı.")
            return text, text
        if kind == "line":
            self._call("line/set", LineSignal.Request(signal=a["signal"], value=int(a["value"])))
            if a["signal"] == "error_code":
                text = self.T(f"Error code {a['value']} sent to the station.", f"İstasyona hata kodu {a['value']} gönderildi hocam.")
            else:
                what = self.T({"ready": "Station ready", "busy": "Busy"}[a["signal"]],
                              {"ready": "Hazır", "busy": "Meşgul"}[a["signal"]])
                text = self.T(f"{what} signal {'on' if a['value'] else 'off'}.",
                              f"{what} sinyali {'açıldı' if a['value'] else 'kapatıldı'} hocam.")
            return text, text
        if kind == "speed":
            self._call("set_override", SetOverride.Request(percent=a["percent"]))
            text = self.T(f"Speed {a['percent']} percent.", f"Hız yüzde {a['percent']}.")
            return text, text
        if kind == "joint_rel":
            jn, remaining = a["joint"], a["delta"]
            start = self.robot.joints_deg[brain.ALL_JOINTS.index(jn)]
            while abs(remaining) > 1e-6:
                chunk = max(-PLANNER_MAX_JOINT_DEG, min(PLANNER_MAX_JOINT_DEG, remaining))
                goal = JointMove.Goal(relative=True, joint=jn, offset_deg=float(chunk))
                result = self._goal(self.joint_action, goal, f"{jn} {chunk:+g}°")
                remaining -= chunk
            final = result.final_deg[brain.ALL_JOINTS.index(jn)]
            shown = self.T(f"Moved {jn} {a['delta']:+g}°: {start:.1f}° → {final:.1f}°.",
                           f"{jn} {a['delta']:+g}° döndü: {start:.1f}° → {final:.1f}°.")
            spoken = joint_phrase(jn, a["delta"], L, done=True).capitalize() + "."
            req = a.get("requested", a["delta"])
            if abs(req - a["delta"]) > 0.05:
                why = self.T(f" Because of the limit it turned {abs(a['delta']):.1f} instead of {abs(req):g} degrees.",
                             f" Sınır nedeniyle {abs(req):g} yerine {abs(a['delta']):.1f} derece dönebildi.")
                shown, spoken = shown + why, spoken + why
            return shown, spoken
        if kind == "joint_abs":
            target = [a["targets"].get(j, 0.0) for j in brain.ALL_JOINTS]
            label = a.get("label", "target")
            self._goal(self.joint_action, JointMove.Goal(relative=False, target_deg=target), label)
            if label == "manoeuvre":
                mv = a["manoeuvre"]
                w = {"UP": "yukarı", "DOWN": "aşağı", "LEFT": "sola", "RIGHT": "sağa", "FORWARD": "ileri", "BACK": "geri"}
                spoken = self.T(f"Done. I reached the point {dist(mv['mm'], 'en')} {mv['word'].lower()} by another path.",
                                f"Düz çizgide olmadı ama farklı bir yoldan {dist(mv['mm'], 'tr')} "
                                f"{w.get(mv['word'], mv['word'])} gittim hocam.")
                return spoken[:-1] + f": {self._pose_text()}.", spoken
            if label == "home":
                name_en, name_tr = "home", "başlangıç konumuna"
            elif label.startswith("point "):
                name_en, name_tr = label, f"{label[6:]} noktasına"
            else:
                name_en, name_tr = "the target", "hedefe"
            shown = self.T(f"Reached {name_en}: {self._pose_text()}.", f"{name_tr.capitalize()} ulaşıldı: {self._pose_text()}.")
            spoken = self.T(f"Reached {name_en}.", f"{name_tr.capitalize()} ulaşıldı.")
            if a.get("requested"):
                off = [j for j in brain.MOVE_JOINTS if abs(a["targets"][j] - a["requested"][j]) > 0.05]
                why = self.T(" As close as the limits allow (" + ", ".join(off) + " held at the limit).",
                             " Sınırların izin verdiği kadar yaklaşıldı (" + ", ".join(brain.jname(j) for j in off) + " sınırda kaldı).")
                shown, spoken = shown + why, spoken + why
            if a.get("gripper") in ("open", "closed"):
                self._call("gripper/open" if a["gripper"] == "open" else "gripper/close", Trigger.Request())
                time.sleep(0.6)
                extra = self.T(f" Gripper {a['gripper']}.",
                               " Tutucu açıldı." if a["gripper"] == "open" else " Tutucu kapatıldı.")
                shown, spoken = shown + extra, spoken + extra
            return shown, spoken
        if kind == "tcp":
            axis, remaining = a["axis"], a["mm"]
            before = self._pose_text()
            while abs(remaining) > 1e-6:
                chunk = max(-PLANNER_MAX_TCP_MM, min(PLANNER_MAX_TCP_MM, remaining))
                self._goal(self.tcp_action, TcpMove.Goal(axis=axis, distance_mm=float(chunk)),
                           f"tool {axis} {chunk:+g} mm")
                remaining -= chunk
            w = DIR_WORDS.get(axis)
            d_en = (w[0] if a["mm"] > 0 else w[1]) if w else axis
            d_tr = (w[2] if a["mm"] > 0 else w[3]) if w else axis
            mm, req = abs(a["mm"]), abs(a.get("requested", a["mm"]))
            if mm < req - 0.5:
                spoken = self.T(f"Because of the limit I moved {d_en} {dist(mm, 'en')} instead of {dist(req, 'en')}.",
                                f"Sınır nedeniyle {dist(req, 'tr')} yerine {dist(mm, 'tr')} {d_tr} gidildi.")
            else:
                spoken = self.T(f"Moved {d_en} {dist(mm, 'en')}.", f"{dist(mm, 'tr')} {d_tr} gidildi.")
            shown = spoken[:-1] + f": {before} → {self._pose_text()}."
            return shown, spoken
        if kind == "path":
            pieces = a["pieces"]
            for i, d in enumerate(pieces, 1):
                if self.abort.is_set():
                    raise Aborted("stopped")
                self._update(response=self.T(f"Drawing: piece {i}/{len(pieces)}", f"Çiziliyor: parça {i}/{len(pieces)}"))
                self._goal(self.tcp_action, TcpMove.Goal(axis="V", vector_mm=[float(x) for x in d]),
                           f"shape piece {i}")
            label = brain.shape_label(a["shape"], a["dims"], a["plane"], L)
            spoken = self.T(f"Drew a {label}.", f"{label[0].upper() + label[1:]} çizildi.")
            if a.get("cause"):
                asked = brain.shape_label(a["shape"], a["requested"], a["plane"], L)
                spoken += self.T(f" Because of {a['cause']}, instead of a {asked}.",
                                 f" {a['cause'][0].upper() + a['cause'][1:]} nedeniyle; istenen: {asked}.")
            return spoken, spoken
        if kind == "save_point":
            self._call("points/save", SavePoint.Request(name=a["name"]))
            text = self.T(f"Saved this position as {a['name']}.", f"Bu konum {a['name']} olarak kaydedildi.")
            return text, text
        if kind == "define_skill":
            key = self.memory.define_skill(a["name"], a["steps"])
            return (self.T(f"Learned skill {key}: {' | '.join(a['steps'])}.", f"{key} becerisi öğrenildi: {' | '.join(a['steps'])}."),
                    self.T(f"I learned the skill {key}.", f"{key} becerisini öğrendim."))
        if kind == "forget_skill":
            key = self.memory.forget_skill(a["name"])
            text = self.T(f"Forgot skill {key}.", f"{key} becerisi unutuldu.")
            return text, text
        raise RuntimeError(f"unknown step {kind}")

    def _read(self, what):
        r = self.robot
        if what == "LIST_POINTS":
            names = ", ".join(self.points) or self.T("none", "yok")
            return self.T(f"Saved points: {names}.", f"Kayıtlı noktalar: {names}.")
        if what == "LIST_SKILLS":
            skills = ", ".join(self.memory.skills) or self.T("none", "yok")
            return self.T(f"Skills: {skills}.", f"Beceriler: {skills}.")
        if r is None or not r.connected:
            return self.T("The robot is not connected.", "Robot bağlı değil.")
        if what == "LINE_STATUS":
            if r.line_inputs < 0 or r.line_outputs < 0:
                return self.T("The transfer line signals have not been read yet.", "Transfer hattı sinyalleri henüz okunmadı hocam.")
            from .serial_link import OUT_BUSY, OUT_STATION_READY, line_bit, line_error_code, line_order
            order = line_order(r.line_inputs)
            order_tr = {"none": "sipariş yok", "assemble": "montaj siparişi var", "sort_out": "ayıklama siparişi var",
                        "take_from_pallet": "paletten parça alma siparişi var"}[order]
            order_en = {"none": "no order", "assemble": "an assembly order", "sort_out": "a sorting-out order",
                        "take_from_pallet": "a take-from-pallet order"}[order]
            ready, busy, code = line_bit(r.line_outputs, OUT_STATION_READY), line_bit(r.line_outputs, OUT_BUSY), line_error_code(r.line_outputs)
            return self.T(f"Station 4: {order_en}. We report ready {'on' if ready else 'off'}, busy {'on' if busy else 'off'}, error code {code}.",
                          f"4. istasyon: {order_tr}. Bizim sinyaller: hazır {'açık' if ready else 'kapalı'}, "
                          f"meşgul {'açık' if busy else 'kapalı'}, hata kodu {code}.")
        if what == "WHERE":
            joints = ", ".join(f"{j} {v:.0f}" for j, v in zip(brain.ALL_JOINTS, r.joints_deg) if not math.isnan(v))
            return self.T(f"Joints {joints} degrees; {self._pose_text()}.",
                          f"Eklemler {joints} derece; {self._pose_text()}.")
        return self.T(
            f"Connected, servo {'on' if r.servo_on else 'off'}, {'ALARM' if r.alarm else 'no alarm'}, "
            f"gripper {r.gripper}, speed {r.override_percent} percent.",
            f"Bağlı, servo {'açık' if r.servo_on else 'kapalı'}, {'ALARM VAR' if r.alarm else 'alarm yok'}, "
            f"tutucu {dict(open='açık', closed='kapalı').get(r.gripper, 'bilinmiyor')}, hız yüzde {r.override_percent}.")


def main():
    rclpy.init()
    node = AINode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
