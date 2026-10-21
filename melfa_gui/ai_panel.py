"""
"AI Control" panel: push-to-talk / text -> (stop-word check) -> local LLM ->
validator -> confirmation -> serial worker, with spoken feedback.

Safety rules implemented here:
  - The raw transcript (or typed text) is checked for stop words FIRST. A
    match fires STOP through the GUI's own STOP handler and the text never
    reaches the language model.
  - Only plans whose every step is READ or SAFE (STOP, SRVOFF) run without
    a click. Anything that changes the robot (outputs, gripper, override,
    control rights, RUN, motion) is shown and waits for Confirm.
  - A plan that fails validation is shown and spoken with the reason and is
    never retried or modified.
  - Confirm re-validates against the current joint angles and refuses plans
    older than PLAN_TTL_S.
  - Text starting with ">" is a literal command list (comma separated) that
    skips the LLM but goes through the same validator and confirmation.
"""

import threading
import time

from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import (
    QCheckBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QPushButton,
)

import ai_speech
import ai_validator as v
from ai_llm import LLMError, OllamaClient
from ai_log import log_event, log_plan
from widgets import Card, StatusPill

PLAN_TTL_S = 120
SILENT_PEAK = 1e-4  # a muted microphone records exact zeros
KIND_COLORS = {
    v.Kind.READ: "#3a3a3c",
    v.Kind.SAFE: "#1f9d45",
    v.Kind.ACTUATE: "#c46a00",
    v.Kind.PROGRAM: "#d70015",
    v.Kind.MOTION: "#d70015",
}
REJECTED_COLOR = "#d70015"


class AIControlPanel(Card):
    progress = pyqtSignal(str)          # from the serial worker during execution
    _planned = pyqtSignal(int, object)  # generation, (raw, parsed) or LLMError
    _transcribed = pyqtSignal(str, str)  # text, error
    _llm_status = pyqtSignal(bool, str)

    def __init__(self, get_context, emergency_stop, execute, parent=None):
        """get_context() -> dict(connected, servo_on, alarm, joints, override,
        line, busy); emergency_stop(text) sends STOP; execute(plan, abort)
        queues the plan on the serial worker and returns True if queued."""
        super().__init__("AI Control", parent)
        self._get_context = get_context
        self._emergency_stop = emergency_stop
        self._execute = execute
        self._llm = OllamaClient()
        self._llm_ok = False
        self._recorder = ai_speech.Recorder() if ai_speech.recording_available() else None
        self._transcriber = ai_speech.Transcriber()
        self._speaker = ai_speech.Speaker()
        self._recording = False
        self._generation = 0          # bumps on every new request; stale LLM replies are dropped
        self._thinking = False
        self._plan = None             # plan awaiting confirmation
        self._plan_meta = None        # {"commands", "say", "source", "time"}
        self._executing = False
        self._abort = threading.Event()
        self._robot = {"connected": False, "busy": False, "servo_on": False}

        self._build()
        self.progress.connect(self._on_progress)
        self._planned.connect(self._on_planned)
        self._transcribed.connect(self._on_transcribed)
        self._llm_status.connect(self._on_llm_status)
        self.refresh_llm_status()
        self._render_buttons()

    # -- layout ------------------------------------------------------------

    def _build(self):
        top = QHBoxLayout()
        top.setSpacing(8)
        self.llm_pill = StatusPill("LLM …", "idle")
        self.llm_pill.setToolTip(f"Local model via Ollama: {self._llm.model}")
        top.addWidget(self.llm_pill)
        top.addStretch(1)
        self.speak_box = QCheckBox("Speak")
        self.speak_box.setChecked(self._speaker.engine is not None)
        self.speak_box.setEnabled(self._speaker.engine is not None)
        self.speak_box.setToolTip(f"Text to speech: {self._speaker.engine or 'none available'}")
        self.speak_box.toggled.connect(lambda on: setattr(self._speaker, "enabled", on))
        top.addWidget(self.speak_box)
        self.body.addLayout(top)

        self.ptt_btn = QPushButton("🎙  Hold to talk")
        self.ptt_btn.setObjectName("pttBtn")
        self.ptt_btn.setCursor(Qt.PointingHandCursor)
        self.ptt_btn.pressed.connect(self._on_ptt_pressed)
        self.ptt_btn.released.connect(self._on_ptt_released)
        if self._recorder is None:
            self.ptt_btn.setToolTip("Install sounddevice to record")
        else:
            self.ptt_btn.setToolTip(f"Speech to text: faster-whisper {ai_speech.WHISPER_MODEL} on CPU")
        self.body.addWidget(self.ptt_btn)

        row = QHBoxLayout()
        row.setSpacing(8)
        self.text_input = QLineEdit()
        self.text_input.setPlaceholderText("Type a request, or >CMD, CMD …")
        self.text_input.returnPressed.connect(self._on_text_submitted)
        self.send_btn = QPushButton("Send")
        self.send_btn.clicked.connect(self._on_text_submitted)
        row.addWidget(self.text_input, 1)
        row.addWidget(self.send_btn)
        self.body.addLayout(row)

        self.transcript = QLabel("Say or type what the robot should do. “stop” always stops.")
        self.transcript.setObjectName("aiTranscript")
        self.transcript.setWordWrap(True)
        self.body.addWidget(self.transcript)

        plan_head = QLabel("Proposed commands")
        plan_head.setObjectName("dim")
        self.body.addWidget(plan_head)
        self.plan_list = QListWidget()
        self.plan_list.setObjectName("aiPlan")
        self.plan_list.setWordWrap(True)
        self.plan_list.setMinimumHeight(90)
        self.body.addWidget(self.plan_list, 1)

        buttons = QHBoxLayout()
        buttons.setSpacing(8)
        self.confirm_btn = QPushButton("Confirm")
        self.confirm_btn.setObjectName("primary")
        self.confirm_btn.clicked.connect(self._on_confirm)
        self.cancel_btn = QPushButton("Cancel")
        self.cancel_btn.clicked.connect(lambda: self._clear_plan("cancelled", log=True))
        buttons.addWidget(self.confirm_btn, 1)
        buttons.addWidget(self.cancel_btn, 1)
        self.body.addLayout(buttons)

        self.response = QLabel("")
        self.response.setObjectName("aiResponse")
        self.response.setWordWrap(True)
        self.body.addWidget(self.response)

        motion = "enabled — every move needs Confirm" if v.MOVE_SYNTAX_VERIFIED else \
            "disabled (MOVE_SYNTAX_VERIFIED = False)"
        self.motion_note = QLabel(f"Motion: {motion}.  Servo ON is never available here.")
        self.motion_note.setObjectName("note")
        self.motion_note.setWordWrap(True)
        self.body.addWidget(self.motion_note)

    # -- state from the main window ----------------------------------------

    def set_robot_state(self, connected, busy, servo_on):
        self._robot = {"connected": connected, "busy": busy, "servo_on": servo_on}
        if not connected and self._plan is not None:
            self._clear_plan("disconnected")
        self._render_buttons()

    def abort(self):
        """Called by the GUI's STOP handler: ends any running plan at the next
        step and drops a plan that is waiting for confirmation."""
        self._abort.set()
        if self._plan is not None:
            self._clear_plan("STOP pressed")

    def on_executed(self, result):
        self._executing = False
        lines = [f"{cmd} → {reply or 'ok'}" for cmd, reply, _ in result.lines]
        log_event("EXEC_RESULT", ok=result.ok, error=result.error, lines=result.lines)
        if result.ok:
            spoken = " ".join(result.spoken) or "Done."
            self._show_response(spoken, "ok")
        else:
            spoken = f"Stopped: {result.error.rstrip('.')}."
            self._show_response(spoken, "error")
        self._speaker.say(spoken)
        if lines:
            self.plan_list.addItem(self._item("result: " + "; ".join(lines), "#0062cc"))
        self._render_buttons()

    def on_execution_failed(self, message):
        self._executing = False
        log_event("EXEC_LINK_ERROR", message=message)
        self._show_response(f"Link error: {message}. Motion disabled; reconnect.", "error")
        self._speaker.say("Link error. The robot was stopped and the servo turned off.")
        self._render_buttons()

    def refresh_llm_status(self):
        def check():
            ok, message = self._llm.check()
            self._llm_status.emit(ok, message)
            if ok:
                self._llm.warm_up()  # load the model into VRAM before the first request
        threading.Thread(target=check, daemon=True).start()

    # -- input: voice ------------------------------------------------------

    def _on_ptt_pressed(self):
        if self._recorder is None or self._recording:
            return
        try:
            self._recorder.start()
        except Exception as e:  # device busy, no microphone ...
            self._show_response(f"Microphone error: {e}", "error")
            return
        self._recording = True
        self.ptt_btn.setText("●  Recording… release to send")
        self.ptt_btn.setProperty("active", True)
        self._repolish(self.ptt_btn)

    def _on_ptt_released(self):
        if not self._recording:
            return
        self._recording = False
        audio = self._recorder.stop()
        self.ptt_btn.setText("…  Transcribing")
        self.ptt_btn.setProperty("active", False)
        self._repolish(self.ptt_btn)

        if len(audio) and float(abs(audio).max()) < SILENT_PEAK:
            self.ptt_btn.setText("🎙  Hold to talk")
            log_event("STT_ERROR", error="silent recording", peak=float(abs(audio).max()))
            self._show_response("The microphone recorded silence — is it muted? "
                                "(wpctl set-mute @DEFAULT_AUDIO_SOURCE@ 0)", "error")
            return

        def transcribe():
            try:
                self._transcribed.emit(self._transcriber.transcribe(audio), "")
            except Exception as e:
                self._transcribed.emit("", str(e))
        threading.Thread(target=transcribe, daemon=True).start()

    def _on_transcribed(self, text, error):
        self.ptt_btn.setText("🎙  Hold to talk")
        if error:
            log_event("STT_ERROR", error=error)
            self._show_response(f"Speech recognition failed: {error}", "error")
            return
        log_event("TRANSCRIPT", text=text)
        if not text:
            self._show_response("Didn't catch that.", "warn")
            return
        self._handle_request(text, source="voice")

    # -- input: text -------------------------------------------------------

    def _on_text_submitted(self):
        text = self.text_input.text().strip()
        if not text:
            return
        self.text_input.clear()
        log_event("TEXT_INPUT", text=text)
        self._handle_request(text, source="text")

    # -- request handling --------------------------------------------------

    def _handle_request(self, text, source):
        self.plan_list.clear()  # never leave an old plan on screen next to a new request
        # 1. Emergency stop on the RAW text, before anything else.
        if v.is_emergency_stop(text):
            self._abort.set()
            self._generation += 1  # drop any reply still coming from the model
            self._thinking = False
            log_event("EMERGENCY_STOP", source=source, text=text)
            self._emergency_stop(text)
            self._clear_plan("emergency stop")
            self.transcript.setText(f"You: “{text}”")
            self._show_response("STOP sent.", "error")
            self._speaker.say("Stopped.")
            self._render_buttons()
            return

        self.transcript.setText(f"You: “{text}”")
        self._clear_plan(None)
        self._generation += 1
        generation = self._generation

        # 2. Literal commands skip the model but not the validator.
        if text.startswith(">"):
            commands = [c.strip() for c in text[1:].split(",") if c.strip()]
            self._on_planned(generation, ("(direct)", {"say": "", "commands": commands, "direct": True}))
            return

        # 3. Natural language -> local model (background thread).
        if not self._llm_ok:
            self.refresh_llm_status()
        context = self._get_context()
        self._thinking = True
        self._show_response("Thinking…", "info")
        self._render_buttons()

        def ask():
            try:
                self._planned.emit(generation, self._llm.plan(text, context))
            except LLMError as e:
                self._planned.emit(generation, e)
        threading.Thread(target=ask, daemon=True).start()

    def _on_planned(self, generation, outcome):
        if generation != self._generation:
            return  # superseded by a newer request or a stop
        self._thinking = False
        if isinstance(outcome, LLMError):
            log_event("LLM_ERROR", error=str(outcome))
            self._show_response(f"AI unavailable: {outcome}", "error")
            self._speaker.say("The language model is not available.")
            self._render_buttons()
            return

        raw, parsed = outcome
        direct = parsed.get("direct", False)
        source = "direct" if direct else "llm"
        if not direct:
            log_event("LLM_OUTPUT", model=self._llm.model, raw=raw)
        commands, say = parsed["commands"], parsed.get("say", "")

        if not commands:
            log_event("VALIDATOR", source=source, ok=False, reason="no commands", say=say)
            self._show_response(say or "Nothing to do.", "info")
            self._speaker.say(say)
            self._render_buttons()
            return

        context = self._get_context()
        plan = v.validate(commands, context.get("joints"))
        log_plan(plan, source)
        self._show_plan(plan)

        if not plan.ok:
            message = f"I can't do that: {plan.reason.rstrip('.')}."
            self._show_response(message, "error")
            self._speaker.say(message)
            self._render_buttons()
            return

        if not plan.needs_confirmation:
            self._show_response(say or "Running.", "info")
            self._start(plan, source, confirmed=False)
            return

        self._plan = plan
        self._plan_meta = {"commands": commands, "say": say, "source": source, "time": time.monotonic()}
        effects = v.describe_effects(plan, context.get("outputs"))
        if effects:
            self.plan_list.addItem(self._item("→ after this plan: " + ", ".join(effects), "#0062cc"))
        log_event("EFFECTS", effects=effects)
        prompt = ("This will set " + ", ".join(effects) + ". " if effects else (say + " " if say else "")) \
            + "Please confirm."
        self._show_response(prompt, "warn")
        self._speaker.say(prompt)
        self._render_buttons()

    def _on_confirm(self):
        plan, meta = self._plan, self._plan_meta
        if plan is None:
            return
        if time.monotonic() - meta["time"] > PLAN_TTL_S:
            self._clear_plan("expired")
            self._show_response("That plan expired. Ask again.", "warn")
            return
        # Re-validate against the robot as it is now (joints may have moved).
        context = self._get_context()
        fresh = v.validate(meta["commands"], context.get("joints"))
        log_plan(fresh, meta["source"] + "/confirm")
        if not fresh.ok:
            self._clear_plan(None)
            self._show_plan(fresh)
            message = f"No longer valid: {fresh.reason.rstrip('.')}."
            self._show_response(message, "error")
            self._speaker.say(message)
            return
        if any(s.kind == v.Kind.MOTION for s in fresh.steps) and not context.get("servo_on"):
            self._show_response("Servo is off. Turn it on with the Servo ON button first.", "warn")
            return
        self._plan, self._plan_meta = None, None
        self._start(fresh, meta["source"], confirmed=True)

    def _start(self, plan, source, confirmed):
        ctx = self._get_context()
        if not ctx.get("connected"):
            self._show_response("Not connected.", "warn")
            self._render_buttons()
            return
        if ctx.get("busy"):
            self._show_response("The robot is busy. Try again in a moment.", "warn")
            self._render_buttons()
            return
        self._abort.clear()
        log_event("EXECUTE", source=source, confirmed=confirmed,
                  commands=[s.command for s in plan.steps])
        if self._execute(plan, self._abort):
            self._executing = True
            self._show_response("Executing…", "info")
        self._render_buttons()

    # -- display helpers -----------------------------------------------------

    def _item(self, text, color):
        item = QListWidgetItem(text)
        item.setForeground(QColor(color))
        return item

    def _show_plan(self, plan):
        self.plan_list.clear()
        for s in plan.steps:
            if s.ok:
                tag = s.kind.value.upper()
                self.plan_list.addItem(self._item(f"[{tag}] {s.command} — {s.reason}", KIND_COLORS[s.kind]))
            else:
                shown = s.command or str(s.source)
                self.plan_list.addItem(self._item(f"✗ {shown} — {s.reason}", REJECTED_COLOR))

    def _clear_plan(self, reason, log=False):
        if self._plan is not None and reason:
            log_event("PLAN_DROPPED", reason=reason,
                      commands=[s.command for s in self._plan.steps])
            if reason != "STOP pressed" and log:
                self._show_response("Cancelled.", "info")
        self._plan, self._plan_meta = None, None
        self._render_buttons()

    def _show_response(self, text, kind):
        self.response.setText(text)
        self.response.setProperty("kind", kind)
        self._repolish(self.response)

    def _on_progress(self, text):
        self.response.setText(text)

    def _on_llm_status(self, ok, message):
        self._llm_ok = ok
        self.llm_pill.set_state(f"LLM {'READY' if ok else 'OFFLINE'}", "ok" if ok else "warn")
        self.llm_pill.setToolTip(message)

    def _render_buttons(self):
        robot = self._robot
        waiting = self._plan is not None
        self.confirm_btn.setEnabled(waiting and robot["connected"] and not robot["busy"]
                                    and not self._executing)
        self.cancel_btn.setEnabled(waiting)
        self.ptt_btn.setEnabled(self._recorder is not None)
        self.send_btn.setEnabled(True)

    @staticmethod
    def _repolish(w):
        w.style().unpolish(w)
        w.style().polish(w)
