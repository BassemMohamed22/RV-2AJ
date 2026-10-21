"""
GUI control panel for a Mitsubishi RV-2AJ / CR1-571 robot over serial.

Safety invariants enforced in this file:
  - Nothing is sent automatically on connect except the read-only poll
    loop (STATE / PPOSF / JPOSF, and VALM_INB / VALM_OUTB for the transfer
    line). CNTLON/SRVON/moves/RUN/output writes are only ever sent from
    explicit button handlers.
  - Transfer-line outputs (station ready, busy, error code) change only when
    the user clicks them, and the panel shows the controller's read-back,
    never the requested state.
  - All serial I/O runs on the worker thread (commands are queued to it),
    except STOP, which is sent directly so it is never stuck behind a queue.
    Esc also triggers STOP.
  - First SRVON each session requires a confirmation dialog; first J3 step
    each session requires another (J3's origin is miscalibrated).
  - Step buttons are enabled only while connected, servo is on, and no
    step is in progress. One tap = one small relative joint move.
  - On any link error or rejected command, polling stops, the port is
    closed, and motion controls are disabled until the user reconnects.
    No auto-reconnect.
  - The speed override resets to 10% on launch and is sent to the
    controller with SERVO ON and with every step.
  - Disconnect and window close send STOP, SRVOFF and CNTLOFF before closing.
  - TCP moves (buttons or voice) use the same gating as joint steps, are
    capped at MAX_TCP_MOVE_MM, and are read back with VAL before EXECMOV.
  - Voice needs an explicit toggle (confirmed once per session), requires
    the word "move"/"go" by default, ignores commands while busy (except
    "stop"), and turns off on disconnect or link error.
  - AI Control (ai_panel.py): stop words are matched on the raw transcript
    before the language model and fire this window's STOP handler. Every
    model command passes ai_validator.py; only read/stop plans run without
    a click, everything else waits for Confirm. The AI layer can never send
    SRVON, and motion stays rejected while MOVE_SYNTAX_VERIFIED is False.
    AI plans run on the serial worker; STOP aborts them between steps.
"""

import json
import os
import queue
import sys
import time
from datetime import datetime

from PyQt5.QtCore import QObject, Qt, QThread, pyqtSignal
from PyQt5.QtGui import QColor, QKeySequence
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QShortcut,
    QSizePolicy,
    QSlider,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

import serial_link
from serial_link import (
    ERROR_CODE_MAX,
    IN_ORDER_BIT0,
    IN_ORDER_BIT1,
    OUT_BUSY,
    OUT_ERR_CODE_BIT0,
    OUT_ERR_CODE_BIT1,
    OUT_STATION_READY,
    STEPPABLE_JOINTS,
    Order,
    SerialLink,
    SerialLinkError,
    bit,
    decode_order,
    error_code,
    is_ok_reply,
    order_bits,
    parse_jposf,
    parse_pposf,
)
import ai_executor
import voice
from ai_panel import AIControlPanel
from voice_commands import DIRECTIONS, parse_command
from widgets import THEME_QSS, Card, GlassBackground, JointCard, Segmented, StatCell, StatusPill

POLL_INTERVAL_SECONDS = 1.0
MOVING_POLL_INTERVAL_SECONDS = 0.3
DEFAULT_OVERRIDE_PERCENT = 10
STEP_SIZES_DEG = (0.5, 1.0, 2.0, 5.0)
DEFAULT_STEP_INDEX = 1
SETTLE_TOLERANCE_DEG = 0.05
TCP_STEPS_MM = (1.0, 5.0, 10.0, 20.0)
DEFAULT_TCP_STEP_INDEX = 1
MAX_TCP_MOVE_MM = 50.0
TCP_SETTLE_TOLERANCE_MM = 0.2
TCP_BUTTONS = (  # label, direction word from voice_commands.DIRECTIONS
    ("↑  Up", "up"), ("←  Left", "left"), ("Forward", "forward"),
    ("↓  Down", "down"), ("Right  →", "right"), ("Back", "back"),
)
STABLE_POLLS_TO_SETTLE = 3
SERVO_ON_SETTLE_SECONDS = 1.5
JOINT_NAMES = ("J1", "J2", "J3", "J4", "J5", "J6")
MISCALIBRATED_JOINTS = {"J3"}
LOG_LIMIT = 200
AI_PANEL_WIDTH = 380
WIDE_WINDOW_WIDTH = 1700  # screens this wide open with the AI column showing

LOG_COLORS = {"info": "#3a3a3c", "ok": "#1f9d45", "warn": "#c46a00", "error": "#d70015"}
POINTS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "points.json")
MOVABLE_JOINTS = tuple(n for n in JOINT_NAMES if int(n[1]) in STEPPABLE_JOINTS)


def load_points():
    try:
        with open(POINTS_FILE) as f:
            data = json.load(f)
        return [{"name": str(p["name"]), "joints": list(p["joints"])[:6]}
                for p in data if len(p.get("joints", [])) >= 6]
    except (OSError, ValueError, KeyError, TypeError):
        return []


def save_points(points):
    tmp = POINTS_FILE + ".tmp"
    with open(tmp, "w") as f:
        json.dump(points, f, indent=2)
    os.replace(tmp, POINTS_FILE)


# ---------------------------------------------------------------------
# Background worker: runs queued commands and the poll loop, off the UI
# ---------------------------------------------------------------------

class PollWorker(QObject):
    state_received = pyqtSignal(str)
    pposf_received = pyqtSignal(dict)
    jposf_received = pyqtSignal(dict)
    line_io_received = pyqtSignal(int, int)  # robot inputs, outputs (bits 0-7)
    command_done = pyqtSignal(str, object)
    link_error = pyqtSignal(str)

    def __init__(self, link):
        super().__init__()
        self._link = link
        self._running = False
        self._commands = queue.Queue()
        self.fast_poll = False

    def submit(self, name, fn):
        self._commands.put((name, fn))

    def start_polling(self):
        self._running = True
        while self._running:
            if not self._run_pending_commands():
                return
            self._poll_once()
            interval = MOVING_POLL_INTERVAL_SECONDS if self.fast_poll else POLL_INTERVAL_SECONDS
            self._sleep_interruptible(interval)

    def stop(self):
        self._running = False

    def _sleep_interruptible(self, seconds):
        end = time.monotonic() + seconds
        while self._running and time.monotonic() < end and self._commands.empty():
            time.sleep(0.05)

    def _run_pending_commands(self):
        while self._running:
            try:
                name, fn = self._commands.get_nowait()
            except queue.Empty:
                return True
            try:
                result = fn()
            except SerialLinkError as e:
                self._fail(f"{name}: {e}")
                return False
            self.command_done.emit(name, result)
        return False

    def _fail(self, message):
        self._running = False
        self.link_error.emit(message)

    def _poll_once(self):
        try:
            state = self._link.query_state()
            if not is_ok_reply(state):
                self._fail(f"STATE returned a non-OK reply: {state.strip()}")
                return
            self.state_received.emit(state)
            pposf = self._link.query_pposf()
            if is_ok_reply(pposf):
                self.pposf_received.emit(parse_pposf(pposf))
            jposf = self._link.query_jposf()
            if is_ok_reply(jposf):
                self.jposf_received.emit(parse_jposf(jposf))
            if not self.fast_poll:  # keep move tracking responsive
                inputs = self._link.read_inputs()
                outputs = self._link.read_outputs()
                self.line_io_received.emit(inputs, outputs)
        except SerialLinkError as e:
            self._fail(str(e))


# ---------------------------------------------------------------------
# Main window
# ---------------------------------------------------------------------

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("RV-2AJ Controller")
        self.resize(1280, 840)
        self.setMinimumSize(1100, 740)
        screen = QApplication.primaryScreen()
        self._screen_width = screen.availableGeometry().width() if screen else 1920

        self.link = SerialLink()
        self.thread = None
        self.worker = None
        self._srvon_confirmed_this_session = False
        self._j3_confirmed_this_session = False
        self._goto_confirmed_this_session = False
        self._connected = False
        self._servo_on = False
        self._busy = False
        # While a move runs: {"label", "targets": {joint: deg} or None until
        # the controller accepted it, "last": {joint: deg}, "stable": int}
        self._move = None
        self._loop = None  # {"index", "cycle"} while the point loop runs
        self._alarm = False
        self._joints = {}
        self._tool = {}
        self._line_io = None  # (inputs, outputs) from the last poll
        self._ai_executing = False
        self._points = load_points()
        self._voice_on = False
        self._voice_confirmed_this_session = False
        self.voice_listener = voice.VoiceListener() if voice.available() else None
        if self.voice_listener is not None:
            self.voice_listener.heard.connect(self._on_voice_heard)
            self.voice_listener.status.connect(self._on_voice_status)

        root = GlassBackground()
        self.setCentralWidget(root)
        outer = QVBoxLayout(root)
        outer.setContentsMargins(20, 16, 20, 20)
        outer.setSpacing(16)

        outer.addWidget(self._build_header())

        body = QHBoxLayout()
        body.setSpacing(16)
        outer.addLayout(body, 1)

        left = QVBoxLayout()
        left.setSpacing(16)
        left.addWidget(self._build_joints_card(), 1)
        bottom = QHBoxLayout()
        bottom.setSpacing(16)
        bottom.addWidget(self._build_tcp_card(), 1)
        side = QVBoxLayout()
        side.setSpacing(16)
        line_card = self._build_line_card()
        line_card.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Minimum)  # never squash it
        side.addWidget(line_card)
        side.addWidget(self._build_program_card(), 1)
        side_widget = QWidget()
        side_widget.setFixedWidth(300)
        side_widget.setLayout(side)
        side.setContentsMargins(0, 0, 0, 0)
        bottom.addWidget(side_widget)
        left.addLayout(bottom)
        body.addLayout(left, 1)

        right_widget = QWidget()
        right_widget.setFixedWidth(380)
        right = QVBoxLayout(right_widget)
        right.setContentsMargins(0, 0, 0, 0)
        right.setSpacing(16)
        right.addWidget(self._build_motion_card())
        right.addWidget(self._build_points_card(), 3)
        right.addWidget(self._build_log_card(), 2)
        body.addWidget(right_widget)

        self.ai_panel = AIControlPanel(get_context=self._ai_context,
                                       emergency_stop=self._ai_emergency_stop,
                                       execute=self._ai_execute)
        self.ai_panel.setFixedWidth(AI_PANEL_WIDTH)
        body.addWidget(self.ai_panel)
        self._fit_window_for_ai(self.ai_toggle_btn.isChecked())

        QShortcut(QKeySequence(Qt.Key_Escape), self, activated=self._on_stop_clicked,
                  context=Qt.ApplicationShortcut)

        self._refresh_controls()
        self._refresh_pills()
        self._log("Ready. Select a port and connect.")

    # -- layout --------------------------------------------------------

    def _build_header(self):
        header = QWidget()
        layout = QHBoxLayout(header)
        layout.setContentsMargins(4, 0, 0, 0)
        layout.setSpacing(12)

        titles = QVBoxLayout()
        titles.setSpacing(0)
        title = QLabel("RV-2AJ Controller")
        title.setObjectName("appTitle")
        subtitle = QLabel("Mitsubishi CR1-571  ·  RS-232C 9600 8E2")
        subtitle.setObjectName("appSubtitle")
        titles.addWidget(title)
        titles.addWidget(subtitle)
        layout.addLayout(titles)
        layout.addSpacing(16)

        self.link_pill = StatusPill()
        self.servo_pill = StatusPill()
        self.alarm_pill = StatusPill()
        for pill in (self.link_pill, self.servo_pill, self.alarm_pill):
            layout.addWidget(pill)
        layout.addStretch(1)

        self.port_combo = QComboBox()
        self.port_combo.setEditable(True)
        self.port_combo.setMinimumWidth(170)
        self._refresh_ports()
        self.refresh_btn = QPushButton("↻")
        self.refresh_btn.setObjectName("iconBtn")
        self.refresh_btn.setToolTip("Refresh serial ports")
        self.refresh_btn.clicked.connect(self._refresh_ports)
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.setObjectName("primary")
        self.connect_btn.setMinimumWidth(110)
        self.connect_btn.clicked.connect(self._on_connect_toggled)
        self.ai_toggle_btn = QPushButton("✦  AI")
        self.ai_toggle_btn.setObjectName("voiceBtn")
        self.ai_toggle_btn.setCheckable(True)
        self.ai_toggle_btn.setToolTip("Show or hide the AI Control panel")
        # Open by default when the screen is wide enough for a third column.
        self.ai_toggle_btn.setChecked(self._screen_width >= WIDE_WINDOW_WIDTH)
        self.ai_toggle_btn.toggled.connect(self._fit_window_for_ai)
        layout.addWidget(self.ai_toggle_btn)
        layout.addSpacing(12)
        layout.addWidget(self.port_combo)
        layout.addWidget(self.refresh_btn)
        layout.addWidget(self.connect_btn)
        layout.addSpacing(12)

        self.stop_btn = QPushButton("STOP")
        self.stop_btn.setObjectName("estop")
        self.stop_btn.setToolTip("Stop motion (Esc)")
        self.stop_btn.setCursor(Qt.PointingHandCursor)
        self.stop_btn.clicked.connect(self._on_stop_clicked)
        layout.addWidget(self.stop_btn)
        return header

    def _build_joints_card(self):
        card = Card("Joints")
        grid = QGridLayout()
        grid.setSpacing(12)
        self.joint_cards = {}
        for i, name in enumerate(JOINT_NAMES):
            jc = JointCard(
                name,
                absent=int(name[1]) not in STEPPABLE_JOINTS,
                warning="check origin" if name in MISCALIBRATED_JOINTS else None,
            )
            jc.step_clicked.connect(lambda sign, n=name: self._on_step_clicked(n, sign))
            self.joint_cards[name] = jc
            grid.addWidget(jc, i // 3, i % 3)
        grid.setRowStretch(0, 1)
        grid.setRowStretch(1, 1)
        card.body.addLayout(grid, 1)
        return card

    def _build_tcp_card(self):
        card = Card("TCP")
        row = QHBoxLayout()
        row.setSpacing(24)
        self.tool_cells = {}
        for key, unit in (("X", "mm"), ("Y", "mm"), ("Z", "mm"), ("A", "°"), ("B", "°")):
            cell = StatCell(key, unit)
            self.tool_cells[key] = cell
            row.addWidget(cell)
        row.addStretch(1)
        self.tcp_moving_label = QLabel()
        self.tcp_moving_label.setObjectName("jointHint")
        self.tcp_moving_label.setProperty("kind", "accent")
        row.addWidget(self.tcp_moving_label, alignment=Qt.AlignTop)
        card.body.addLayout(row)

        dist_row = QHBoxLayout()
        dist_label = QLabel("Distance per move (mm)")
        dist_label.setObjectName("dim")
        self.tcp_step = Segmented([(f"{s:g}", s) for s in TCP_STEPS_MM], default_index=DEFAULT_TCP_STEP_INDEX)
        self.tcp_step.setToolTip("Millimetres per button press, and for voice commands without a number")
        self.tcp_step.setFixedWidth(220)
        dist_row.addWidget(dist_label)
        dist_row.addStretch(1)
        dist_row.addWidget(self.tcp_step)
        card.body.addLayout(dist_row)

        pad = QGridLayout()
        pad.setSpacing(8)
        self.tcp_buttons = []
        for i, (label, direction) in enumerate(TCP_BUTTONS):
            axis, sign = DIRECTIONS[direction]
            btn = QPushButton(label)
            btn.setObjectName("stepBtn")
            if axis == "R":
                btn.setToolTip(f"{direction}: {'out along' if sign > 0 else 'back along'} the arm's reach")
            else:
                btn.setToolTip(f"{direction}: {'+' if sign > 0 else '−'}{axis} (base frame)")
            btn.setCursor(Qt.PointingHandCursor)
            btn.clicked.connect(
                lambda _=False, a=axis, s=sign, d=direction:
                self._on_tcp_move_requested(a, s * self.tcp_step.value(), d, "button")
            )
            self.tcp_buttons.append(btn)
            pad.addWidget(btn, i // 3, i % 3)
        card.body.addLayout(pad)

        voice_row = QHBoxLayout()
        voice_row.setSpacing(8)
        self.voice_btn = QPushButton("●  Voice off")
        self.voice_btn.setObjectName("voiceBtn")
        self.voice_btn.setCheckable(True)
        self.voice_btn.clicked.connect(self._on_voice_toggled)
        self.mic_combo = QComboBox()
        self.mic_combo.addItem("Default microphone", None)
        for index, name in enumerate(voice.microphone_names()):
            self.mic_combo.addItem(name, index)
        self.mic_combo.setMinimumWidth(150)
        self.require_keyword = QCheckBox("Require “move”")
        self.require_keyword.setChecked(True)
        self.require_keyword.setToolTip("Only act on phrases containing 'move' or 'go', "
                                        "so normal conversation can't move the robot")
        voice_row.addWidget(self.voice_btn)
        voice_row.addWidget(self.mic_combo, 1)
        voice_row.addWidget(self.require_keyword)
        card.body.addLayout(voice_row)

        self.voice_heard = QLabel("Say e.g. “move up 20”, “go left 10”, “move forward 2 cm”, “stop”.")
        self.voice_heard.setObjectName("dim")
        self.voice_heard.setWordWrap(True)
        card.body.addWidget(self.voice_heard)
        if self.voice_listener is None:
            self.voice_btn.setToolTip("Install SpeechRecognition and PyAudio to use voice")
        return card

    def _build_motion_card(self):
        card = Card("Motion")

        servo_row = QHBoxLayout()
        servo_row.setSpacing(8)
        self.servo_on_btn = QPushButton("Servo ON")
        self.servo_on_btn.setObjectName("servoOn")
        self.servo_on_btn.clicked.connect(self._on_servo_on_requested)
        self.servo_off_btn = QPushButton("Servo OFF")
        self.servo_off_btn.setObjectName("servoOff")
        self.servo_off_btn.clicked.connect(self._on_servo_off_requested)
        servo_row.addWidget(self.servo_on_btn)
        servo_row.addWidget(self.servo_off_btn)
        card.body.addLayout(servo_row)

        speed_head = QHBoxLayout()
        speed_label = QLabel("Speed override")
        speed_label.setObjectName("dim")
        self.speed_value = QLabel(f"{DEFAULT_OVERRIDE_PERCENT}%")
        self.speed_value.setObjectName("midValue")
        speed_head.addWidget(speed_label)
        speed_head.addStretch(1)
        speed_head.addWidget(self.speed_value)
        card.body.addLayout(speed_head)

        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(5, 100)
        self.speed_slider.setValue(DEFAULT_OVERRIDE_PERCENT)
        self.speed_slider.valueChanged.connect(lambda v: self.speed_value.setText(f"{v}%"))
        self.speed_slider.sliderReleased.connect(self._on_override_changed)
        card.body.addWidget(self.speed_slider)

        step_label = QLabel("Step size")
        step_label.setObjectName("dim")
        card.body.addWidget(step_label)
        self.step_size = Segmented(
            [(f"{s:g}°", s) for s in STEP_SIZES_DEG], default_index=DEFAULT_STEP_INDEX
        )
        self.step_size.changed.connect(self._apply_step_size)
        card.body.addWidget(self.step_size)
        self._apply_step_size(self.step_size.value())

        self.reset_alarm_btn = QPushButton("Reset alarm")
        self.reset_alarm_btn.clicked.connect(self._on_reset_alarm_requested)
        card.body.addWidget(self.reset_alarm_btn)
        return card

    def _build_program_card(self):
        card = Card("Program")
        row = QHBoxLayout()
        row.setSpacing(8)
        self.program_spin = QSpinBox()
        self.program_spin.setRange(1, 9999)
        self.program_spin.setPrefix("Prog  ")
        self.program_spin.setButtonSymbols(QSpinBox.NoButtons)
        self.run_btn = QPushButton("Run")
        self.run_btn.clicked.connect(lambda: self._on_run_requested(self.program_spin.value()))
        self.program_stop_btn = QPushButton("Stop")
        self.program_stop_btn.clicked.connect(self._on_stop_clicked)
        row.addWidget(self.program_spin, 1)
        row.addWidget(self.run_btn)
        row.addWidget(self.program_stop_btn)
        card.body.addLayout(row)

        note = QLabel("RUN has not been tested on this controller yet.")
        note.setObjectName("note")
        card.body.addWidget(note)

        self.state_raw = QLineEdit()
        self.state_raw.setObjectName("stateRaw")
        self.state_raw.setReadOnly(True)
        self.state_raw.setPlaceholderText("STATE reply")
        card.body.addWidget(self.state_raw)
        card.body.addStretch(1)
        return card

    def _build_line_card(self):
        card = Card("Transfer Line  ·  Station 4 (XPA4)")

        order_row = QHBoxLayout()
        self.order_value = QLabel("—")
        self.order_value.setObjectName("orderValue")
        self.order_value.setToolTip("Order requested by Station 4")
        self.order_bits_label = QLabel()
        self.order_bits_label.setObjectName("ioBits")
        self.order_bits_label.setToolTip(
            f"Raw order bits from Station 4: IN{IN_ORDER_BIT1} = Bit1, IN{IN_ORDER_BIT0} = Bit0")
        order_row.addWidget(self.order_value)
        order_row.addStretch(1)
        order_row.addWidget(self.order_bits_label)
        card.body.addLayout(order_row)

        asserts = QLabel("Robot asserts to the line:")
        asserts.setObjectName("dim")
        card.body.addWidget(asserts)

        toggles = QHBoxLayout()
        toggles.setSpacing(8)
        self.ready_btn = QPushButton()
        self.ready_btn.setObjectName("ioToggle")
        self.ready_btn.setToolTip(f"OUT{OUT_STATION_READY} → Station 4 IN3. Click to toggle.")
        self.ready_btn.clicked.connect(
            lambda: self._on_line_output_requested("ready", OUT_STATION_READY, "Station ready"))
        self.busy_btn = QPushButton()
        self.busy_btn.setObjectName("ioToggle")
        self.busy_btn.setToolTip(f"OUT{OUT_BUSY} → Station 4 IN2. Click to toggle.")
        self.busy_btn.clicked.connect(
            lambda: self._on_line_output_requested("busy", OUT_BUSY, "Busy"))
        toggles.addWidget(self.ready_btn, 1)
        toggles.addWidget(self.busy_btn, 1)
        card.body.addLayout(toggles)

        err_row = QHBoxLayout()
        err_label = QLabel("Error code")
        err_label.setObjectName("dim")
        self.error_code_sel = Segmented([(str(c), c) for c in range(ERROR_CODE_MAX + 1)])
        self.error_code_sel.setFixedWidth(150)
        self.error_code_sel.setToolTip(
            f"Error code the robot sends to Station 4 IN1/IN0 on "
            f"OUT{OUT_ERR_CODE_BIT1} (bit 1) / OUT{OUT_ERR_CODE_BIT0} (bit 0). Click to change.")
        self.error_code_sel.changed.connect(self._on_error_code_requested)
        err_row.addWidget(err_label)
        err_row.addStretch(1)
        err_row.addWidget(self.error_code_sel)
        card.body.addLayout(err_row)

        self._render_line_io()
        return card

    def _build_points_card(self):
        card = Card("Points & loop")

        edit_row = QHBoxLayout()
        edit_row.setSpacing(8)
        self.save_point_btn = QPushButton("＋ Save point")
        self.save_point_btn.setObjectName("primary")
        self.save_point_btn.setToolTip("Record the robot's current joint angles")
        self.save_point_btn.clicked.connect(self._on_save_point)
        self.goto_btn = QPushButton("Go to")
        self.goto_btn.setToolTip("Move to the selected point")
        self.goto_btn.clicked.connect(self._on_goto_selected)
        self.delete_point_btn = QPushButton("Delete")
        self.delete_point_btn.clicked.connect(self._on_delete_point)
        self.clear_points_btn = QPushButton("Clear")
        self.clear_points_btn.clicked.connect(self._on_clear_points)
        edit_row.addWidget(self.save_point_btn, 1)
        edit_row.addWidget(self.goto_btn)
        edit_row.addWidget(self.delete_point_btn)
        edit_row.addWidget(self.clear_points_btn)
        card.body.addLayout(edit_row)

        header = QLabel("      " + "".join(f"{j:>8}" for j in MOVABLE_JOINTS))
        header.setObjectName("pointsHeader")
        card.body.addWidget(header)
        self.points_list = QListWidget()
        self.points_list.setObjectName("points")
        self.points_list.setMinimumHeight(110)
        self.points_list.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.points_list.itemSelectionChanged.connect(self._refresh_controls)
        self.points_list.itemDoubleClicked.connect(lambda _item: self._on_goto_selected())
        card.body.addWidget(self.points_list, 1)

        loop_row = QHBoxLayout()
        loop_row.setSpacing(8)
        self.loop_start_btn = QPushButton("▶  Start loop")
        self.loop_start_btn.setObjectName("loopStart")
        self.loop_start_btn.clicked.connect(self._on_loop_start)
        self.loop_end_btn = QPushButton("■  End loop")
        self.loop_end_btn.setObjectName("loopEnd")
        self.loop_end_btn.clicked.connect(self._on_loop_end)
        loop_row.addWidget(self.loop_start_btn, 1)
        loop_row.addWidget(self.loop_end_btn, 1)
        card.body.addLayout(loop_row)

        self.loop_status = QLabel()
        self.loop_status.setObjectName("dim")
        card.body.addWidget(self.loop_status)

        self._render_points()
        return card

    def _build_log_card(self):
        card = Card("Event log")
        self.log_list = QListWidget()
        self.log_list.setObjectName("log")
        self.log_list.setWordWrap(True)
        card.body.addWidget(self.log_list, 1)
        return card

    # -- presentation helpers -----------------------------------------

    def _log(self, text, kind="info"):
        item = QListWidgetItem(f"{datetime.now():%H:%M:%S}  {text}")
        item.setForeground(QColor(LOG_COLORS.get(kind, LOG_COLORS["info"])))
        self.log_list.addItem(item)
        while self.log_list.count() > LOG_LIMIT:
            self.log_list.takeItem(0)
        self.log_list.scrollToBottom()

    def _refresh_pills(self, link_error=False):
        if link_error:
            self.link_pill.set_state("● LINK ERROR", "error")
        elif self._connected:
            self.link_pill.set_state("● CONNECTED", "ok")
        else:
            self.link_pill.set_state("● OFFLINE", "idle")

        if not self._connected:
            self.servo_pill.set_state("SERVO —", "idle")
        elif self._servo_on:
            self.servo_pill.set_state("SERVO ON", "warn")
        else:
            self.servo_pill.set_state("SERVO OFF", "idle")

        if self._connected and self._alarm:
            self.alarm_pill.set_state("ALARM", "error")
        elif self._loop is not None:
            self.alarm_pill.set_state(f"LOOP · CYCLE {self._loop['cycle']}", "info")
        else:
            self.alarm_pill.set_state("NO ALARM", "idle")

    def _refresh_controls(self):
        idle = self._connected and not self._busy
        looping = self._loop is not None
        moving = self._move is not None
        for w in (self.servo_on_btn, self.servo_off_btn, self.reset_alarm_btn, self.speed_slider):
            w.setEnabled(idle)
        for w in (self.run_btn, self.program_spin):
            w.setEnabled(idle and not looping)
        self.program_stop_btn.setEnabled(self._connected)

        ready_to_move = idle and self._servo_on and not moving and not looping
        for jc in self.joint_cards.values():
            jc.set_steps_enabled(ready_to_move)
        for btn in self.tcp_buttons:
            btn.setEnabled(ready_to_move)
        line_ready = idle and self._line_io is not None
        for w in (self.ready_btn, self.busy_btn, self.error_code_sel):
            w.setEnabled(line_ready)
        self.voice_btn.setEnabled(self.voice_listener is not None and self._connected)
        self.ai_panel.set_robot_state(self._connected, self._busy or moving or looping, self._servo_on)
        self.mic_combo.setEnabled(self.voice_listener is not None and not self._voice_on)

        have_pose = all(self._joints.get(j) is not None for j in MOVABLE_JOINTS)
        selected = self.points_list.currentRow() >= 0 and bool(self.points_list.selectedItems())
        self.save_point_btn.setEnabled(self._connected and have_pose and not moving and not looping)
        self.goto_btn.setEnabled(ready_to_move and selected)
        self.delete_point_btn.setEnabled(selected and not looping)
        self.clear_points_btn.setEnabled(bool(self._points) and not looping)
        self.loop_start_btn.setEnabled(ready_to_move and len(self._points) >= 2)
        self.loop_end_btn.setEnabled(looping)

        if looping:
            name = self._points[self._loop["index"]]["name"]
            self.loop_status.setText(f"Looping · cycle {self._loop['cycle']} · moving to {name}")
        elif len(self._points) < 2:
            self.loop_status.setText("Save at least 2 points to loop.")
        else:
            self.loop_status.setText(f"{len(self._points)} points ready.")

        self.port_combo.setEnabled(not self._connected)
        self.refresh_btn.setEnabled(not self._connected)
        self.connect_btn.setText("Disconnect" if self._connected else "Connect")
        self.connect_btn.setObjectName("" if self._connected else "primary")
        self.connect_btn.style().unpolish(self.connect_btn)
        self.connect_btn.style().polish(self.connect_btn)

    def _render_points(self):
        row = self.points_list.currentRow()
        self.points_list.clear()
        for p in self._points:
            angles = "".join(
                f"{p['joints'][i]:>8.1f}" for i, j in enumerate(JOINT_NAMES) if j in MOVABLE_JOINTS
            )
            self.points_list.addItem(f"{p['name']:<4}{angles}")
        if 0 <= row < self.points_list.count():
            self.points_list.setCurrentRow(row)

    def _persist_points(self):
        try:
            save_points(self._points)
        except OSError as e:
            self._log(f"Could not save points file: {e}", "error")

    def _apply_step_size(self, size):
        for jc in self.joint_cards.values():
            jc.set_step_size(size)

    def _refresh_ports(self):
        current = self.port_combo.currentText()
        self.port_combo.clear()
        ports = serial_link.list_available_ports()
        if serial_link.PORT_DEFAULT not in ports:
            ports.insert(0, serial_link.PORT_DEFAULT)
        self.port_combo.addItems(ports)
        self.port_combo.setCurrentText(current or serial_link.PORT_DEFAULT)

    def _clear_readouts(self):
        self._joints = {}
        for jc in self.joint_cards.values():
            jc.set_value(None)
            jc.set_moving(False)
        self._tool = {}
        for cell in self.tool_cells.values():
            cell.set_value(None)
        self.tcp_moving_label.setText("")
        self.state_raw.clear()
        self._line_io = None
        self._render_line_io()

    # -- connection lifecycle -----------------------------------------

    def _on_connect_toggled(self):
        if self._connected:
            self._on_disconnect_requested()
        else:
            port = self.port_combo.currentText().strip()
            if port:
                self._on_connect_requested(port)

    def _on_connect_requested(self, port):
        try:
            self.link.connect(port)
        except SerialLinkError as e:
            self._log(f"Connect failed: {e}", "error")
            self._refresh_pills(link_error=True)
            return

        self._connected = True
        self._servo_on = False
        self._busy = False
        self._move = None
        self._loop = None
        self._alarm = False
        self._srvon_confirmed_this_session = False
        self._j3_confirmed_this_session = False
        self._goto_confirmed_this_session = False
        self._log(f"Connected to {port}. Servo is off.", "ok")
        self._start_worker()
        self._refresh_controls()
        self._refresh_pills()

    def _shutdown_link(self, send_servo_off):
        self._loop = None
        self._set_voice(False)
        self._stop_worker()
        if send_servo_off and self.link.is_open:
            for command in ("STOP", "SRVOFF", "CNTLOFF"):
                try:
                    self.link.send(command)
                except SerialLinkError:
                    break
        self.link.disconnect()
        self._connected = False
        self._servo_on = False
        self._busy = False
        self._move = None
        for jc in self.joint_cards.values():
            jc.set_moving(False)
        self._line_io = None  # never show a stale order or output state
        self._render_line_io()
        self._refresh_controls()

    def _on_disconnect_requested(self):
        self._shutdown_link(send_servo_off=True)
        self._clear_readouts()
        self._refresh_pills()
        self._refresh_controls()
        self._log("Disconnected (servo off sent).")

    def _on_link_error(self, message):
        # No auto-reconnect: the user must press Connect again, so no stale
        # servo/move/loop state can resume silently. Still try to drop the servo.
        self._shutdown_link(send_servo_off=True)
        self._refresh_pills(link_error=True)
        self._log(f"{message}", "error")
        if self._ai_executing:
            self._ai_executing = False
            self.ai_panel.on_execution_failed(message)
        self._log("Motion disabled. Reconnect to continue.", "warn")

    def _start_worker(self):
        self.worker = PollWorker(self.link)
        self.thread = QThread(self)
        self.worker.moveToThread(self.thread)
        self.thread.started.connect(self.worker.start_polling)
        self.worker.state_received.connect(self._on_state_received)
        self.worker.pposf_received.connect(self._on_pposf_received)
        self.worker.jposf_received.connect(self._on_jposf_received)
        self.worker.line_io_received.connect(self._on_line_io_received)
        self.worker.command_done.connect(self._on_command_done)
        self.worker.link_error.connect(self._on_link_error)
        self.thread.start()

    def _stop_worker(self):
        if self.worker is not None:
            self.worker.stop()
        if self.thread is not None:
            self.thread.quit()
            self.thread.wait(5000)
        self.thread = None
        self.worker = None

    def _submit(self, name, fn):
        if self.worker is None:
            return
        self._busy = True
        self._refresh_controls()
        self.worker.submit(name, fn)

    # -- poll results --------------------------------------------------

    def _on_state_received(self, raw_state):
        self.state_raw.setText(raw_state.strip())
        alarm = raw_state[2:3].islower()
        if alarm != self._alarm:
            self._alarm = alarm
            self._refresh_pills()
            if alarm:
                self._log("Controller reports an alarm. Use Reset alarm.", "warn")

    def _on_pposf_received(self, values):
        self._tool = values
        for key, cell in self.tool_cells.items():
            cell.set_value(values.get(key))
        if self._move is not None and self._move["source"] == "tool":
            self._check_settle(values)

    def _on_jposf_received(self, joints):
        had_pose = all(self._joints.get(j) is not None for j in MOVABLE_JOINTS)
        self._joints = joints
        for name, jc in self.joint_cards.items():
            jc.set_value(joints.get(name))
        if not had_pose:
            self._refresh_controls()
        if self._move is not None and self._move["source"] == "joints":
            self._check_settle(joints)

    def _on_line_io_received(self, inputs, outputs):
        first = self._line_io is None
        old_order = None if first else decode_order(self._line_io[0])
        self._line_io = (inputs, outputs)
        order = decode_order(inputs)
        if order != old_order:
            self._log(f"Transfer line order: {order.value}", "info" if order is Order.NONE else "ok")
        self._render_line_io()
        if first:
            self._refresh_controls()

    def _render_line_io(self):
        if self._line_io is None:
            self.order_value.setText("—")
            self.order_value.setProperty("kind", "")
            self.order_bits_label.setText(f"IN{IN_ORDER_BIT1} –  IN{IN_ORDER_BIT0} –")
            self.ready_btn.setText("Station ready  —")
            self.busy_btn.setText("Busy  —")
            for w in (self.ready_btn, self.busy_btn):
                w.setProperty("active", False)
        else:
            inputs, outputs = self._line_io
            order = decode_order(inputs)
            b1, b0 = order_bits(inputs)
            self.order_value.setText(order.value)
            self.order_value.setProperty("kind", "" if order is Order.NONE else "active")
            self.order_bits_label.setText(f"IN{IN_ORDER_BIT1} {b1}  IN{IN_ORDER_BIT0} {b0}")
            ready = bit(outputs, OUT_STATION_READY)
            busy = bit(outputs, OUT_BUSY)
            self.ready_btn.setText(f"Station ready  {'ON' if ready else 'OFF'}")
            self.busy_btn.setText(f"Busy  {'ON' if busy else 'OFF'}")
            self.ready_btn.setProperty("active", bool(ready))
            self.busy_btn.setProperty("active", bool(busy))
            self.error_code_sel.set_value(error_code(outputs))
        for w in (self.order_value, self.ready_btn, self.busy_btn):
            w.style().unpolish(w)
            w.style().polish(w)

    def _check_settle(self, readings):
        move = self._move
        if move["targets"] is None:
            return
        current = {k: readings.get(k) for k in move["targets"]}
        if any(v is None for v in current.values()):
            return
        last = move["last"]
        if last is not None and all(abs(current[k] - last[k]) < 0.01 for k in current):
            move["stable"] += 1
        else:
            move["stable"] = 0
        move["last"] = current

        tol = TCP_SETTLE_TOLERANCE_MM if move["source"] == "tool" else SETTLE_TOLERANCE_DEG
        reached = all(abs(current[k] - t) <= tol for k, t in move["targets"].items())
        if (reached and move["stable"] >= 1) or move["stable"] >= STABLE_POLLS_TO_SETTLE:
            self._finish_move(move, reached, current)

    def _finish_move(self, move, reached, current):
        self._move = None
        if self.worker is not None:
            self.worker.fast_poll = False
        self._clear_move_highlight(move)

        tool = move["source"] == "tool"
        unit, tol = (" mm", TCP_SETTLE_TOLERANCE_MM) if tool else ("°", SETTLE_TOLERANCE_DEG)
        if reached:
            self._log(f"{move['label']} reached", "ok")
        else:
            off = ", ".join(f"{k} {current[k]:.2f}{unit} (target {t:.2f}{unit})"
                            for k, t in move["targets"].items()
                            if abs(current[k] - t) > tol)
            self._log(f"{move['label']} stopped early: {off}", "warn")

        if self._loop is not None:
            if reached:
                self._advance_loop()
            else:
                self._end_loop("the robot stopped before reaching the point")
        self._refresh_controls()
        self._refresh_pills()

    def _clear_move_highlight(self, move):
        if move["source"] == "tool":
            self.tcp_moving_label.setText("")
        elif move["targets"]:
            for j in move["targets"]:
                self.joint_cards[j].set_moving(False)
        elif move.get("joint"):
            self.joint_cards[move["joint"]].set_moving(False)

    def _on_command_done(self, name, result):
        self._busy = False
        if name == "servo_on":
            self._servo_on = True
            self._log(f"Servo ON, override {self.speed_slider.value()}%", "warn")
        elif name == "servo_off":
            self._servo_on = False
            self._log("Servo OFF")
        elif name == "reset_alarm":
            self._log("Alarm reset sent")
        elif name == "override":
            self._log(f"Override set to {result}%")
        elif name == "run":
            self._log(f"RUN sent for program {result}", "warn")
        elif name == "ai_exec":
            self._ai_executing = False
            if result.servo_off:
                self._servo_on = False
            for command, reply, _summary in result.lines:
                self._log(f"AI: {command} → {reply or 'ok'}", "ok" if result.ok else "warn")
            if not result.ok:
                self._log(f"AI plan stopped: {result.error}", "error")
            self.ai_panel.on_executed(result)
        elif name in ("line_ready", "line_busy", "line_error"):
            label, outputs = result
            if self._line_io is not None:
                self._line_io = (self._line_io[0], outputs)
            self._render_line_io()
            self._log(f"Transfer line: {label} (outputs {outputs:08b})", "ok")
        elif name == "step" and self._move is not None:
            joint = self._move["joint"]
            self._move["targets"] = {joint: result}
            self.joint_cards[joint].set_moving(True, result)
        elif name == "goto" and self._move is not None:
            targets = {j: result[i] for i, j in enumerate(JOINT_NAMES) if j in MOVABLE_JOINTS}
            self._move["targets"] = targets
            for j, t in targets.items():
                now = self._joints.get(j)
                if now is None or abs(now - t) > SETTLE_TOLERANCE_DEG:
                    self.joint_cards[j].set_moving(True, t)
        elif name == "tcp" and self._move is not None:
            self._move["targets"] = result
            self.tcp_moving_label.setText(
                "moving → " + "  ".join(f"{a} {v:.2f}" for a, v in result.items())
            )
        self._refresh_controls()
        self._refresh_pills()

    # -- user actions (explicit only) ----------------------------------

    def _confirm(self, title, text):
        reply = QMessageBox.warning(self, title, text, QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        return reply == QMessageBox.Yes

    def _on_servo_on_requested(self):
        if not self._srvon_confirmed_this_session:
            if not self._confirm("Confirm servo on", "Robot will move when jogged. Clear the area."):
                return
            self._srvon_confirmed_this_session = True

        link = self.link
        override = self.speed_slider.value()

        def servo_on():
            link.expect_ok("CNTLON")
            link.expect_ok("SRVON")
            time.sleep(SERVO_ON_SETTLE_SECONDS)
            link.expect_ok(f"EXECJOVRD {override}")

        self._log("Turning servo on…")
        self._submit("servo_on", servo_on)

    def _on_servo_off_requested(self):
        if self._loop is not None:
            self._end_loop("servo turned off")
        if self._move is not None:
            self._clear_move_highlight(self._move)
        self._move = None
        if self.worker is not None:
            self.worker.fast_poll = False
        self._submit("servo_off", lambda: self.link.expect_ok("SRVOFF"))

    def _on_reset_alarm_requested(self):
        self._submit("reset_alarm", lambda: self.link.expect_ok("RSTALRM"))

    def _on_override_changed(self):
        link = self.link
        percent = self.speed_slider.value()

        def set_override():
            link.expect_ok(f"EXECJOVRD {percent}")
            return percent

        self._submit("override", set_override)

    def _on_step_clicked(self, joint, sign):
        self._on_step_requested(joint, sign * self.step_size.value())

    def _on_step_requested(self, joint, offset_degrees):
        if (not (self._connected and self._servo_on) or self._busy
                or self._move is not None or self._loop is not None):
            return
        if joint in MISCALIBRATED_JOINTS and not self._j3_confirmed_this_session:
            if not self._confirm(
                f"Confirm {joint} move",
                f"{joint}'s origin is miscalibrated on this robot, so its angle reading "
                f"and soft limits may be wrong. Step {joint} anyway?",
            ):
                return
            self._j3_confirmed_this_session = True

        link = self.link
        joint_number = int(joint[1])
        override = self.speed_slider.value()
        self._move = {"label": f"{joint} step", "source": "joints", "joint": joint,
                      "targets": None, "last": None, "stable": 0}
        self.worker.fast_poll = True
        self.joint_cards[joint].set_moving(True)
        self._log(f"Step {joint} {offset_degrees:+g}° at {override}%")
        self._submit("step", lambda: link.send_step_move(joint_number, offset_degrees, override))

    # -- points and loop ------------------------------------------------

    def _on_save_point(self):
        if not all(self._joints.get(j) is not None for j in MOVABLE_JOINTS):
            return
        used = [int(p["name"][1:]) for p in self._points if p["name"][1:].isdigit()]
        name = f"P{max(used, default=0) + 1}"
        joints = [self._joints.get(j) if j in MOVABLE_JOINTS else None for j in JOINT_NAMES]
        joints = [0.0 if v is None else round(v, 2) for v in joints]
        self._points.append({"name": name, "joints": joints})
        self._persist_points()
        self._render_points()
        self.points_list.setCurrentRow(len(self._points) - 1)
        self._log(f"Saved {name}", "ok")
        self._refresh_controls()

    def _on_delete_point(self):
        row = self.points_list.currentRow()
        if not (0 <= row < len(self._points)) or self._loop is not None:
            return
        removed = self._points.pop(row)
        self._persist_points()
        self._render_points()
        self._log(f"Deleted {removed['name']}")
        self._refresh_controls()

    def _on_clear_points(self):
        if not self._points or self._loop is not None:
            return
        if not self._confirm("Clear points", f"Delete all {len(self._points)} saved points?"):
            return
        self._points = []
        self._persist_points()
        self._render_points()
        self._log("Cleared all points")
        self._refresh_controls()

    def _on_goto_selected(self):
        row = self.points_list.currentRow()
        if not (0 <= row < len(self._points)):
            return
        if not self._goto_confirmed_this_session:
            if not self._confirm(
                "Confirm move to point",
                "The robot moves every joint straight to the saved angles. This can be a "
                "large move. Clear the area.",
            ):
                return
            self._goto_confirmed_this_session = True
        self._goto_point(row)

    def _goto_point(self, index):
        if (not (self._connected and self._servo_on) or self._busy or self._move is not None):
            return False
        point = self._points[index]
        link = self.link
        values = list(point["joints"])
        override = self.speed_slider.value()
        self._move = {"label": f"Move to {point['name']}", "source": "joints", "targets": None,
                      "last": None, "stable": 0}
        self.worker.fast_poll = True
        self.points_list.setCurrentRow(index)
        self._log(f"Moving to {point['name']} at {override}%")
        self._submit("goto", lambda: link.send_joint_move(values, override))
        return True

    def _on_loop_start(self):
        if len(self._points) < 2 or self._loop is not None:
            return
        names = " → ".join(p["name"] for p in self._points)
        if not self._confirm(
            "Start loop",
            f"The robot will move continuously through {len(self._points)} points "
            f"({names} → {self._points[0]['name']} …) at {self.speed_slider.value()}% override "
            f"until you press End loop.\n\nMoves between points can be large. Clear the area.",
        ):
            return
        self._goto_confirmed_this_session = True
        self._loop = {"index": 0, "cycle": 1}
        self._log(f"Loop started: {names}", "warn")
        if not self._goto_point(0):
            self._end_loop("robot was not ready")
        self._refresh_controls()
        self._refresh_pills()

    def _advance_loop(self):
        loop = self._loop
        loop["index"] += 1
        if loop["index"] >= len(self._points):
            loop["index"] = 0
            loop["cycle"] += 1
            self._log(f"Cycle {loop['cycle']}")
        if not self._goto_point(loop["index"]):
            self._end_loop("robot was not ready for the next move")

    def _end_loop(self, reason):
        if self._loop is None:
            return
        cycles = self._loop["cycle"]
        self._loop = None
        self._log(f"Loop ended ({reason}) during cycle {cycles}", "warn")
        self._refresh_controls()
        self._refresh_pills()

    def _on_loop_end(self):
        self._end_loop("End pressed")
        self._on_stop_clicked()

    # -- TCP moves and voice ---------------------------------------------

    def _on_tcp_move_requested(self, axis, distance_mm, direction, source):
        busy_reason = None
        if not self._connected:
            busy_reason = "not connected"
        elif not self._servo_on:
            busy_reason = "servo is off"
        elif self._loop is not None:
            busy_reason = "a loop is running"
        elif self._busy or self._move is not None:
            busy_reason = "robot is busy"
        if busy_reason:
            self._log(f"{source.capitalize()} move ignored: {busy_reason}", "warn")
            return
        if abs(distance_mm) > MAX_TCP_MOVE_MM:
            self._log(f"{source.capitalize()} move ignored: {abs(distance_mm):g} mm is over the "
                      f"{MAX_TCP_MOVE_MM:g} mm limit", "warn")
            return

        link = self.link
        override = self.speed_slider.value()
        self._move = {"label": f"TCP {direction} {abs(distance_mm):g} mm", "source": "tool",
                      "axis": axis, "targets": None, "last": None, "stable": 0}
        self.worker.fast_poll = True
        self.tcp_moving_label.setText(f"moving {direction}…")
        where = "along arm" if axis == "R" else f"{axis}{distance_mm:+g}"
        self._log(f"TCP {direction} {abs(distance_mm):g} mm ({where}) at {override}% [{source}]")
        self._submit("tcp", lambda: link.send_tcp_move(axis, distance_mm, override))

    def _on_voice_toggled(self):
        want_on = self.voice_btn.isChecked()
        if want_on and not self._voice_confirmed_this_session:
            if not self._confirm(
                "Enable voice control",
                "Spoken commands will move the robot's TCP while the servo is on.\n\n"
                "Saying “stop” stops it, but recognition takes 1–2 seconds. Use the red "
                "STOP button or Esc for anything urgent. Clear the area.",
            ):
                self.voice_btn.setChecked(False)
                return
            self._voice_confirmed_this_session = True
        self._set_voice(want_on)

    def _set_voice(self, on):
        if self.voice_listener is None or on == self._voice_on:
            self.voice_btn.setChecked(self._voice_on)
            return
        self._voice_on = on
        self.voice_btn.setChecked(on)
        self.voice_btn.setText("●  Voice on" if on else "●  Voice off")
        self.voice_btn.setProperty("active", on)
        self.voice_btn.style().unpolish(self.voice_btn)
        self.voice_btn.style().polish(self.voice_btn)
        if on:
            self.voice_listener.start(self.mic_combo.currentData())
            self._log("Voice control on (needs internet for Google recognition)", "warn")
        else:
            self.voice_listener.stop()
            self._log("Voice control off")
        self._refresh_controls()

    def _on_voice_status(self, text, kind):
        if self._voice_on or kind == "idle":
            self.voice_btn.setToolTip(text)
        if kind == "error":
            self._log(f"Voice: {text}", "error")
            self._set_voice(False)
        elif text == "Listening" and self._voice_on:
            self._log("Voice: listening", "ok")

    def _on_voice_heard(self, text):
        if not self._voice_on:
            return
        cmd = parse_command(text, require_keyword=self.require_keyword.isChecked(),
                            default_mm=self.tcp_step.value())
        self.voice_heard.setText(f"Heard: “{text}”  →  {cmd.describe()}")
        if cmd.kind == "stop":
            self._log(f"Voice: “{text}” → STOP", "error")
            self._on_stop_clicked()
        elif cmd.kind == "move":
            self._log(f"Voice: “{text}” → {cmd.describe()}")
            self._on_tcp_move_requested(cmd.axis, cmd.sign * cmd.distance_mm, cmd.direction, "voice")
        else:
            self._log(f"Voice: “{text}” → {cmd.describe()}")

    # -- transfer line outputs ----------------------------------------------

    def _on_line_output_requested(self, key, output, label):
        if self._line_io is None:
            return
        on = not bit(self._line_io[1], output)
        link = self.link
        setter = link.set_station_ready if output == OUT_STATION_READY else link.set_busy
        text = f"{label} {'ON' if on else 'OFF'}"
        self._log(f"Transfer line: asserting {text} (OUT{output})…")
        self._submit(f"line_{key}", lambda: (text, setter(on)))

    def _on_error_code_requested(self, code):
        if self._line_io is None:
            return
        link = self.link
        self._log(f"Transfer line: asserting error code {code} "
                  f"(OUT{OUT_ERR_CODE_BIT1}={code >> 1 & 1}, OUT{OUT_ERR_CODE_BIT0}={code & 1})…")
        self._submit("line_error", lambda: (f"error code {code}", link.set_error_code(code)))

    def _on_run_requested(self, program_number):
        link = self.link

        def run():
            link.expect_ok(f"RUN {program_number}")
            return program_number

        self._submit("run", run)

    def _on_stop_clicked(self):
        # Sent directly (not queued) so it goes out as soon as the port is free.
        self.ai_panel.abort()
        self._end_loop("STOP pressed")
        if not self.link.is_open:
            return
        try:
            reply = self.link.send_stop()
        except SerialLinkError as e:
            self._on_link_error(str(e))
            return
        self._log(f"STOP sent ({reply.strip()})", "error")

    # -- AI control -----------------------------------------------------------

    def _fit_window_for_ai(self, show):
        self.ai_panel.setVisible(show)
        self.ai_toggle_btn.setProperty("active", show)
        self.ai_toggle_btn.style().unpolish(self.ai_toggle_btn)
        self.ai_toggle_btn.style().polish(self.ai_toggle_btn)
        extra = AI_PANEL_WIDTH + 16 if show else 0
        self.setMinimumSize(1100 + extra, 740)
        if show and self.width() < 1280 + extra:
            self.resize(min(1280 + extra, self._screen_width), self.height())
        elif not show:
            self.resize(max(1280, self.width() - AI_PANEL_WIDTH - 16), self.height())

    def _ai_context(self):
        line = None
        if self._line_io is not None:
            inputs, outputs = self._line_io
            b1, b0 = order_bits(inputs)
            line = (f"transfer line order {decode_order(inputs).value} (IN7={b1} IN6={b0}), "
                    f"station ready={bit(outputs, OUT_STATION_READY)} busy={bit(outputs, OUT_BUSY)} "
                    f"error code={error_code(outputs)}")
        return {
            "connected": self._connected,
            "servo_on": self._servo_on,
            "alarm": self._alarm,
            "joints": dict(self._joints) if self._joints else None,
            "override": self.speed_slider.value(),
            "line": line,
            "outputs": self._line_io[1] if self._line_io is not None else None,
            "busy": self._busy or self._move is not None or self._loop is not None,
        }

    def _ai_emergency_stop(self, text):
        self._log(f"AI: stop word in “{text}” → STOP", "error")
        self._on_stop_clicked()

    def _ai_execute(self, plan, abort):
        if not self._connected or self.worker is None:
            return False
        link = self.link
        override = self.speed_slider.value()
        progress = self.ai_panel.progress.emit
        self._ai_executing = True
        self._log("AI: executing " + ", ".join(s.command for s in plan.steps), "warn")
        self._submit("ai_exec", lambda: ai_executor.execute(link, plan, abort, override, progress))
        return True

    def closeEvent(self, event):
        self._set_voice(False)
        if self._connected:
            self._shutdown_link(send_servo_off=True)
        super().closeEvent(event)


def main():
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setStyleSheet(THEME_QSS)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
