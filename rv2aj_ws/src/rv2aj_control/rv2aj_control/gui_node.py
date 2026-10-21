"""Operator GUI for the RV-2AJ stack. A pure ROS client: it never opens the
serial port, it calls the driver/sequencer/voice/translator interfaces.

Human confirmations live here (servo on, first J3 step, go-to point, loop
start, voice on); the hard safety rules are enforced by the driver itself.
"""

import json
import math
import sys
import re
import threading
import time
from datetime import datetime

import rclpy
from PyQt5.QtCore import QEvent, QObject, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QKeySequence
from PyQt5.QtWidgets import (
    QAbstractSpinBox,
    QApplication,
    QCheckBox,
    QComboBox,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QShortcut,
    QSlider,
    QSpinBox,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)
from rcl_interfaces.msg import Log
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from rv2aj_interfaces.action import JointMove, TcpMove
from rv2aj_interfaces.msg import MotionCommand, RobotStatus, SequencerStatus
from rv2aj_interfaces.srv import (
    Connect,
    LineSignal,
    PointName,
    RunProgram,
    SavePoint,
    SetOverride,
    SetPointIO,
    SetPointSpeed,
)

from . import ai_brain, ai_speech, ai_stop_listener, ai_voice_fx, kinematics, serial_link
from .voice_commands import DIRECTIONS
from .widgets import THEME_QSS, Card, GlassBackground, JointCard, Segmented, StatCell, StatusPill

DEFAULT_OVERRIDE_PERCENT = 10
STEP_SIZES_DEG = (1.0, 5.0, 10.0, 20.0, 45.0)
TCP_STEPS_MM = (5.0, 10.0, 20.0, 50.0)
JOINT_NAMES = ("J1", "J2", "J3", "J4", "J5", "J6")
MOVABLE_JOINTS = ("J1", "J2", "J3", "J5", "J6")
MISCALIBRATED_JOINTS = {"J3"}
STATUS_TIMEOUT_S = 3.0
LOG_LIMIT = 300
AI_PANEL_WIDTH = 380
SIDE_COLUMN_WIDTH = 320   # Gripper + Robot IO column; the IO card's own content needs ~310px
# A point's own speed override (see sequencer_node._transition_override): the "Slow"/"Fast"
# buttons set a SELECTED point to one of these two values. Edit these two numbers to
# change what "Slow"/"Fast" mean; the point's own exact speed stays editable per-point
# regardless (operator's choice, 2026-10-20).
SLOW_OVERRIDE_PERCENT = 10
FAST_OVERRIDE_PERCENT = 40
AI_KIND_COLORS = {"read": "#3a3a3c", "safe": "#1f9d45", "actuate": "#c46a00", "teach": "#0062cc",
                  "motion": "#d70015"}
LOG_COLORS = {"info": "#3a3a3c", "ok": "#1f9d45", "warn": "#c46a00", "error": "#d70015"}
ROS_LEVEL = {10: "info", 20: "info", 30: "warn", 40: "error", 50: "error"}
TCP_BUTTONS = (
    ("↑  Up", "up"), ("←  Left", "left"), ("Forward", "forward"),
    ("↓  Down", "down"), ("Right  →", "right"), ("Back", "back"),
)


class RosBridge(QObject):
    """rclpy node spinning on its own thread; results come back as Qt signals."""

    status = pyqtSignal(object)
    sequencer = pyqtSignal(object)
    voice_text = pyqtSignal(str)
    voice_command = pyqtSignal(object)
    voice_status = pyqtSignal(str)
    log = pyqtSignal(str, str)
    done = pyqtSignal(str, bool, str)   # what, success, message
    ai_state = pyqtSignal(str)           # JSON from the ai node

    def __init__(self):
        super().__init__()
        self.node = Node("gui")
        n = self.node
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        n.create_subscription(RobotStatus, "status", lambda m: self.status.emit(m), 10)
        n.create_subscription(SequencerStatus, "sequencer/status", lambda m: self.sequencer.emit(m), latched)
        n.create_subscription(String, "voice/text", lambda m: self.voice_text.emit(m.data), 10)
        n.create_subscription(MotionCommand, "voice/command", lambda m: self.voice_command.emit(m), 10)
        n.create_subscription(String, "voice/status", lambda m: self.voice_status.emit(m.data), latched)
        n.create_subscription(Log, "/rosout", self._on_rosout, 50)
        n.create_subscription(String, "ai/state", lambda m: self.ai_state.emit(m.data), latched)
        self.ai_request_pub = n.create_publisher(String, "ai/request", 10)

        self.clients = {
            "connect": n.create_client(Connect, "connect"),
            "disconnect": n.create_client(Trigger, "disconnect"),
            "servo": n.create_client(SetBool, "servo"),
            "stop": n.create_client(Trigger, "stop"),
            "reset_alarm": n.create_client(Trigger, "reset_alarm"),
            "set_override": n.create_client(SetOverride, "set_override"),
            "run_program": n.create_client(RunProgram, "run_program"),
            "points/save": n.create_client(SavePoint, "points/save"),
            "points/set_speed": n.create_client(SetPointSpeed, "points/set_speed"),
            "points/set_io": n.create_client(SetPointIO, "points/set_io"),
            "points/delete": n.create_client(PointName, "points/delete"),
            "points/clear": n.create_client(Trigger, "points/clear"),
            "points/goto": n.create_client(PointName, "points/goto"),
            "loop/start": n.create_client(Trigger, "loop/start"),
            "loop/stop": n.create_client(Trigger, "loop/stop"),
            "voice/enable": n.create_client(SetBool, "voice/enable"),
            "gripper/open": n.create_client(Trigger, "gripper/open"),
            "home/goto": n.create_client(Trigger, "home/goto"),
            "gripper/close": n.create_client(Trigger, "gripper/close"),
            "ai/confirm": n.create_client(Trigger, "ai/confirm"),
            "line/set": n.create_client(LineSignal, "line/set"),
            "ai/cancel": n.create_client(Trigger, "ai/cancel"),
        }
        ai_speech.WHISPER_MODEL = n.declare_parameter("whisper_model", "small").value
        ai_speech.WHISPER_LANGUAGE = n.declare_parameter("whisper_language", "tr").value or None
        if ai_speech.WHISPER_LANGUAGE == "tr":
            ai_speech.WHISPER_PROMPT = ai_brain.WHISPER_PROMPT_TR  # biases recognition toward command words
        planned = n.declare_parameter("use_planner", True).value
        prefix = "planned/" if planned else ""
        self.tcp_action = ActionClient(n, TcpMove, prefix + "tcp_move")
        self.joint_action = ActionClient(n, JointMove, prefix + "joint_move")
        self.planned = planned
        self.translator_params = AsyncParameterClient(n, "translator")
        self.namespace = n.get_namespace().strip("/")

        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(n)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def _on_rosout(self, msg):
        if msg.name.split(".")[0] != self.namespace or msg.name.endswith(".gui"):
            return
        self.log.emit(ROS_LEVEL.get(msg.level, "info"), f"[{msg.name.split('.')[-1]}] {msg.msg}")

    DISCOVERY_WAIT_S = 3.0

    def call(self, name, request, what=None):
        """Call a service without blocking the UI; waits briefly for discovery."""
        what = what or name
        client = self.clients[name]

        def run():
            if not client.wait_for_service(timeout_sec=self.DISCOVERY_WAIT_S):
                self.done.emit(what, False, f"service {name} is not available (is the stack running?)")
                return

            def finished(f):
                response = f.result()
                if response is None:
                    self.done.emit(what, False, "no response")
                else:
                    self.done.emit(what, response.success, response.message)

            client.call_async(request).add_done_callback(finished)

        threading.Thread(target=run, daemon=True).start()

    def send_goal(self, client, goal, what):
        def run():
            if not client.wait_for_server(timeout_sec=self.DISCOVERY_WAIT_S):
                self.done.emit(what, False, f"{client._action_name} is not available (is the stack running?)")
                return

            def accepted(f):
                handle = f.result()
                if handle is None or not handle.accepted:
                    self.done.emit(what, False, "goal rejected (another move is running?)")
                    return
                handle.get_result_async().add_done_callback(
                    lambda r: self.done.emit(what, r.result().result.success, r.result().result.message)
                )

            client.send_goal_async(goal).add_done_callback(accepted)

        threading.Thread(target=run, daemon=True).start()

    def set_translator_param(self, name, value):
        if self.translator_params.services_are_ready():
            self.translator_params.set_parameters([Parameter(name, value=value)])

    def shutdown(self):
        self.executor.shutdown()
        self.node.destroy_node()



class _HotKeys(QObject):
    """Application-wide keys, seen before any widget gets them:
    "+" (main or numpad)  emergency stop, always, even while typing in a field;
    hold "S"              push-to-talk, like holding the talk button (not while typing)."""

    TEXT_FIELDS = (QLineEdit, QAbstractSpinBox, QTextEdit, QPlainTextEdit)

    def __init__(self, window):
        super().__init__(window)
        self.window = window
        self.talking = False

    def _typing(self):
        w = QApplication.focusWidget()
        return isinstance(w, self.TEXT_FIELDS) or (isinstance(w, QComboBox) and w.isEditable())

    def eventFilter(self, obj, event):
        kind = event.type()
        if kind == QEvent.KeyPress and not event.isAutoRepeat():
            if event.key() == Qt.Key_Plus:
                self.window._on_stop()
                return True
            if event.key() == Qt.Key_S and not self._typing() and not self.talking:
                self.talking = True
                self.window.ai_ptt.setDown(True)
                self.window._on_ai_ptt_pressed()
                return True
        elif kind == QEvent.KeyPress and event.isAutoRepeat() and event.key() == Qt.Key_S and self.talking:
            return True  # holding the key: swallow the repeats
        elif kind == QEvent.KeyRelease and not event.isAutoRepeat() and event.key() == Qt.Key_S and self.talking:
            self._release()
            return True
        elif kind == QEvent.ApplicationDeactivate and self.talking:
            self._release()  # window lost focus while holding S: send what was said
        return False

    def _release(self):
        self.talking = False
        self.window.ai_ptt.setDown(False)
        self.window._on_ai_ptt_released()

class MainWindow(QMainWindow):
    """The operator window. One ROS 2 state (self.robot: RobotStatus, self.seq:
    SequencerStatus) drives every widget; there is no other source of truth here.

    Layout, top to bottom:
      header (never scrolls: connection/servo/alarm pills, STOP)
      body, inside a QScrollArea so nothing ever overlaps if a card needs more
      room than the window currently has (_build_body_scroll_area):
        left column:  Joints, then a row of  TCP | [Gripper, Robot IO]
        middle column: Motion (servo/speed/step/home), Points & loop, Event log
        right column: the AI panel (chat, push-to-talk, plan, confirm/cancel)

    Method groups, in the order they appear below:
      _build_*      construct widgets once, in __init__
      _render_line / _on_status / _on_sequencer / _refresh   ROS state -> widgets
      _on_*         user-initiated actions (button clicks, hotkeys, voice)
    Every action that can move the robot asks for a one-time confirmation per
    session (self._j3_confirmed etc.) before its first use; the driver enforces
    the actual safety rules independently of what this GUI does.
    """

    _ai_transcribed = pyqtSignal(str, str)  # text, error (from the whisper thread)
    _ai_heard_stop = pyqtSignal(str)        # from the always-on "dur" listener
    _ai_listener_error = pyqtSignal(str)

    def __init__(self, bridge):
        super().__init__()
        self.ros = bridge
        self.setWindowTitle("RV-2AJ Controller · ROS 2")
        # The real size is set at the end of __init__, once the body (and so its
        # natural size) exists.

        self.robot = None
        self.last_status_time = 0.0
        self.seq = None
        self.pending = set()        # GUI-initiated actions/services still running
        self.voice_state = "off"
        self._srvon_confirmed = False
        self._j3_confirmed = False
        self._goto_confirmed = False
        self._voice_confirmed = False

        root = GlassBackground()
        self._background = root  # the scroll area re-invalidates its painted shadows against this
        self.setCentralWidget(root)
        outer = QVBoxLayout(root)
        outer.setContentsMargins(20, 16, 20, 20)
        outer.setSpacing(16)
        # The header holds STOP and the connection/servo/alarm state: it must never
        # scroll out of view, so it sits outside the scroll area below.
        outer.addWidget(self._build_header())
        outer.addWidget(self._build_body_scroll_area(), 1)

        screen = QApplication.primaryScreen()
        width = screen.availableGeometry().width() if screen else 1920
        self.resize(min(1280 + AI_PANEL_WIDTH + 16, width), 920)
        self.setMinimumSize(900, 620)

        QShortcut(QKeySequence(Qt.Key_Escape), self, activated=self._on_stop,
                  context=Qt.ApplicationShortcut)
        # "+" = emergency stop everywhere (even while typing), hold "S" = push-to-talk.
        self._keys = _HotKeys(self)
        QApplication.instance().installEventFilter(self._keys)

        bridge.status.connect(self._on_status)
        bridge.sequencer.connect(self._on_sequencer)
        bridge.voice_text.connect(self._on_voice_text)
        bridge.voice_command.connect(self._on_voice_command)
        bridge.voice_status.connect(self._on_voice_status)
        bridge.log.connect(self._log)
        bridge.done.connect(self._on_done)
        bridge.ai_state.connect(self._on_ai_state)
        self._ai_transcribed.connect(self._on_ai_transcribed)
        self._ai_heard_stop.connect(self._on_ai_heard_stop)
        self._ai_listener_error.connect(lambda msg: self._on_ai_listener_error(msg))
        ai_voice_fx.prebuild()
        if self.ai_stop_box.isChecked():
            # Open the microphone only once the window is fully up (starting it
            # during Qt start-up crashed the GUI once).
            QTimer.singleShot(3000, lambda: self.ai_stop_box.isChecked() and self.ai_stop_listener.start())

        self.watchdog = QTimer(self)
        self.watchdog.timeout.connect(self._refresh)
        self.watchdog.start(1000)
        self._refresh()
        self._log("info", "GUI started. Waiting for the driver…")

    # -- layout ---------------------------------------------------------------

    def _build_header(self):
        header = QWidget()
        layout = QHBoxLayout(header)
        layout.setContentsMargins(4, 0, 0, 0)
        layout.setSpacing(12)
        titles = QVBoxLayout()
        titles.setSpacing(0)
        title = QLabel("RV-2AJ Controller")
        title.setObjectName("appTitle")
        subtitle = QLabel("Mitsubishi CR1-571  ·  ROS 2 · "
                          + ("MoveIt-validated motion" if self.ros.planned else "direct motion (no MoveIt)"))
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
        ports = serial_link.list_available_ports()
        if serial_link.PORT_DEFAULT not in ports:
            ports.insert(0, serial_link.PORT_DEFAULT)
        self.port_combo.addItems(ports)
        self.port_combo.setCurrentText(serial_link.PORT_DEFAULT)
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.setObjectName("primary")
        self.connect_btn.setMinimumWidth(110)
        self.connect_btn.clicked.connect(self._on_connect_toggled)
        layout.addWidget(self.port_combo)
        layout.addWidget(self.connect_btn)
        layout.addSpacing(12)

        self.stop_btn = QPushButton("STOP")
        self.stop_btn.setObjectName("estop")
        self.stop_btn.setToolTip("Stop motion and any loop  ( + or Esc )")
        self.stop_btn.setCursor(Qt.PointingHandCursor)
        self.stop_btn.clicked.connect(self._on_stop)
        layout.addWidget(self.stop_btn)
        return header

    def _build_body_scroll_area(self):
        """Everything below the header: Joints/TCP on the left, Motion/Points/Log in
        the middle, the AI panel on the right. Wrapped in a QScrollArea so that on a
        small screen, or if a card ever needs more room than it's given, the content
        SCROLLS instead of being squeezed into overlapping widgets."""
        left = QVBoxLayout()
        left.setSpacing(16)
        left.addWidget(self._build_joints_card())
        bottom = QHBoxLayout()
        bottom.setSpacing(16)
        bottom.addWidget(self._build_tcp_card(), 1)
        side = QVBoxLayout()
        side.setSpacing(16)
        side.setContentsMargins(0, 0, 0, 0)
        side.addWidget(self._build_gripper_card())
        side.addWidget(self._build_line_card())
        side.addStretch(1)
        side_widget = QWidget()
        side_widget.setLayout(side)
        side_widget.setFixedWidth(SIDE_COLUMN_WIDTH)
        bottom.addWidget(side_widget)
        left.addLayout(bottom)

        right_widget = QWidget()
        right_widget.setFixedWidth(380)
        right = QVBoxLayout(right_widget)
        right.setContentsMargins(0, 0, 0, 0)
        right.setSpacing(16)
        right.addWidget(self._build_motion_card())
        right.addWidget(self._build_points_card(), 3)
        right.addWidget(self._build_log_card(), 2)

        body = QHBoxLayout()
        body.setSpacing(16)
        body.addLayout(left, 1)
        body.addWidget(right_widget)
        self.ai_card = self._build_ai_card()
        self.ai_card.setFixedWidth(AI_PANEL_WIDTH)
        body.addWidget(self.ai_card)

        body_widget = QWidget()
        body_widget.setObjectName("bodyScrollContent")
        body_widget.setLayout(body)

        scroll = QScrollArea()
        scroll.setObjectName("bodyScroll")
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        scroll.setWidget(body_widget)
        # GlassBackground caches each Card's painted drop-shadow at a fixed screen
        # position and only repaints it on that Card's own resize/move/show/hide
        # (see widgets.Card). Scrolling moves cards on screen without firing any of
        # those (their position relative to this scroll area's content widget never
        # changes), so without this the shadows would freeze in place while the
        # cards they belong to scroll away underneath them.
        scroll.horizontalScrollBar().valueChanged.connect(self._background.invalidate)
        scroll.verticalScrollBar().valueChanged.connect(self._background.invalidate)
        return scroll

    def _build_joints_card(self):
        card = Card("Joints")
        grid = QGridLayout()
        grid.setSpacing(12)
        self.joint_cards = {}
        for i, name in enumerate(JOINT_NAMES):
            jc = JointCard(name, absent=name not in MOVABLE_JOINTS,
                           warning="check origin" if name in MISCALIBRATED_JOINTS else None,
                           locked="LOCKED · encoder fault" if name in kinematics.FROZEN_JOINTS else None)
            jc.step_clicked.connect(lambda sign, n=name: self._on_joint_step(n, sign * self.step_size.value()))
            self.joint_cards[name] = jc
            grid.addWidget(jc, i // 3, i % 3)
        # No row stretch: each JointCard keeps its own natural (compact) height
        # instead of ballooning to fill whatever space a tall window happens to give it.
        card.body.addLayout(grid)
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
        card.body.addLayout(row)
        self.moving_label = QLabel()
        self.moving_label.setObjectName("jointHint")
        self.moving_label.setProperty("kind", "accent")
        card.body.addWidget(self.moving_label)
        card.body.addLayout(self._build_program_row())

        dist_row = QHBoxLayout()
        dist_label = QLabel("Distance per move (mm)")
        dist_label.setObjectName("dim")
        self.tcp_step = Segmented([(f"{s:g}", s) for s in TCP_STEPS_MM], default_index=1)
        self.tcp_step.setFixedWidth(220)
        self.tcp_step.changed.connect(lambda v: self.ros.set_translator_param("default_distance_mm", float(v)))
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
            btn.setCursor(Qt.PointingHandCursor)
            btn.setToolTip("out along / back along the arm's reach" if axis == "R"
                           else f"{'+' if sign > 0 else '−'}{axis} (base frame)")
            btn.clicked.connect(lambda _=False, a=axis, s=sign, d=direction:
                                self._on_tcp_move(a, s * self.tcp_step.value(), d))
            self.tcp_buttons.append(btn)
            pad.addWidget(btn, i // 3, i % 3)
        card.body.addLayout(pad)

        voice_row = QHBoxLayout()
        voice_row.setSpacing(8)
        self.voice_btn = QPushButton("●  Voice off")
        self.voice_btn.setObjectName("voiceBtn")
        self.voice_btn.setCheckable(True)
        self.voice_btn.clicked.connect(self._on_voice_toggled)
        self.voice_state_label = QLabel("voice node: —")
        self.voice_state_label.setObjectName("dim")
        self.require_keyword = QCheckBox("Require “move”")
        self.require_keyword.setChecked(True)
        self.require_keyword.toggled.connect(lambda v: self.ros.set_translator_param("require_keyword", bool(v)))
        voice_row.addWidget(self.voice_btn)
        voice_row.addWidget(self.voice_state_label, 1)
        voice_row.addWidget(self.require_keyword)
        card.body.addLayout(voice_row)

        self.voice_heard = QLabel("Say “move up 20”, “go left 10”, “go to point 2”, “start loop”, “stop”.")
        self.voice_heard.setObjectName("dim")
        self.voice_heard.setWordWrap(True)
        card.body.addWidget(self.voice_heard)
        return card

    def _build_motion_card(self):
        card = Card("Motion")
        servo_row = QHBoxLayout()
        servo_row.setSpacing(8)
        self.servo_on_btn = QPushButton("Servo ON")
        self.servo_on_btn.setObjectName("servoOn")
        self.servo_on_btn.clicked.connect(self._on_servo_on)
        self.servo_off_btn = QPushButton("Servo OFF")
        self.servo_off_btn.setObjectName("servoOff")
        self.servo_off_btn.clicked.connect(lambda: self._call("servo", SetBool.Request(data=False), "servo off"))
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
        self.speed_slider.sliderReleased.connect(
            lambda: self._call("set_override", SetOverride.Request(percent=self.speed_slider.value()),
                               "override"))
        card.body.addWidget(self.speed_slider)

        step_label = QLabel("Joint step size")
        step_label.setObjectName("dim")
        card.body.addWidget(step_label)
        self.step_size = Segmented([(f"{s:g}°", s) for s in STEP_SIZES_DEG], default_index=1)
        self.step_size.changed.connect(self._apply_step_size)
        card.body.addWidget(self.step_size)
        self._apply_step_size(self.step_size.value())

        self.reset_alarm_btn = QPushButton("Reset alarm")
        self.reset_alarm_btn.clicked.connect(lambda: self._call("reset_alarm", Trigger.Request()))
        buttons = QHBoxLayout()
        buttons.setSpacing(8)
        self.home_btn = QPushButton("⌂  Home")
        self.home_btn.setObjectName("homeBtn")
        self.home_btn.setToolTip("Joint move to the saved home pose (home_joints_deg)")
        self.home_btn.clicked.connect(self._on_home)
        buttons.addWidget(self.home_btn)
        buttons.addWidget(self.reset_alarm_btn)
        card.body.addLayout(buttons)
        return card

    def _build_gripper_card(self):
        card = Card("Gripper")
        row = QHBoxLayout()
        row.setSpacing(8)
        # Plain clicks: open/close are discrete state commands, not motion.
        self.gripper_open_btn = QPushButton("OPEN")
        self.gripper_open_btn.clicked.connect(lambda: self._on_gripper(True))
        self.gripper_close_btn = QPushButton("CLOSE")
        self.gripper_close_btn.clicked.connect(lambda: self._on_gripper(False))
        row.addWidget(self.gripper_open_btn)
        row.addWidget(self.gripper_close_btn)
        card.body.addLayout(row)
        self.gripper_state = QLabel("Last commanded: Unknown")
        self.gripper_state.setObjectName("dim")
        self.gripper_state.setWordWrap(True)
        card.body.addWidget(self.gripper_state)
        warning = QLabel("⚠ 160 N at 6 bar: keep fingers clear.")
        warning.setObjectName("note")
        warning.setWordWrap(True)
        card.body.addWidget(warning)
        self.gripper_error = ""
        return card

    # Robot I/O in use on Station 4 (survey 2026-10-16): general 0-15, hand 900-907. All outputs writable.
    # (row title, status field, first bit number, is output)
    IO_ROWS = (("IN 0", "line_inputs", 0, False), ("IN 8", "inputs_8_15", 8, False),
               ("OUT 0", "line_outputs", 0, True), ("OUT 8", "outputs_8_15", 8, True),
               ("EL G", "hand_inputs", 900, False), ("EL Ç", "hand_outputs", 900, True))
    IO_NAMES = {("IN", 6): "IN6 · sipariş bit 0", ("IN", 7): "IN7 · sipariş bit 1",
                ("OUT", 4): "OUT4 · hata kodu bit 0", ("OUT", 5): "OUT5 · hata kodu bit 1",
                ("OUT", 6): "OUT6 · Meşgul", ("OUT", 7): "OUT7 · İstasyon hazır",
                ("OUT", 900): "OUT900 · el 1 (tutucu)", ("OUT", 901): "OUT901 · el 1 (tutucu)"}

    def _add_io_row(self, grid, row, title, field, first, is_out):
        """One row of the Robot IO grid: a row label plus 8 LED buttons for bits
        `first`..`first+7` of one status byte (see IO_ROWS). Output rows are
        clickable (toggle that one bit); input rows are read-only indicators.
        Stores the row's buttons in self.io_leds[field] for _render_line to update."""
        label = QLabel(title)
        label.setObjectName("ioRowLabel")
        grid.addWidget(label, row, 0)
        leds = []
        for b in range(8):
            n = first + b
            kind = "OUT" if is_out else "IN"
            led = QPushButton(str(n % 10) if first < 900 else str(b))
            led.setObjectName("ioLed")
            led.setFocusPolicy(Qt.NoFocus)
            led.setToolTip(self.IO_NAMES.get((kind, n), f"{kind}{n}") +
                           ("  (tıkla: aç/kapat)" if is_out else ""))
            led.setProperty("clickable", is_out)
            if is_out:
                led.clicked.connect(lambda _=False, f=field, n=n, b=b: self._on_out_click(f, n, b))
            else:
                led.setAttribute(Qt.WA_TransparentForMouseEvents, True)
            grid.addWidget(led, row, b + 1)
            leds.append(led)
        self.io_leds[field] = leds

    def _build_line_card(self):
        card = Card("Robot IO · İstasyon 4")
        row = QHBoxLayout()
        self.line_order = QLabel("—")
        self.line_order.setObjectName("orderValue")
        self.line_bits = QLabel("")
        self.line_bits.setObjectName("dim")
        row.addWidget(self.line_order)
        row.addStretch(1)
        row.addWidget(self.line_bits)
        card.body.addLayout(row)
        grid = QGridLayout()
        grid.setHorizontalSpacing(3)
        grid.setVerticalSpacing(5)
        self.io_leds = {}   # status field -> [8 LEDs], filled in by _add_io_row below
        for r, (title, field, first, is_out) in enumerate(self.IO_ROWS):
            self._add_io_row(grid, r, title, field, first, is_out)
        card.body.addLayout(grid)
        legend = QLabel("Çıkışlar tıklanabilir · EL = el (pnömatik) IO 900-907 · 900/901 tutucu")
        legend.setObjectName("dim")
        legend.setWordWrap(True)
        card.body.addWidget(legend)
        self.io_record_box = QCheckBox("Tıklananı seçili noktaya kaydet")
        self.io_record_box.setToolTip("Açıkken, bir çıkışa tıklamak onu robotta da değiştirir VE seçili "
                                      "noktaya kaydeder: nokta oraya ulaşınca bu çıkış otomatik ayarlanır "
                                      "(tutucu açma/kapama gibi).")
        card.body.addWidget(self.io_record_box)
        toggles = QHBoxLayout()
        self.line_ready_btn = QPushButton("Hazır")
        self.line_busy_btn = QPushButton("Meşgul")
        for btn, signal in ((self.line_ready_btn, "ready"), (self.line_busy_btn, "busy")):
            btn.setObjectName("ioToggle")
            btn.clicked.connect(lambda _=False, sig=signal: self._on_line_toggle(sig))
            toggles.addWidget(btn, 1)
        card.body.addLayout(toggles)
        error_row = QHBoxLayout()
        error_label = QLabel("Hata kodu")
        error_label.setObjectName("dim")
        self.line_error = Segmented([(str(c), c) for c in range(4)])
        self.line_error.setToolTip("İstasyona gönderilen hata kodu (0 = hata yok)")
        self.line_error.changed.connect(lambda code: self._call(
            "line/set", LineSignal.Request(signal="error_code", value=int(code)), f"hata kodu {code}"))
        error_row.addWidget(error_label)
        error_row.addWidget(self.line_error, 1)
        card.body.addLayout(error_row)
        self.line_reset_btn = QPushButton("Genel çıkışları sıfırla (OUT0-15)")
        self.line_reset_btn.setToolTip("El (pnömatik) çıkışlarına dokunmaz: valfler aniden yön değiştirmesin.")
        self.line_reset_btn.clicked.connect(self._on_line_reset)
        card.body.addWidget(self.line_reset_btn)
        return card

    def _on_out_click(self, field, n, b):
        """Toggle one output; the driver writes that bit only and reads its byte back.
        With "record to point" checked and a point selected, also save that output +
        value against the point (set after it's reached, same slot as the gripper)."""
        r = self.robot
        value = getattr(r, field, -1) if r is not None else -1
        if value < 0:
            return
        on = not (value >> b) & 1
        self._call("line/set", LineSignal.Request(signal=f"out{n}", value=int(on)),
                   f"OUT{n} {'açık' if on else 'kapalı'}")
        name = self._selected_point()
        if self.io_record_box.isChecked() and name:
            self._call("points/set_io", SetPointIO.Request(name=name, output=n, value=bool(on)),
                       f"{name}: OUT{n} = {int(on)} kaydedildi")

    def _on_line_reset(self):
        r = self.robot
        if r is None:
            return
        for field, first in (("line_outputs", 0), ("outputs_8_15", 8)):
            value = getattr(r, field, -1)
            for b in range(8):
                if value >= 0 and (value >> b) & 1:
                    self._call("line/set", LineSignal.Request(signal=f"out{first + b}", value=0),
                               f"OUT{first + b} kapalı")

    def _on_line_toggle(self, signal):
        r = self.robot
        if r is None or r.line_outputs < 0:
            return
        bit = 7 if signal == "ready" else 6
        on = not (r.line_outputs >> bit) & 1
        self._call("line/set", LineSignal.Request(signal=signal, value=int(on)),
                   f"{'hazır' if signal == 'ready' else 'meşgul'} {'açık' if on else 'kapalı'}")

    @staticmethod
    def _set_led(led, on):
        if led.property("on") != on:
            led.setProperty("on", on)
            led.style().unpolish(led)
            led.style().polish(led)

    def _render_line(self, r):
        if r is None or r.line_inputs < 0 or r.line_outputs < 0 or not r.connected:
            self.line_order.setText("—")
            self.line_bits.setText("IO okunamıyor" if r is not None and r.connected else "")
            for w in (self.line_ready_btn, self.line_busy_btn, self.line_error, self.line_reset_btn):
                w.setEnabled(False)
            for leds in self.io_leds.values():
                for led in leds:
                    self._set_led(led, False)
                    led.setEnabled(False)
            return
        for title, field, first, is_out in self.IO_ROWS:
            value = getattr(r, field, -1)
            for b, led in enumerate(self.io_leds[field]):
                self._set_led(led, value >= 0 and bool((value >> b) & 1))
                led.setEnabled(value >= 0)
        order = {(0, 0): "Sipariş yok", (0, 1): "Montaj", (1, 0): "Ayıklama", (1, 1): "Paletten al"}[
            ((r.line_inputs >> 7) & 1, (r.line_inputs >> 6) & 1)]
        self.line_order.setText(order)
        self.line_order.setProperty("kind", "" if order == "Sipariş yok" else "active")
        self.line_order.style().unpolish(self.line_order)
        self.line_order.style().polish(self.line_order)
        self.line_bits.setText(f"IN {r.line_inputs:08b}  OUT {r.line_outputs:08b}")
        for btn, bit, name in ((self.line_ready_btn, 7, "Hazır"), (self.line_busy_btn, 6, "Meşgul")):
            on = bool((r.line_outputs >> bit) & 1)
            btn.setText(f"{name}  {'AÇIK' if on else 'KAPALI'}")
            btn.setProperty("active", on)
            btn.style().unpolish(btn)
            btn.style().polish(btn)
            btn.setEnabled(True)
        self.line_error.set_value(((r.line_outputs >> 5) & 1) << 1 | ((r.line_outputs >> 4) & 1))
        self.line_error.setEnabled(True)
        self.line_reset_btn.setEnabled(True)

    def _build_program_row(self):
        """Controller program: one compact row inside the TCP card (RUN is rarely used)."""
        row = QHBoxLayout()
        row.setSpacing(8)
        self.program_spin = QSpinBox()
        self.program_spin.setRange(1, 9999)
        self.program_spin.setPrefix("Prog  ")
        self.program_spin.setButtonSymbols(QSpinBox.NoButtons)
        self.program_spin.setFixedWidth(110)
        self.run_btn = QPushButton("Run")
        self.run_btn.setToolTip("RUN has not been tested on this controller yet.")
        self.run_btn.clicked.connect(
            lambda: self._call("run_program", RunProgram.Request(program=self.program_spin.value()), "run"))
        self.program_stop_btn = QPushButton("Stop")
        self.program_stop_btn.clicked.connect(self._on_stop)
        self.state_raw = QLineEdit()
        self.state_raw.setObjectName("stateRaw")
        self.state_raw.setReadOnly(True)
        self.state_raw.setPlaceholderText("STATE reply")
        row.addWidget(self.program_spin)
        row.addWidget(self.run_btn)
        row.addWidget(self.program_stop_btn)
        row.addWidget(self.state_raw, 1)
        return row

    def _build_points_card(self):
        card = Card("Points & loop")
        # Two rows instead of one cramped row of 5: every label gets enough room
        # to never truncate or overflow, whatever font the system substitutes.
        edit_row1 = QHBoxLayout()
        edit_row1.setSpacing(8)
        self.save_point_btn = QPushButton("＋ Save point")
        self.save_point_btn.setObjectName("primary")
        self.save_point_btn.clicked.connect(lambda: self._call("points/save", SavePoint.Request(), "save point"))
        self.insert_point_btn = QPushButton("＋ Insert after")
        self.insert_point_btn.setToolTip("Save the current pose as a new point right after the selected one "
                                         "(e.g. between P1 and P2: select P1)")
        self.insert_point_btn.clicked.connect(self._on_insert_point)
        edit_row1.addWidget(self.save_point_btn, 1)
        edit_row1.addWidget(self.insert_point_btn, 1)
        card.body.addLayout(edit_row1)

        edit_row2 = QHBoxLayout()
        edit_row2.setSpacing(8)
        self.goto_btn = QPushButton("Go to")
        self.goto_btn.clicked.connect(self._on_goto)
        self.delete_point_btn = QPushButton("Delete")
        self.delete_point_btn.clicked.connect(self._on_delete_point)
        self.clear_points_btn = QPushButton("Clear")
        self.clear_points_btn.clicked.connect(self._on_clear_points)
        edit_row2.addWidget(self.goto_btn, 1)
        edit_row2.addWidget(self.delete_point_btn, 1)
        edit_row2.addWidget(self.clear_points_btn, 1)
        card.body.addLayout(edit_row2)

        speed_label = QLabel("Selected point's speed (a move uses the slower of its two points):")
        speed_label.setObjectName("dim")
        speed_label.setWordWrap(True)
        card.body.addWidget(speed_label)
        speed_row = QHBoxLayout()
        speed_row.setSpacing(8)
        self.slow_point_btn = QPushButton(f"Slow ({SLOW_OVERRIDE_PERCENT}%)")
        self.slow_point_btn.setToolTip("Set the selected point to this speed. A move uses the SLOWER of its "
                                       "two points, so this also slows the approach and the retreat.")
        self.slow_point_btn.clicked.connect(lambda: self._on_set_point_speed(SLOW_OVERRIDE_PERCENT))
        self.fast_point_btn = QPushButton(f"Fast ({FAST_OVERRIDE_PERCENT}%)")
        self.fast_point_btn.setToolTip("Set the selected point to this speed (only takes effect for a "
                                       "transition whose OTHER point is at least this fast too).")
        self.fast_point_btn.clicked.connect(lambda: self._on_set_point_speed(FAST_OVERRIDE_PERCENT))
        speed_row.addWidget(self.slow_point_btn, 1)
        speed_row.addWidget(self.fast_point_btn, 1)
        card.body.addLayout(speed_row)

        io_row = QHBoxLayout()
        io_row.setSpacing(8)
        self.point_io_label = QLabel("")
        self.point_io_label.setObjectName("dim")
        self.point_io_label.setWordWrap(True)
        self.clear_point_io_btn = QPushButton("Çıkışları temizle")
        self.clear_point_io_btn.setToolTip("Bu noktaya kaydedilmiş tüm çıkış eylemlerini kaldırır "
                                          "(noktanın kendisini veya tutucusunu etkilemez).")
        self.clear_point_io_btn.clicked.connect(self._on_clear_point_io)
        io_row.addWidget(self.point_io_label, 1)
        io_row.addWidget(self.clear_point_io_btn)
        card.body.addLayout(io_row)

        header = QLabel("      " + "".join(f"{j:>7}" for j in MOVABLE_JOINTS) + "   Grip  Spd")
        header.setObjectName("pointsHeader")
        card.body.addWidget(header)
        self.points_list = QListWidget()
        self.points_list.setObjectName("points")
        self.points_list.setMinimumHeight(110)
        self.points_list.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.points_list.itemSelectionChanged.connect(self._refresh)
        self.points_list.itemDoubleClicked.connect(lambda _item: self._on_goto())
        card.body.addWidget(self.points_list, 1)

        loop_row = QHBoxLayout()
        loop_row.setSpacing(8)
        self.loop_start_btn = QPushButton("▶  Start loop")
        self.loop_start_btn.setObjectName("loopStart")
        self.loop_start_btn.clicked.connect(self._on_loop_start)
        self.loop_end_btn = QPushButton("■  End loop")
        self.loop_end_btn.setObjectName("loopEnd")
        self.loop_end_btn.clicked.connect(lambda: self._call("loop/stop", Trigger.Request(), "end loop"))
        loop_row.addWidget(self.loop_start_btn, 1)
        loop_row.addWidget(self.loop_end_btn, 1)
        card.body.addLayout(loop_row)
        self.loop_status = QLabel()
        self.loop_status.setObjectName("dim")
        card.body.addWidget(self.loop_status)
        return card

    def _build_log_card(self):
        card = Card("Event log")
        self.log_list = QListWidget()
        self.log_list.setObjectName("log")
        self.log_list.setWordWrap(True)
        card.body.addWidget(self.log_list, 1)
        return card

    # -- AI control --------------------------------------------------------------

    def _build_ai_card(self):
        card = Card("Yapay Zekâ Kontrolü")
        top = QHBoxLayout()
        self.ai_pill = StatusPill("YZ …", "idle")
        top.addWidget(self.ai_pill)
        top.addStretch(1)
        self.ai_llm_label = QLabel("")
        self.ai_llm_label.setObjectName("dim")
        top.addWidget(self.ai_llm_label)
        card.body.addLayout(top)

        self.ai_stop_listener = ai_stop_listener.StopListener(
            on_stop=self._ai_heard_stop.emit, on_error=self._ai_listener_error.emit)
        self.ai_stop_box = QCheckBox("Sürekli “dur” dinle (ek güvenlik)")
        self.ai_stop_box.setToolTip("Mikrofon sürekli dinler; “dur dur” / “robotu durdur” duyunca robotu durdurur. "
                                    "STOP düğmesi ve acil stop her zaman asıl durdurma yoludur.")
        listen_ok = ai_stop_listener.available()
        self.ai_stop_box.setEnabled(listen_ok)
        self.ai_stop_box.setChecked(listen_ok and self.ros.node.declare_parameter("stop_listener", True).value)
        self.ai_stop_box.toggled.connect(
            lambda on: self.ai_stop_listener.start() if on else self.ai_stop_listener.stop())
        card.body.addWidget(self.ai_stop_box)

        self.ai_chat = QListWidget()
        self.ai_chat.setObjectName("aiChat")
        self.ai_chat.setWordWrap(True)
        self.ai_chat.setMinimumHeight(160)
        card.body.addWidget(self.ai_chat, 3)
        self._ai_chat_len = 0
        # "Typing" bubble at the end of the chat while the robot listens, transcribes,
        # thinks or moves: animated dots, so waiting feels like a conversation.
        self._ai_local_busy = None   # set by the GUI itself (recording / transcribing / just sent)
        self._ai_node_busy = None    # derived from the AI node's phase
        self._ai_typing_item = None
        self._ai_dots = 0
        self._ai_typing_timer = QTimer(self)
        self._ai_typing_timer.setInterval(350)
        self._ai_typing_timer.timeout.connect(self._tick_ai_typing)

        self.ai_recorder = ai_speech.Recorder() if ai_speech.recording_available() else None
        self.ai_transcriber = ai_speech.Transcriber()
        # Load Whisper in the background shortly after start, so the first command is not slow.
        threading.Timer(5.0, lambda: self.ai_transcriber.load() if self.ai_recorder is not None else None).start()
        self.ai_recording = False
        self.ai_ptt = QPushButton("Basılı tut ve konuş  (S)")
        self.ai_ptt.setObjectName("pttBtn")
        self.ai_ptt.setEnabled(self.ai_recorder is not None)
        self.ai_ptt.pressed.connect(self._on_ai_ptt_pressed)
        self.ai_ptt.released.connect(self._on_ai_ptt_released)
        card.body.addWidget(self.ai_ptt)

        row = QHBoxLayout()
        self.ai_input = QLineEdit()
        self.ai_input.setPlaceholderText("örn. 3 santim yukarı çık, sonra tutucuyu aç")
        self.ai_input.returnPressed.connect(self._on_ai_send)
        send = QPushButton("Gönder")
        send.clicked.connect(self._on_ai_send)
        row.addWidget(self.ai_input, 1)
        row.addWidget(send)
        card.body.addLayout(row)

        self.ai_transcript = QLabel("Benimle normal konuşabilirsiniz. “dur” her zaman durdurur.")
        self.ai_transcript.setObjectName("aiTranscript")
        self.ai_transcript.setWordWrap(True)
        card.body.addWidget(self.ai_transcript)

        head = QLabel("Önerilen plan")
        head.setObjectName("dim")
        card.body.addWidget(head)
        self.ai_plan = QListWidget()
        self.ai_plan.setObjectName("aiPlan")
        self.ai_plan.setWordWrap(True)
        self.ai_plan.setMaximumHeight(120)
        card.body.addWidget(self.ai_plan, 1)

        buttons = QHBoxLayout()
        self.ai_confirm_btn = QPushButton("Onayla")
        self.ai_confirm_btn.setObjectName("primary")
        self.ai_confirm_btn.clicked.connect(
            lambda: self.ros.call("ai/confirm", Trigger.Request(), "AI confirm"))
        self.ai_cancel_btn = QPushButton("İptal")
        self.ai_cancel_btn.clicked.connect(lambda: self.ros.call("ai/cancel", Trigger.Request(), "AI cancel"))
        buttons.addWidget(self.ai_confirm_btn, 1)
        buttons.addWidget(self.ai_cancel_btn, 1)
        card.body.addLayout(buttons)

        self.ai_response = QLabel("")
        self.ai_response.setObjectName("aiResponse")
        self.ai_response.setWordWrap(True)
        card.body.addWidget(self.ai_response)
        note = QLabel("Hareketler MoveIt ile planlanır. Net komutlar hemen çalışır; emin olmadığında onayınızı ister.")
        note.setObjectName("note")
        note.setWordWrap(True)
        card.body.addWidget(note)
        self.ai_confirm_btn.setEnabled(False)
        self.ai_cancel_btn.setEnabled(False)
        return card

    TYPING_DOTS = ("●  ·  ·", "·  ●  ·", "·  ·  ●")

    def _set_ai_local_busy(self, label):
        self._ai_local_busy = label
        self._ai_local_busy_since = time.monotonic()
        self._update_ai_typing()

    def _update_ai_typing(self):
        label = self._ai_local_busy or self._ai_node_busy
        if label is None:
            self._ai_typing_timer.stop()
            if self._ai_typing_item is not None:
                row = self.ai_chat.row(self._ai_typing_item)
                if row >= 0:
                    self.ai_chat.takeItem(row)
                self._ai_typing_item = None
            return
        if self._ai_typing_item is None or self.ai_chat.row(self._ai_typing_item) < 0:
            self._ai_typing_item = QListWidgetItem()
            font = self._ai_typing_item.font()
            font.setItalic(True)
            self._ai_typing_item.setFont(font)
            self._ai_typing_item.setForeground(QColor("#8e8e93"))
            self.ai_chat.addItem(self._ai_typing_item)
            self.ai_chat.scrollToBottom()
        self._ai_typing_label = label
        self._tick_ai_typing(advance=False)
        if not self._ai_typing_timer.isActive():
            self._ai_typing_timer.start()

    def _tick_ai_typing(self, advance=True):
        if self._ai_typing_item is None:
            return
        if advance:
            self._ai_dots = (self._ai_dots + 1) % len(self.TYPING_DOTS)
            if self._ai_local_busy == "düşünüyor" and time.monotonic() - self._ai_local_busy_since > 30:
                self._set_ai_local_busy(None)  # no answer came (node down?): do not spin forever
                return
        who = "Siz" if self._ai_local_busy in ("dinliyorum", "yazıya çeviriyorum") else "Robot"
        self._ai_typing_item.setText(f"{who}:  {self.TYPING_DOTS[self._ai_dots]}   {self._ai_typing_label}")
        if who == "Siz":
            self._ai_typing_item.setTextAlignment(Qt.AlignRight)
        else:
            self._ai_typing_item.setTextAlignment(Qt.AlignLeft)

    def _ai_submit(self, text):
        text = text.strip()
        if not text:
            return
        self.ai_transcript.setText(f"You: “{text}”")
        if ai_brain.is_emergency_stop(text):  # never routed through the model
            self._on_stop()
            self.ai_response.setText("Durduruldu.")
            return
        self.ros.ai_request_pub.publish(String(data=text))
        self._set_ai_local_busy("düşünüyor")

    def _on_ai_send(self):
        text = self.ai_input.text()
        self.ai_input.clear()
        self._ai_submit(text)

    def _on_ai_ptt_pressed(self):
        if self.ai_recorder is None or self.ai_recording:
            return
        ai_voice_fx.play_sound("listen")
        try:
            self.ai_recorder.start()
        except Exception as e:
            self.ai_response.setText(f"Mikrofon hatası: {e}")
            return
        self.ai_recording = True
        self._set_ai_local_busy("dinliyorum")
        self.ai_ptt.setText("●  Kaydediliyor… bırakınca gönderilir")
        self.ai_ptt.setProperty("active", True)
        self.ai_ptt.style().unpolish(self.ai_ptt)
        self.ai_ptt.style().polish(self.ai_ptt)

    def _on_ai_ptt_released(self):
        if not self.ai_recording:
            return
        self.ai_recording = False
        audio = self.ai_recorder.stop()
        if len(audio) > 0.5 * 16000:
            audio = audio[int(0.25 * 16000):]  # the first 0.25 s holds the listening cue
        ai_voice_fx.play_sound("send")
        self.ai_ptt.setProperty("active", False)
        self.ai_ptt.style().unpolish(self.ai_ptt)
        self.ai_ptt.style().polish(self.ai_ptt)
        if len(audio) and float(abs(audio).max()) < 1e-4:
            self._set_ai_local_busy(None)
            self.ai_ptt.setText("Basılı tut ve konuş  (S)")
            self.ai_response.setText("Mikrofon sessiz kaydetti — kapalı olabilir mi?")
            return
        self.ai_ptt.setText("…  Yazıya çevriliyor")
        self._set_ai_local_busy("yazıya çeviriyorum")

        def run():
            try:
                self._ai_transcribed.emit(self.ai_transcriber.transcribe(audio), "")
            except Exception as e:
                self._ai_transcribed.emit("", str(e))
        threading.Thread(target=run, daemon=True).start()

    def _on_ai_transcribed(self, text, error):
        self.ai_ptt.setText("Basılı tut ve konuş  (S)")
        self._set_ai_local_busy(None)
        if error:
            self.ai_response.setText(f"Konuşma tanıma hatası: {error}")
        elif not text:
            self.ai_response.setText("Ses algılanmadı, lütfen tekrar söyleyin.")
        else:
            self._ai_submit(text)

    def _on_ai_heard_stop(self, heard):
        self._on_stop()
        self.ai_response.setText(f"“{heard}” duyuldu — robot durduruldu.")
        self._log("error", f"Always-on listener heard “{heard}” → STOP")

    def _on_ai_listener_error(self, message):
        self.ai_stop_box.setChecked(False)
        self.ai_stop_box.setToolTip(f"Dinleyici çalışmadı: {message}")
        self._log("warn", f"Stop listener unavailable: {message}")

    def _on_ai_state(self, data):
        try:
            state = json.loads(data)
        except ValueError:
            return
        phase = state.get("phase", "idle")
        self.ai_stop_listener.set_speaking(bool(state.get("speaking")))
        pill = {"idle": ("YZ HAZIR", "ok"), "thinking": ("DÜŞÜNÜYOR", "info"),
                "awaiting_confirmation": ("ONAY?", "warn"), "executing": ("ÇALIŞIYOR", "error")}
        text, kind = pill.get(phase, (phase.upper(), "idle"))
        if not state.get("llm_ok", True) and phase == "idle":
            text, kind = "MODEL KAPALI", "warn"
        self.ai_pill.set_state(text, kind)
        self.ai_llm_label.setText(state.get("llm", ""))
        self.ai_plan.clear()
        plan = state.get("plan") or {}
        for step in plan.get("steps", []):
            tag = {"read": "OKUMA", "safe": "GÜVENLİ", "actuate": "EYLEM", "teach": "ÖĞRETME",
                   "motion": "HAREKET"}.get(step.get("kind"), "?")
            if step.get("ok"):
                item = QListWidgetItem(f"[{tag}] {step['command']} — {step['reason']}")
                item.setForeground(QColor(AI_KIND_COLORS.get(step.get("kind"), "#3a3a3c")))
            else:
                item = QListWidgetItem(f"✗ {step.get('command') or step.get('source')} — {step['reason']}")
                item.setForeground(QColor("#d70015"))
            self.ai_plan.addItem(item)
        chat = state.get("chat") or []
        if len(chat) != self._ai_chat_len or (chat and self.ai_chat.count() and
                                              not self.ai_chat.item(self.ai_chat.count() - 1).text().endswith(chat[-1]["text"])):
            self.ai_chat.clear()
            for line in chat:
                you = line["who"] == "you"
                item = QListWidgetItem(("Siz:  " if you else "Robot:  ") + line["text"])
                item.setForeground(QColor("#1d1d1f" if you else "#0062cc"))
                if you:
                    item.setTextAlignment(Qt.AlignRight)
                self.ai_chat.addItem(item)
            self._ai_chat_len = len(chat)
            self._ai_typing_item = None  # cleared with the list; re-added below if still busy
            self.ai_chat.scrollToBottom()
        if phase == "thinking":
            self._ai_node_busy = "düşünüyor"
        elif phase == "executing":
            step = re.match(r"(?:Adım|Step) (\d+)/(\d+)", state.get("response", ""))
            self._ai_node_busy = "hareket ediyor" + (f" · adım {step[1]}/{step[2]}" if step else "")
        else:
            self._ai_node_busy = None
        if self._ai_local_busy == "düşünüyor" and phase != "thinking" and chat and chat[-1]["who"] != "you":
            self._ai_local_busy = None  # the robot answered
        self._update_ai_typing()
        self.ai_response.setText(state.get("response", ""))
        self.ai_response.setProperty("kind", state.get("kind", "info"))
        self.ai_response.style().unpolish(self.ai_response)
        self.ai_response.style().polish(self.ai_response)
        self.ai_confirm_btn.setEnabled(phase == "awaiting_confirmation")
        self.ai_cancel_btn.setEnabled(phase in ("awaiting_confirmation", "executing", "thinking"))

    def _log(self, kind, text):
        item = QListWidgetItem(f"{datetime.now():%H:%M:%S}  {text}")
        item.setForeground(QColor(LOG_COLORS.get(kind, LOG_COLORS["info"])))
        self.log_list.addItem(item)
        while self.log_list.count() > LOG_LIMIT:
            self.log_list.takeItem(0)
        self.log_list.scrollToBottom()

    def _confirm(self, title, text):
        return QMessageBox.warning(self, title, text, QMessageBox.Yes | QMessageBox.No,
                                   QMessageBox.No) == QMessageBox.Yes

    def _call(self, service, request, what=None):
        what = what or service
        self.pending.add(what)
        self._refresh()
        self.ros.call(service, request, what)

    def _apply_step_size(self, size):
        for jc in self.joint_cards.values():
            jc.set_step_size(size)

    def _driver_alive(self):
        return self.robot is not None and time.monotonic() - self.last_status_time < STATUS_TIMEOUT_S

    def _selected_point(self):
        item = self.points_list.currentItem()
        return item.data(Qt.UserRole) if item is not None and item.isSelected() else None

    def _selected_teachpoint(self):
        """The full TeachPoint message for the selected point (speed, gripper, IO
        actions), or None if nothing is selected or the sequencer isn't connected yet."""
        name = self._selected_point()
        if name is None or self.seq is None:
            return None
        return next((p for p in self.seq.points if p.name == name), None)

    # -- state -> UI ----------------------------------------------------------------

    def _refresh(self):
        alive = self._driver_alive()
        r = self.robot if alive else None
        connected = bool(r and r.connected)
        servo = bool(connected and r.servo_on)
        moving = bool(connected and r.moving)
        looping = bool(self.seq and self.seq.loop_active)
        motion_pending = bool(self.pending & {"tcp move", "joint step", "servo on"})

        if not alive:
            self.link_pill.set_state("● NO DRIVER", "error" if self.robot else "idle")
        elif connected:
            self.link_pill.set_state("● CONNECTED", "ok")
        elif r.last_error:
            self.link_pill.set_state("● LINK ERROR", "error")
        else:
            self.link_pill.set_state("● OFFLINE", "idle")
        if not connected:
            self.servo_pill.set_state("SERVO —", "idle")
        else:
            self.servo_pill.set_state("SERVO ON" if servo else "SERVO OFF", "warn" if servo else "idle")
        if connected and r.alarm:
            self.alarm_pill.set_state("ALARM", "error")
        elif looping:
            self.alarm_pill.set_state(f"LOOP · CYCLE {self.seq.cycle}", "info")
        else:
            self.alarm_pill.set_state("NO ALARM", "idle")

        for w in (self.servo_on_btn, self.servo_off_btn, self.reset_alarm_btn, self.speed_slider):
            w.setEnabled(connected)
        for w in (self.run_btn, self.program_spin):
            w.setEnabled(connected and not looping and not moving)
        self.program_stop_btn.setEnabled(connected)
        # Pneumatic: needs a connection, not servo power.
        gripper_ready = connected and not (self.pending & {"gripper open", "gripper close"})
        self.gripper_open_btn.setEnabled(gripper_ready)
        self.gripper_close_btn.setEnabled(gripper_ready)
        state = {"open": "Open", "closed": "Closed"}.get(r.gripper if connected else "", "Unknown")
        if self.gripper_error:
            self.gripper_state.setText(f"Last commanded: {state}\nRejected: {self.gripper_error}")
            self.gripper_state.setObjectName("error")
        else:
            self.gripper_state.setText(f"Last commanded: {state}")
            self.gripper_state.setObjectName("dim")
        self.gripper_state.style().unpolish(self.gripper_state)
        self.gripper_state.style().polish(self.gripper_state)

        ready = servo and not moving and not looping and not motion_pending
        for jc in self.joint_cards.values():
            jc.set_steps_enabled(ready)
        for btn in self.tcp_buttons:
            btn.setEnabled(ready)

        have_points = bool(self.seq and self.seq.points)
        selected = self._selected_point() is not None
        self.save_point_btn.setEnabled(connected and not moving and not looping)
        self.insert_point_btn.setEnabled(connected and not moving and not looping and selected)
        self.slow_point_btn.setEnabled(selected and not looping)
        self.fast_point_btn.setEnabled(selected and not looping)
        self.io_record_box.setEnabled(selected)
        if not selected and self.io_record_box.isChecked():
            self.io_record_box.setChecked(False)
        point = self._selected_teachpoint()
        has_io = bool(point and point.io_outputs)
        self.clear_point_io_btn.setEnabled(has_io and not looping)
        if point is None:
            self.point_io_label.setText("")
        elif not point.io_outputs:
            self.point_io_label.setText(f"{point.name}: kayıtlı çıkış yok")
        else:
            actions = ", ".join(f"OUT{o}={int(v)}" for o, v in zip(point.io_outputs, point.io_values))
            self.point_io_label.setText(f"{point.name} vardığında: {actions}")
        self.goto_btn.setEnabled(ready and selected)
        self.home_btn.setEnabled(ready and "home" not in self.pending)
        self.delete_point_btn.setEnabled(selected and not looping)
        self.clear_points_btn.setEnabled(have_points and not looping)
        self.loop_start_btn.setEnabled(ready and self.seq is not None and len(self.seq.points) >= 2)
        self.loop_end_btn.setEnabled(looping)
        if looping:
            self.loop_status.setText(f"Looping · cycle {self.seq.cycle} · moving to {self.seq.current_point or '…'}")
        elif self.seq is None:
            self.loop_status.setText("Sequencer not running.")
        elif len(self.seq.points) < 2:
            self.loop_status.setText("Save at least 2 points to loop.")
        else:
            self.loop_status.setText(f"{len(self.seq.points)} points ready.")

        self.connect_btn.setEnabled(alive)
        self.port_combo.setEnabled(alive and not connected)
        self.connect_btn.setText("Disconnect" if connected else "Connect")
        self.connect_btn.setObjectName("" if connected else "primary")
        self.connect_btn.style().unpolish(self.connect_btn)
        self.connect_btn.style().polish(self.connect_btn)
        self.voice_btn.setEnabled(True)

    def _on_status(self, msg):
        first = self.robot is None
        was_connected = bool(self.robot and self.robot.connected)
        self.robot = msg
        self.last_status_time = time.monotonic()
        self._render_line(msg)
        if first:
            self._log("ok", "Driver found")
        if was_connected and not msg.connected and msg.last_error:
            self._log("error", f"Link dropped: {msg.last_error}")
        for i, name in enumerate(JOINT_NAMES):
            value = msg.joints_deg[i]
            self.joint_cards[name].set_value(None if math.isnan(value) else value)
        for key, value in (("X", msg.x_mm), ("Y", msg.y_mm), ("Z", msg.z_mm), ("A", msg.a_deg), ("B", msg.b_deg)):
            self.tool_cells[key].set_value(None if math.isnan(value) else value)
        self.state_raw.setText(msg.state_raw)
        self.moving_label.setText(f"moving: {msg.active_motion}" if msg.moving else "")
        if not self.speed_slider.isSliderDown() and msg.connected:
            self.speed_slider.setValue(msg.override_percent)
        self._refresh()

    def _on_insert_point(self):
        name = self._selected_point()
        if name:
            self._call("points/save", SavePoint.Request(after=name), f"insert after {name}")

    def _on_set_point_speed(self, percent):
        name = self._selected_point()
        if name:
            self._call("points/set_speed", SetPointSpeed.Request(name=name, percent=percent),
                       f"{name} speed {percent}%")

    def _on_clear_point_io(self):
        point = self._selected_teachpoint()
        if point is None:
            return
        for output in point.io_outputs:
            self._call("points/set_io", SetPointIO.Request(name=point.name, output=int(output), clear=True),
                       f"{point.name}: OUT{output} kaldırıldı")

    def _on_sequencer(self, msg):
        selected = self._selected_point()
        self.seq = msg
        self.points_list.clear()
        for p in msg.points:
            angles = "".join(f"{p.joints_deg[i]:>7.1f}" for i, j in enumerate(JOINT_NAMES) if j in MOVABLE_JOINTS)
            grip = {"open": "open", "closed": "close"}.get(p.gripper, "  -")
            item = QListWidgetItem(f"{p.name:<4}{angles}   {grip} {p.speed_percent:4d}%")
            item.setData(Qt.UserRole, p.name)
            self.points_list.addItem(item)
            if p.name == selected or (msg.loop_active and p.name == msg.current_point):
                item.setSelected(True)
                self.points_list.setCurrentItem(item)
        self._refresh()

    def _on_done(self, what, success, message):
        self.pending.discard(what)
        if what in ("gripper open", "gripper close"):
            self.gripper_error = "" if success else message
        if not success:
            self._log("warn" if not what.startswith("gripper") else "error", f"{what}: {message}")
        self._refresh()

    def _on_gripper(self, open_):
        self.gripper_error = ""
        self._call("gripper/open" if open_ else "gripper/close", Trigger.Request(),
                   "gripper open" if open_ else "gripper close")

    # -- actions ------------------------------------------------------------------

    def _on_connect_toggled(self):
        if self.robot and self.robot.connected:
            self._call("disconnect", Trigger.Request())
        else:
            self._call("connect", Connect.Request(port=self.port_combo.currentText().strip()))

    def _on_servo_on(self):
        if not self._srvon_confirmed:
            if not self._confirm("Confirm servo on", "Robot will move when jogged. Clear the area."):
                return
            self._srvon_confirmed = True
        self._call("servo", SetBool.Request(data=True), "servo on")

    def _on_stop(self):
        self.ros.call("stop", Trigger.Request(), "stop")
        self.ros.call("ai/cancel", Trigger.Request(), "AI cancel")
        if self.seq and self.seq.loop_active:
            self.ros.call("loop/stop", Trigger.Request(), "end loop")
        self._log("error", "STOP pressed")

    def _on_joint_step(self, joint, offset):
        if joint in MISCALIBRATED_JOINTS and not self._j3_confirmed:
            if not self._confirm(f"Confirm {joint} move",
                                 f"{joint}'s origin is miscalibrated on this robot, so its angle reading "
                                 f"and soft limits may be wrong. Step {joint} anyway?"):
                return
            self._j3_confirmed = True
        goal = JointMove.Goal(relative=True, joint=joint, offset_deg=float(offset))
        self.pending.add("joint step")
        self._refresh()
        self.ros.send_goal(self.ros.joint_action, goal, "joint step")

    def _on_tcp_move(self, axis, distance, direction):
        goal = TcpMove.Goal(axis=axis, distance_mm=float(distance))
        self.pending.add("tcp move")
        self._refresh()
        self._log("info", f"TCP {direction} {abs(distance):g} mm [button]")
        self.ros.send_goal(self.ros.tcp_action, goal, "tcp move")

    def _on_home(self):
        if not self._goto_confirmed:
            if not self._confirm("Confirm move home",
                                 "The robot moves every joint straight to the home pose. "
                                 "This can be a large move. Clear the area."):
                return
            self._goto_confirmed = True
        self._call("home/goto", Trigger.Request(), "home")

    def _on_goto(self):
        name = self._selected_point()
        if name is None:
            return
        if not self._goto_confirmed:
            if not self._confirm("Confirm move to point",
                                 "The robot moves every joint straight to the saved angles. "
                                 "This can be a large move. Clear the area."):
                return
            self._goto_confirmed = True
        self._call("points/goto", PointName.Request(name=name), "go to")

    def _on_delete_point(self):
        name = self._selected_point()
        if name is not None:
            self._call("points/delete", PointName.Request(name=name), "delete point")

    def _on_clear_points(self):
        if self.seq and self._confirm("Clear points", f"Delete all {len(self.seq.points)} saved points?"):
            self._call("points/clear", Trigger.Request(), "clear points")

    def _on_loop_start(self):
        if not self.seq or len(self.seq.points) < 2:
            return
        names = " → ".join(p.name for p in self.seq.points)
        if not self._confirm(
            "Start loop",
            f"The robot will move continuously through {len(self.seq.points)} points ({names} → "
            f"{self.seq.points[0].name} …) at {self.speed_slider.value()}% override until you press "
            f"End loop.\n\nMoves between points can be large. Clear the area.",
        ):
            return
        self._goto_confirmed = True
        self._call("loop/start", Trigger.Request(), "start loop")

    # -- voice --------------------------------------------------------------------

    def _on_voice_toggled(self):
        want_on = self.voice_btn.isChecked()
        if want_on and not self._voice_confirmed:
            if not self._confirm(
                "Enable voice control",
                "Spoken commands will move the robot while the servo is on.\n\n"
                "Saying “stop” stops it, but recognition takes 1–2 seconds. Use the red STOP "
                "button or Esc for anything urgent. Clear the area.",
            ):
                self.voice_btn.setChecked(False)
                return
            self._voice_confirmed = True
        self.voice_btn.setChecked(self.voice_state not in ("off", "error"))
        self.ros.call("voice/enable", SetBool.Request(data=want_on), "voice")

    def _on_voice_status(self, state):
        self.voice_state = state
        on = state not in ("off", "error")
        self.voice_btn.setChecked(on)
        self.voice_btn.setText("●  Voice on" if on else "●  Voice off")
        self.voice_btn.setProperty("active", on)
        self.voice_btn.style().unpolish(self.voice_btn)
        self.voice_btn.style().polish(self.voice_btn)
        self.voice_state_label.setText(f"voice node: {state}")

    def _on_voice_text(self, text):
        self.voice_heard.setText(f"Heard: “{text}”")

    def _on_voice_command(self, cmd):
        if cmd.kind == MotionCommand.KIND_NONE:
            meaning = f"ignored ({cmd.reason})"
        elif cmd.kind == MotionCommand.KIND_TCP:
            where = "along arm" if cmd.axis == "R" else cmd.axis
            meaning = f"TCP {where} {cmd.distance_mm:+g} mm"
        elif cmd.kind == MotionCommand.KIND_GOTO_POINT:
            meaning = f"go to {cmd.point}"
        else:
            meaning = cmd.kind.replace("_", " ")
        self.voice_heard.setText(f"Heard: “{cmd.text}”  →  {meaning}")

    def closeEvent(self, event):
        self.watchdog.stop()
        self.ai_stop_listener.stop()
        super().closeEvent(event)


def main(args=None):
    import faulthandler
    faulthandler.enable()  # a native crash prints where it happened instead of dying silently
    rclpy.init(args=args)
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    app = QApplication(sys.argv[:1])
    app.setStyle("Fusion")
    app.setStyleSheet(THEME_QSS)
    bridge = RosBridge()
    window = MainWindow(bridge)
    window.show()
    code = app.exec_()
    bridge.shutdown()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
