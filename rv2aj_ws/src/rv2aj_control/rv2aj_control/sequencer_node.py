"""Teach points and loop playback, on top of a joint_move action: the
planner's planned/joint_move (MoveIt-validated, default) or the driver's.

Services: points/save, points/delete, points/clear, points/goto,
          loop/start, loop/stop, home/goto (home_joints_deg)
Topic:    sequencer/status (latched: points, loop state)

Points are stored as JSON ([{"name": "P1", "joints": [J1..J6 deg]}]),
the same format as the standalone GUI's points.json.
The loop moves P1 -> P2 -> ... -> P1 ... until loop/stop, and ends by itself
if any move fails or is stopped (driver STOP aborts the active goal).
"""

import itertools
import json
import math
import os
import shutil
import string
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_srvs.srv import Trigger

from rv2aj_interfaces.action import JointMove
from rv2aj_interfaces.msg import RobotStatus, SequencerStatus, TeachPoint
from rv2aj_interfaces.srv import LineSignal, PointName, SavePoint, SetOverride, SetPointIO, SetPointSpeed

MOVABLE = (0, 1, 2, 4, 5)  # J4 is not fitted
DEFAULT_POINT_SPEED_PERCENT = 20   # points saved before this feature existed get this speed
VALID_OUTPUTS = tuple(range(16)) + tuple(range(900, 908))  # matches the driver's WRITABLE_OUTPUTS
def _suffixes():
    """A, B, ... Z, then AA, AB, ... ZZ, then AAA ...: letters for inserted points never run out."""
    n = 1
    while True:
        for combo in itertools.product(string.ascii_uppercase, repeat=n):
            yield "".join(combo)
        n += 1


_PENDING = object()  # goal sent, not yet accepted (goal handles override ==, so compare with is)


def _clamp_percent(value, default):
    """An int 1-100, or `default` if `value` is missing/not a usable number (an
    older points.json without speed_percent, or a corrupt value)."""
    try:
        value = int(value)
    except (TypeError, ValueError):
        return default
    return value if 1 <= value <= 100 else default


def _clean_io(raw):
    """A list of [output, value] pairs from a points.json entry, dropping anything
    malformed (a missing/corrupt field, or an older file with no "io" at all)."""
    out = []
    for pair in raw or []:
        try:
            output, value = int(pair[0]), bool(pair[1])
        except (TypeError, ValueError, IndexError):
            continue
        if output in VALID_OUTPUTS:
            out.append([output, value])
    return out


class SequencerNode(Node):
    def __init__(self):
        super().__init__("sequencer")
        path = self.declare_parameter("points_file", "~/.ros/rv2aj_points.json").value
        joint_action = self.declare_parameter("joint_action", "planned/joint_move").value
        self.gripper_wait_s = float(self.declare_parameter("gripper_wait_s", 0.6).value)
        self.home = [float(v) for v in self.declare_parameter(
            "home_joints_deg", [-118.49, -101.71, 186.64, 0.00, -127.75, -279.42]).value]
        # Home is a transit point like any other for the slowest-of-two speed rule;
        # it has no taught speed of its own, so it gets this fixed one.
        self.home_speed_percent = int(self.declare_parameter("home_speed_percent", 40).value)
        self.points_file = os.path.expanduser(path)
        self.points = self._load()
        self.robot = None
        self.goal_handle = None     # None, _PENDING or a ClientGoalHandle
        self.cancel_pending = False
        self.goal_point = ""
        # The point we last actually arrived at (name), or None if unknown (never
        # reached one yet, or the last move failed/stopped short). Used only to look
        # up its speed for the slowest-of-two rule on the NEXT move; "HOME" is valid.
        self.last_reached_point = None
        self.loop_active = False
        self.loop_index = 0
        self.cycle = 0

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(SequencerStatus, "sequencer/status", latched)
        self.create_subscription(RobotStatus, "status", self._on_robot, 10)
        self.move_client = ActionClient(self, JointMove, joint_action)
        self.override_client = self.create_client(SetOverride, "set_override")
        self.line_client = self.create_client(LineSignal, "line/set")
        self.grip_clients = {"open": self.create_client(Trigger, "gripper/open"),
                             "closed": self.create_client(Trigger, "gripper/close")}

        self.create_service(SavePoint, "points/save", self._save)
        self.create_service(PointName, "points/delete", self._delete)
        self.create_service(Trigger, "points/clear", self._clear)
        self.create_service(PointName, "points/goto", self._goto)
        self.create_service(SetPointSpeed, "points/set_speed", self._set_speed)
        self.create_service(SetPointIO, "points/set_io", self._set_io)
        self.create_service(Trigger, "loop/start", self._loop_start)
        self.create_service(Trigger, "loop/stop", self._loop_stop)
        self.create_service(Trigger, "home/goto", self._goto_home)
        self.get_logger().info(f"{len(self.points)} points loaded from {self.points_file}")
        self._publish()

    # -- storage -------------------------------------------------------------

    def _load(self):
        try:
            with open(self.points_file) as f:
                data = json.load(f)
            return [{"name": str(p["name"]), "joints": [float(v) for v in p["joints"][:6]],
                     "gripper": p.get("gripper") if p.get("gripper") in ("open", "closed") else None,
                     "speed_percent": _clamp_percent(p.get("speed_percent"), DEFAULT_POINT_SPEED_PERCENT),
                     "io": _clean_io(p.get("io"))}
                    for p in data if len(p.get("joints", [])) >= 6]
        except FileNotFoundError:
            return []
        except (OSError, ValueError, KeyError, TypeError) as e:
            self.get_logger().error(f"Could not read {self.points_file}: {e}")
            return []

    def _persist(self):
        os.makedirs(os.path.dirname(self.points_file) or ".", exist_ok=True)
        # Points are kept always: the file being replaced is first copied to
        # <name>_history/<name>_<time>.json, never deleted (operator's choice
        # 2026-10-19). Delete / Clear / an unreadable file stay recoverable.
        if os.path.exists(self.points_file):
            stem = os.path.splitext(self.points_file)[0]
            history = stem + "_history"
            os.makedirs(history, exist_ok=True)
            stamp = time.strftime("%Y-%m-%d_%H%M%S") + f"_{time.time_ns() % 1_000_000_000:09d}"
            shutil.copy2(self.points_file, os.path.join(history, f"{os.path.basename(stem)}_{stamp}.json"))
        tmp = self.points_file + ".tmp"
        with open(tmp, "w") as f:
            json.dump(self.points, f, indent=2)
        os.replace(tmp, self.points_file)

    def _publish(self):
        msg = SequencerStatus()
        for p in self.points:
            tp = TeachPoint()
            tp.name = p["name"]
            tp.joints_deg = [float(v) for v in p["joints"]]
            tp.gripper = p.get("gripper") or ""
            tp.speed_percent = int(p["speed_percent"])
            tp.io_outputs = [int(o) for o, _ in p.get("io", [])]
            tp.io_values = [bool(v) for _, v in p.get("io", [])]
            msg.points.append(tp)
        msg.loop_active = self.loop_active
        msg.cycle = self.cycle
        msg.current_point = self.goal_point
        self.status_pub.publish(msg)

    def _find(self, name):
        name = name.strip().upper()
        return next((i for i, p in enumerate(self.points) if p["name"].upper() == name), None)

    # -- robot status ---------------------------------------------------------

    def _on_robot(self, msg):
        self.robot = msg

    # -- services ---------------------------------------------------------------

    def _save(self, request, response):
        r = self.robot
        if r is None or not r.connected:
            response.success, response.message = False, "robot not connected"
            return response
        if r.moving or self.loop_active:
            response.success, response.message = False, "wait until the robot has stopped"
            return response
        if any(math.isnan(r.joints_deg[i]) for i in MOVABLE):
            response.success, response.message = False, "joint angles not known yet"
            return response
        name = request.name.strip().upper()
        after = request.after.strip().upper()
        insert_at = None
        if after:
            ai = self._find(after)
            if ai is None:
                response.success, response.message = False, f"no point named {after}"
                return response
            insert_at = ai + 1
            if name and self._find(name) is not None:
                response.success, response.message = False, f"{name} already exists"
                return response
            if not name:
                # P1 -> P1A, P1B ...; inserting after P1A also continues P1's letters.
                stem = after.rstrip("ABCDEFGHIJKLMNOPQRSTUVWXYZ") if after[1:].rstrip(
                    "ABCDEFGHIJKLMNOPQRSTUVWXYZ").isdigit() else after
                letter = next(c for c in _suffixes() if self._find(stem + c) is None)
                name = stem + letter
        if not name:
            used = [int(p["name"][1:]) for p in self.points if p["name"][1:].isdigit()]
            name = f"P{max(used, default=0) + 1}"
        joints = [0.0 if i not in MOVABLE else round(r.joints_deg[i], 2) for i in range(6)]
        # The gripper's last commanded state is saved with the point and re-applied
        # after the robot reaches it (unknown = leave the gripper alone).
        gripper = r.gripper if r.gripper in ("open", "closed") else None
        index = self._find(name)
        requested_speed = _clamp_percent(request.speed_percent, 0)  # 0 = not given
        if index is None:
            # A new point with no speed requested inherits whatever override the
            # operator was jogging at, so it remembers the speed it was taught at.
            speed = requested_speed or _clamp_percent(r.override_percent, DEFAULT_POINT_SPEED_PERCENT)
        else:
            # Re-teaching an existing point's position keeps its speed unless asked.
            speed = requested_speed or self.points[index]["speed_percent"]
        # Re-teaching an existing point's position keeps its saved IO actions too
        # (only points/set_io changes those); a brand-new point starts with none.
        io = list(self.points[index]["io"]) if index is not None else []
        point = {"name": name, "joints": joints, "gripper": gripper, "speed_percent": speed, "io": io}
        if insert_at is not None:
            self.points.insert(insert_at, point)
        elif index is None:
            self.points.append(point)
        else:
            self.points[index] = point
        self._persist()
        self._publish()
        self.get_logger().info(f"Saved {name}: " + ", ".join(f"J{i + 1} {joints[i]:.2f}" for i in MOVABLE)
                               + f", gripper {gripper or 'unchanged'}, speed {speed}%")
        where = f" after {after}" if insert_at is not None else ""
        response.success, response.message, response.name = True, f"saved {name}{where} at {speed}%", name
        return response

    def _set_speed(self, request, response):
        index = self._find(request.name)
        percent = _clamp_percent(request.percent, 0)
        if index is None:
            response.success, response.message = False, f"no point named {request.name!r}"
        elif not percent:
            response.success, response.message = False, "percent must be 1-100"
        else:
            self.points[index]["speed_percent"] = percent
            self._persist()
            self._publish()
            name = self.points[index]["name"]
            self.get_logger().info(f"{name}: speed set to {percent}%")
            response.success, response.message, response.percent = True, f"{name} speed {percent}%", percent
        return response

    def _set_io(self, request, response):
        index = self._find(request.name)
        if index is None:
            response.success, response.message = False, f"no point named {request.name!r}"
            return response
        if request.output not in VALID_OUTPUTS:
            response.success, response.message = False, "output must be 0-15 or 900-907"
            return response
        name = self.points[index]["name"]
        io = self.points[index]["io"]
        io[:] = [pair for pair in io if pair[0] != request.output]  # drop any existing action on this output
        if request.clear:
            self._persist()
            self._publish()
            self.get_logger().info(f"{name}: OUT{request.output} action removed")
            response.success, response.message = True, f"{name}: OUT{request.output} removed"
            return response
        io.append([request.output, bool(request.value)])
        self._persist()
        self._publish()
        self.get_logger().info(f"{name}: will set OUT{request.output} = {int(request.value)} on arrival")
        response.success, response.message = True, f"{name}: OUT{request.output} = {int(request.value)}"
        return response

    def _delete(self, request, response):
        index = self._find(request.name)
        if index is None:
            response.success, response.message = False, f"no point named {request.name!r}"
        elif self.loop_active:
            response.success, response.message = False, "stop the loop first"
        else:
            removed = self.points.pop(index)
            self._persist()
            self._publish()
            self.get_logger().info(f"Deleted {removed['name']}")
            response.success, response.message = True, f"deleted {removed['name']}"
        return response

    def _clear(self, request, response):
        if self.loop_active:
            response.success, response.message = False, "stop the loop first"
            return response
        count = len(self.points)
        self.points = []
        self._persist()
        self._publish()
        self.get_logger().info(f"Cleared {count} points")
        response.success, response.message = True, f"cleared {count} points"
        return response

    def _goto(self, request, response):
        index = self._find(request.name)
        if index is None:
            response.success, response.message = False, f"no point named {request.name!r}"
        elif self.loop_active:
            response.success, response.message = False, "a loop is running"
        else:
            response.success, response.message = self._send_point(index)
        return response

    def _goto_home(self, request, response):
        if self.loop_active:
            response.success, response.message = False, "a loop is running"
        elif len(self.home) != 6:
            response.success, response.message = False, "home_joints_deg must have 6 values"
        else:
            response.success, response.message = self._send_joints(self.home, "HOME")
        return response

    def _loop_start(self, request, response):
        if self.loop_active:
            response.success, response.message = False, "loop already running"
        elif len(self.points) < 2:
            response.success, response.message = False, "save at least 2 points first"
        else:
            self.loop_active = True
            self.loop_index = 0
            self.cycle = 1
            ok, message = self._send_point(0)
            if not ok:
                self._end_loop(message)
            else:
                names = " → ".join(p["name"] for p in self.points)
                self.get_logger().warning(f"Loop started: {names}")
            response.success, response.message = ok, message
        return response

    def _loop_stop(self, request, response):
        was_active = self.loop_active
        self._end_loop("loop/stop called")
        if self.goal_handle is _PENDING:
            self.cancel_pending = True
        elif self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()   # driver sends STOP on cancel
        response.success = True
        response.message = "loop stopped" if was_active else "no loop was running"
        return response

    # -- motion -------------------------------------------------------------------

    def _point_speed(self, name):
        """Speed percent for a named point, HOME's fixed speed, or None if unknown."""
        if name == "HOME":
            return self.home_speed_percent
        index = self._find(name)
        return self.points[index]["speed_percent"] if index is not None else None

    def _transition_override(self, dest_name):
        """Slowest-of-two (operator's choice): a move uses the slower of the point
        it leaves and the point it arrives at, so marking a grab point slow also
        slows the approach and the retreat. Only the destination's own speed is
        used when we don't know where we're coming from (nothing reached yet, or
        the previous move didn't finish cleanly)."""
        dest_speed = self._point_speed(dest_name) or DEFAULT_POINT_SPEED_PERCENT
        if self.last_reached_point is None:
            return dest_speed
        from_speed = self._point_speed(self.last_reached_point)
        return min(dest_speed, from_speed) if from_speed is not None else dest_speed

    def _send_point(self, index):
        point = self.points[index]
        return self._send_joints(point["joints"], point["name"])

    def _send_joints(self, joints, name):
        if self.goal_handle is not None:
            return False, "a move is already in progress"
        if not self.move_client.server_is_ready():
            return False, f"{self.move_client._action_name} not available (is the stack running?)"
        if not self.override_client.service_is_ready():
            return False, "set_override not available (is the stack running?)"
        self.goal_point = name
        self.goal_handle = _PENDING
        self.cancel_pending = False
        self._publish()
        percent = self._transition_override(name)
        self.get_logger().info(f"Moving to {name} at {percent}%")
        # The speed must be set on the driver before the move is sent (EXECJOVRD
        # takes effect on the next EXECMOV); the goal is sent once that's confirmed.
        # But skip that round trip entirely when the speed is already what we want
        # (the controller's current override, authoritative in RobotStatus): most
        # consecutive points share a speed, and re-sending it just adds a serial
        # round trip of standing still (operator wants continuous motion).
        current = self.robot.override_percent if self.robot is not None else None
        if percent == current:
            self._send_move_goal(joints)
        else:
            self.override_client.call_async(SetOverride.Request(percent=percent)).add_done_callback(
                lambda f, j=joints: self._override_set(f, j))
        return True, f"moving to {name}"

    def _override_set(self, future, joints):
        if self.cancel_pending:
            self.cancel_pending = False
            self.goal_handle = None
            self._move_finished(False, "cancelled")
            return
        response = future.result()
        if response is None or not response.success:
            self.goal_handle = None
            reason = response.message if response is not None else "no response"
            self._move_finished(False, f"could not set speed: {reason}")
            return
        self._send_move_goal(joints)

    def _send_move_goal(self, joints):
        goal = JointMove.Goal()
        goal.relative = False
        goal.target_deg = [float(v) for v in joints]
        self.move_client.send_goal_async(goal).add_done_callback(self._goal_response)

    def _goal_response(self, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self.goal_handle = None
            self._move_finished(False, "goal rejected")
            return
        self.goal_handle = handle
        if self.cancel_pending:
            self.cancel_pending = False
            handle.cancel_goal_async()
        handle.get_result_async().add_done_callback(self._goal_result)

    def _goal_result(self, future):
        result = future.result().result
        self.goal_handle = None
        self._move_finished(result.success, result.message)

    def _move_finished(self, success, message):
        name = self.goal_point
        self.goal_point = ""
        if success:
            self.last_reached_point = name   # for the slowest-of-two rule on the next move
            self.get_logger().info(f"Reached {name}")
            self._run_arrival_actions(name)
            return
        self.last_reached_point = None   # stopped short / cancelled: position no longer known
        self.get_logger().warning(f"Move to {name} failed: {message}")
        self._after_point(False, name)

    def _run_arrival_actions(self, name):
        """After reaching a point: its gripper command (if any), then each of its
        saved IO actions in order, each waited out before the next — only once all
        of them are done does the loop continue / the goto report finished. Any one
        of them failing stops here, exactly like a move that failed."""
        index = self._find(name)
        point = self.points[index] if index is not None else None
        grip = point.get("gripper") if point else None
        io_actions = list(point.get("io", [])) if point else []
        self._publish()  # the move itself is done; don't wait for these to show that

        def next_io():
            if not io_actions:
                self._after_point(True, name)
                return
            output, value = io_actions.pop(0)
            self._set_output(output, value, name, next_io)

        # Only actuate the gripper when its state actually changes. Re-commanding
        # it to where it already is just costs a command + the gripper_wait_s pause
        # with no physical effect, which made every transit point stand still for
        # ~0.6 s (operator wants continuous motion, 2026-10-20). The driver's
        # last-commanded gripper state is authoritative (it's the only writer).
        current_grip = self.robot.gripper if self.robot is not None else None
        if grip and grip != current_grip:
            self._actuate_gripper(grip, name, next_io)
        else:
            next_io()

    def _actuate_gripper(self, grip, name, then):
        client = self.grip_clients[grip]
        if not client.service_is_ready():
            self.get_logger().error(f"{name}: gripper service not available")
            self._after_point(False, name)
            return
        self.get_logger().info(f"{name}: gripper {grip}")

        def done(future):
            response = future.result()
            ok = response is not None and response.success
            if not ok:
                self.get_logger().warning(f"{name}: gripper {grip} failed: "
                                          f"{response.message if response else 'no response'}")
                self._after_point(False, name)
                return
            # Give the pneumatic jaws time to finish before the next action.
            self._after_delay(self.gripper_wait_s, then)

        client.call_async(Trigger.Request()).add_done_callback(done)

    def _set_output(self, output, value, name, then):
        if not self.line_client.service_is_ready():
            self.get_logger().error(f"{name}: line/set not available")
            self._after_point(False, name)
            return
        self.get_logger().info(f"{name}: OUT{output} = {int(value)}")

        def done(future):
            response = future.result()
            ok = response is not None and response.success
            if not ok:
                self.get_logger().warning(f"{name}: OUT{output} failed: "
                                          f"{response.message if response else 'no response'}")
                self._after_point(False, name)
                return
            then()

        self.line_client.call_async(LineSignal.Request(signal=f"out{output}", value=int(value))
                                    ).add_done_callback(done)

    def _after_delay(self, seconds, fn):
        timer = None

        def fire():
            timer.cancel()
            self.destroy_timer(timer)
            fn()

        timer = self.create_timer(seconds, fire)

    def _after_point(self, success, name):
        if self.loop_active:
            if not success:
                self._end_loop(f"move to {name} failed")
            else:
                self.loop_index += 1
                if self.loop_index >= len(self.points):
                    self.loop_index = 0
                    self.cycle += 1
                    self.get_logger().info(f"Cycle {self.cycle}")
                ok, why = self._send_point(self.loop_index)
                if not ok:
                    self._end_loop(why)
        self._publish()

    def _end_loop(self, reason):
        if self.loop_active:
            self.get_logger().warning(f"Loop ended during cycle {self.cycle}: {reason}")
        self.loop_active = False
        self._publish()


def main(args=None):
    rclpy.init(args=args)
    node = SequencerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
