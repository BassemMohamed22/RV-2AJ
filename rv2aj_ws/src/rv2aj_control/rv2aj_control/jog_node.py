"""RViz arrow jog: an interactive marker at the TCP with X/Y/Z arrows.

Drag an arrow and release: the TCP makes one straight, MoveIt-validated move
(planner planned/tcp_move, axis "V") to where the marker was dropped, then
the marker snaps back to the robot's real TCP. Nothing moves while dragging,
and a move over max_move_mm is refused, not clamped. Arrows are in the robot
base frame; the servo must be on.

RViz: add an "InteractiveMarkers" display with namespace /rv2aj/jog (the
provided moveit.rviz already has it).
"""

import math

import rclpy
from geometry_msgs.msg import Point, Pose
from interactive_markers import InteractiveMarkerServer
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
)

from rv2aj_interfaces.action import TcpMove
from rv2aj_interfaces.msg import RobotStatus

MARKER = "tcp_jog"
MIN_MOVE_MM = 1.0


def _arrow(direction, rgb):
    """Arrow along `direction` (unit vector in the interactive marker's frame:
    custom control markers are drawn in the marker frame, not the control's)."""
    a = Marker()
    a.type = Marker.ARROW
    dx, dy, dz = direction
    a.points = [Point(x=0.03 * dx, y=0.03 * dy, z=0.03 * dz), Point(x=0.13 * dx, y=0.13 * dy, z=0.13 * dz)]
    a.scale.x, a.scale.y, a.scale.z = 0.012, 0.028, 0.035   # shaft, head width, head length
    a.color.r, a.color.g, a.color.b = rgb
    a.color.a = 0.95
    return a


def _axis_control(name, x, y, z, rgb, axis):
    c = InteractiveMarkerControl()
    c.name = name
    norm = math.sqrt(1 + x * x + y * y + z * z)
    c.orientation.w, c.orientation.x, c.orientation.y, c.orientation.z = 1 / norm, x / norm, y / norm, z / norm
    c.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    c.always_visible = True
    c.markers.append(_arrow(axis, rgb))
    c.markers.append(_arrow(tuple(-v for v in axis), rgb))
    return c


class JogNode(Node):
    def __init__(self):
        super().__init__("jog")
        self.frame = self.declare_parameter("frame_id", "base_link").value
        self.max_move_mm = float(self.declare_parameter("max_move_mm", 50.0).value)
        self.robot = None
        self.dragging = False
        self.busy = False
        self.start_mm = None

        self.create_subscription(RobotStatus, "status", self._on_status, 10)
        self.tcp = ActionClient(self, TcpMove, "planned/tcp_move")
        self.server = InteractiveMarkerServer(self, "jog")
        self.server.insert(self._marker(Pose()), feedback_callback=self._on_feedback)
        self.server.applyChanges()
        self.create_timer(0.3, self._follow_robot)

    def _marker(self, pose):
        m = InteractiveMarker()
        m.header.frame_id = self.frame
        m.name = MARKER
        m.description = "TCP jog: drag an arrow, release to move"
        m.scale = 0.25   # arrows stick well out of the arm
        m.pose = pose
        ball = Marker()
        ball.type = Marker.SPHERE
        ball.scale.x = ball.scale.y = ball.scale.z = 0.035
        ball.color.r, ball.color.g, ball.color.b, ball.color.a = 0.0, 0.48, 1.0, 0.9
        visual = InteractiveMarkerControl()
        visual.always_visible = True
        visual.markers.append(ball)
        m.controls.append(visual)
        m.controls.append(_axis_control("move_x", 1, 0, 0, (0.9, 0.2, 0.2), (1, 0, 0)))   # red   = X
        m.controls.append(_axis_control("move_y", 0, 0, 1, (0.2, 0.75, 0.3), (0, 1, 0)))  # green = Y
        m.controls.append(_axis_control("move_z", 0, 1, 0, (0.2, 0.45, 1.0), (0, 0, 1)))  # blue  = Z
        return m

    def _tcp_pose(self):
        r = self.robot
        if r is None or not r.connected or any(math.isnan(v) for v in (r.x_mm, r.y_mm, r.z_mm)):
            return None
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = r.x_mm / 1000, r.y_mm / 1000, r.z_mm / 1000
        pose.orientation.w = 1.0
        return pose

    def _on_status(self, msg):
        self.robot = msg

    def _follow_robot(self):
        if self.dragging or self.busy:
            return
        pose = self._tcp_pose()
        if pose is not None:
            self.server.setPose(MARKER, pose)
            self.server.applyChanges()

    def _on_feedback(self, fb):
        if fb.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
            pose = self._tcp_pose()
            self.dragging = pose is not None and not self.busy
            if self.dragging:
                self.start_mm = [pose.position.x * 1000, pose.position.y * 1000, pose.position.z * 1000]
        elif fb.event_type == InteractiveMarkerFeedback.MOUSE_UP and self.dragging:
            self.dragging = False
            p = fb.pose.position
            delta = [p.x * 1000 - self.start_mm[0], p.y * 1000 - self.start_mm[1], p.z * 1000 - self.start_mm[2]]
            self._jog(delta)

    def _jog(self, delta):
        length = math.sqrt(sum(d * d for d in delta))
        if length < MIN_MOVE_MM:
            return
        if length > self.max_move_mm:
            self.get_logger().warning(f"Jog of {length:.0f} mm refused: drag at most {self.max_move_mm:g} mm per move")
            return
        r = self.robot
        if r is None or not r.servo_on:
            self.get_logger().warning("Jog refused: servo is off")
            return
        if not self.tcp.server_is_ready():
            self.get_logger().error("planned/tcp_move is not available (is the planner running?)")
            return
        goal = TcpMove.Goal()
        goal.axis = "V"
        goal.vector_mm = [float(d) for d in delta]
        self.busy = True
        self.get_logger().info("Jog (" + ", ".join(f"{d:+.1f}" for d in delta) + ") mm")
        self.tcp.send_goal_async(goal).add_done_callback(self._accepted)

    def _accepted(self, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self.busy = False
            self.get_logger().warning("Jog not accepted (another move is running?)")
            return
        handle.get_result_async().add_done_callback(self._finished)

    def _finished(self, future):
        self.busy = False
        result = future.result().result
        if not result.success:
            self.get_logger().warning(f"Jog: {result.message}")


def main(args=None):
    rclpy.init(args=args)
    node = JogNode()
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
