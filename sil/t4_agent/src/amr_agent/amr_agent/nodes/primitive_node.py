"""Primitive controller node — MovePrimitive action server → /cmd_vel.

Pose source is ground-truth odom by default (Isaac publishes it); set pose_source:=amcl
to close the loop on AMCL instead. One goal at a time; a new goal while busy is rejected.
"""
from __future__ import annotations

import math
import threading
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from amr_agent_msgs.action import MovePrimitive

from ..control.primitives import DriveController, Gains, Limits, TurnController, wrap


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class PrimitiveController(Node):
    def __init__(self) -> None:
        super().__init__("primitive_controller")
        p = self.declare_parameters("", [
            ("pose_source", "odom"), ("rate_hz", 20.0),
            ("tol_xy", 0.05), ("tol_theta", 0.03), ("timeout_s", 30.0),
            ("v_max", 0.8), ("w_max", 1.2), ("a_max", 1.0), ("alpha_max", 2.5),
            ("v_min", 0.05), ("w_min", 0.12),
            ("k_turn", 2.5), ("k_heading", 2.5), ("k_lateral", 1.5),
        ])
        g = {x.name: x.value for x in p}
        self.pose_source = g["pose_source"]
        self.dt = 1.0 / float(g["rate_hz"])
        self.tol_xy, self.tol_theta, self.timeout = g["tol_xy"], g["tol_theta"], g["timeout_s"]
        self.limits = Limits(g["v_max"], g["w_max"], g["a_max"], g["alpha_max"], g["v_min"], g["w_min"])
        self.gains = Gains(g["k_turn"], g["k_heading"], g["k_lateral"])

        self._pose = None
        self._lock = threading.Lock()
        self._busy = False
        cb = ReentrantCallbackGroup()
        self.create_subscription(Odometry, "odom", self._on_odom, 20, callback_group=cb)
        qos_tl = QoSProfile(depth=5)
        qos_tl.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(PoseWithCovarianceStamped, "amcl_pose", self._on_amcl, qos_tl,
                                 callback_group=cb)
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self._server = ActionServer(self, MovePrimitive, "move_primitive", self._execute,
                                    goal_callback=self._on_goal, cancel_callback=self._on_cancel,
                                    callback_group=cb)
        self.get_logger().info(f"primitive controller up — pose_source={self.pose_source}")

    # ---- pose ----------------------------------------------------------------
    def _on_odom(self, m: Odometry) -> None:
        if self.pose_source == "odom":
            p = m.pose.pose
            with self._lock:
                self._pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def _on_amcl(self, m: PoseWithCovarianceStamped) -> None:
        if self.pose_source == "amcl":
            p = m.pose.pose
            with self._lock:
                self._pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def pose(self):
        with self._lock:
            return self._pose

    # ---- action --------------------------------------------------------------
    def _on_goal(self, goal: MovePrimitive.Goal) -> GoalResponse:
        if self._busy:
            self.get_logger().warn("goal rejected: busy")
            return GoalResponse.REJECT
        if goal.kind not in (MovePrimitive.Goal.TURN, MovePrimitive.Goal.DRIVE):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_cancel(self, _handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def _execute(self, handle):
        self._busy = True
        goal = handle.request
        result = MovePrimitive.Result()
        t0 = time.monotonic()
        try:
            pose = self.pose()
            deadline = t0 + 2.0
            while pose is None and time.monotonic() < deadline:
                time.sleep(0.05)
                pose = self.pose()
            if pose is None:
                handle.abort()
                result.message = "no pose"
                return result
            timeout = goal.timeout_s if goal.timeout_s > 0 else self.timeout
            if goal.kind == MovePrimitive.Goal.TURN:
                ctrl = TurnController(goal.theta, goal.tolerance_theta or self.tol_theta,
                                      self.limits, self.gains)
            else:
                ctrl = DriveController((pose[0], pose[1]), (goal.x, goal.y), goal.reverse,
                                       goal.tolerance_xy or self.tol_xy, self.limits, self.gains,
                                       goal.max_speed)
            fb = MovePrimitive.Feedback()
            t_prev = time.monotonic()
            while rclpy.ok():
                if handle.is_cancel_requested:
                    self._stop()
                    handle.canceled()
                    result.message = "canceled"
                    return result
                if time.monotonic() - t0 > timeout:
                    self._stop()
                    handle.abort()
                    result.message = f"timeout after {timeout:.1f}s"
                    return result
                x, y, yaw = self.pose()
                t_now = time.monotonic()
                dt = min(max(t_now - t_prev, 0.5 * self.dt), 2.0 * self.dt)
                t_prev = t_now
                if goal.kind == MovePrimitive.Goal.TURN:
                    v, w, done, e_h = ctrl.step(yaw, dt)
                    dist = 0.0
                else:
                    v, w, done, dist, e_h = ctrl.step(x, y, yaw, dt)
                tw = Twist()
                tw.linear.x, tw.angular.z = float(v), float(w)
                self.cmd_pub.publish(tw)
                fb.distance_remaining, fb.heading_error, fb.speed = float(dist), float(e_h), float(v)
                handle.publish_feedback(fb)
                if done:
                    break
                time.sleep(self.dt)
            self._stop()
            x, y, yaw = self.pose()
            result.success = True
            result.message = "ok"
            result.final_error_xy = 0.0 if goal.kind == MovePrimitive.Goal.TURN else math.hypot(goal.x - x, goal.y - y)
            result.final_error_theta = abs(wrap(goal.theta - yaw)) if goal.kind == MovePrimitive.Goal.TURN else 0.0
            result.elapsed_s = time.monotonic() - t0
            handle.succeed()
            return result
        finally:
            self._busy = False


def main() -> None:
    rclpy.init()
    node = PrimitiveController()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
