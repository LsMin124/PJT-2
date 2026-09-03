"""Kinematic fake robot: /cmd_vel → odom (+ TF odom→base_link). Stands in for Isaac.

Response mimics T1 measurement (near-instant, ~6 m/s^2) so the controller's own
ramps are what shape the motion — same as against Isaac.
"""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class FakeRobot(Node):
    def __init__(self) -> None:
        super().__init__("fake_robot")
        p = self.declare_parameters("", [
            ("x", 0.0), ("y", 0.0), ("yaw", 0.0), ("rate_hz", 50.0),
            ("a_max", 6.0), ("alpha_max", 12.0), ("v_max", 0.835), ("w_max", 2.0),
            ("odom_frame", "odom"), ("base_frame", "base_link"), ("publish_tf", True),
        ])
        g = {q.name: q.value for q in p}
        self.x, self.y, self.yaw = g["x"], g["y"], g["yaw"]
        self.v = self.w = 0.0
        self.v_cmd = self.w_cmd = 0.0
        self.a_max, self.alpha_max = g["a_max"], g["alpha_max"]
        self.v_max, self.w_max = g["v_max"], g["w_max"]
        self.odom_frame, self.base_frame = g["odom_frame"], g["base_frame"]
        self.dt = 1.0 / g["rate_hz"]
        self.create_subscription(Twist, "cmd_vel", self._on_cmd, 10)
        self.odom_pub = self.create_publisher(Odometry, "odom", 20)
        self.tf = TransformBroadcaster(self) if g["publish_tf"] else None
        self.create_timer(self.dt, self._step)
        self.get_logger().info(f"fake robot at ({self.x:.2f}, {self.y:.2f}, {self.yaw:.2f})")

    def _on_cmd(self, m: Twist) -> None:
        self.v_cmd = max(-self.v_max, min(self.v_max, m.linear.x))
        self.w_cmd = max(-self.w_max, min(self.w_max, m.angular.z))

    def _step(self) -> None:
        dv, dw = self.a_max * self.dt, self.alpha_max * self.dt
        self.v += max(-dv, min(dv, self.v_cmd - self.v))
        self.w += max(-dw, min(dw, self.w_cmd - self.w))
        self.x += self.v * math.cos(self.yaw) * self.dt
        self.y += self.v * math.sin(self.yaw) * self.dt
        self.yaw = math.atan2(math.sin(self.yaw + self.w * self.dt), math.cos(self.yaw + self.w * self.dt))
        now = self.get_clock().now().to_msg()
        qz, qw = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
        od = Odometry()
        od.header.stamp = now
        od.header.frame_id = self.odom_frame
        od.child_frame_id = self.base_frame
        od.pose.pose.position.x, od.pose.pose.position.y = self.x, self.y
        od.pose.pose.orientation.z, od.pose.pose.orientation.w = qz, qw
        od.twist.twist.linear.x, od.twist.twist.angular.z = self.v, self.w
        self.odom_pub.publish(od)
        if self.tf:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x, t.transform.translation.y = self.x, self.y
            t.transform.rotation.z, t.transform.rotation.w = qz, qw
            self.tf.sendTransform(t)


def main() -> None:
    rclpy.init()
    n = FakeRobot()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
