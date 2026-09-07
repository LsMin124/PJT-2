"""순수 기하 유틸 — patrol · follower · spin_probe · measure_accel 에 흩어져 있던 중복을 한 곳으로.

rclpy 없이 import 된다(표준 math 만). ROS 메시지는 duck-typing 으로 받는다:
  yaw_of(q)          q.x/q.y/q.z/q.w 를 가진 쿼터니언 → yaw(rad)
  pose2d_of(pose)    geometry_msgs/Pose(position · orientation) → Pose2D(x, y, yaw)
  stamp_to_sec(st)   builtin_interfaces/Time(sec · nanosec) → float 초
"""
import math
from typing import NamedTuple


class Pose2D(NamedTuple):
    """평면 포즈 — 튜플처럼 `x, y, yaw = pose` 로 풀 수 있다."""
    x: float
    y: float
    yaw: float


def wrap(a):
    """각도를 (-π, π] 로 접는다 (patrol.wrap · amr_agent.control.primitives.wrap 과 동일식)."""
    return math.atan2(math.sin(a), math.cos(a))


def ang_diff(a, b):
    """a - b 를 (-π, π] 로 접은 값 (follower.ang_diff 와 동일식)."""
    return wrap(a - b)


def yaw_of(q):
    """쿼터니언(x, y, z, w) → yaw(rad). patrol.pose · follower.cb_odom · spin_probe.yaw_of 와 동일식."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def pose2d_of(pose):
    """geometry_msgs/Pose → Pose2D."""
    return Pose2D(pose.position.x, pose.position.y, yaw_of(pose.orientation))


def stamp_to_sec(stamp):
    """builtin_interfaces/Time → 초(float)."""
    return stamp.sec + stamp.nanosec * 1e-9


def dist(p, q):
    """두 점(x, y) 사이 유클리드 거리."""
    return math.hypot(q[0] - p[0], q[1] - p[1])


def heading_to(p, q):
    """p 에서 q 를 바라보는 방위각(rad)."""
    return math.atan2(q[1] - p[1], q[0] - p[0])
