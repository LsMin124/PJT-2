"""sil_ros.geometry 단위 테스트 — rclpy 없이 표준 math 만으로 돈다.
  cd sil && python3 -m pytest -q tests/test_sil_ros_geometry.py
"""
import math
import os
import random
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))   # …/sil
from sil_ros.geometry import Pose2D, ang_diff, dist, heading_to, pose2d_of, stamp_to_sec, wrap, yaw_of  # noqa: E402

PI = math.pi


def quat_z(yaw):
    """z축 회전 yaw 의 쿼터니언 (duck-typed geometry_msgs/Quaternion)."""
    return SimpleNamespace(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))


def random_unit_quat(rng):
    q = [rng.gauss(0, 1) for _ in range(4)]
    n = math.sqrt(sum(v * v for v in q))
    return SimpleNamespace(x=q[0] / n, y=q[1] / n, z=q[2] / n, w=q[3] / n)


# ── wrap · ang_diff ──
@pytest.mark.parametrize("a", [0.0, 0.5, -0.5, PI - 1e-9, -PI + 1e-9, 3 * PI, -3 * PI, 10.0, -10.0, 2 * PI + 0.3])
def test_wrap_stays_within_pi(a):
    w = wrap(a)
    assert -PI <= w <= PI
    assert math.isclose(math.sin(w), math.sin(a), abs_tol=1e-12)
    assert math.isclose(math.cos(w), math.cos(a), abs_tol=1e-12)


def test_wrap_is_identity_inside_range():
    assert wrap(0.5) == pytest.approx(0.5)
    assert wrap(-2.0) == pytest.approx(-2.0)
    assert wrap(2 * PI + 0.3) == pytest.approx(0.3)


def test_ang_diff_matches_original_follower_formula():
    # follower.ang_diff 원식: atan2(sin(a-b), cos(a-b))
    for a in (0.0, 1.0, 3.0, -3.0, 6.0):
        for b in (0.0, -1.0, 2.5, 4.0):
            assert ang_diff(a, b) == pytest.approx(math.atan2(math.sin(a - b), math.cos(a - b)))


def test_ang_diff_shortest_way_across_pi():
    assert ang_diff(PI - 0.1, -PI + 0.1) == pytest.approx(-0.2)
    assert ang_diff(-PI + 0.1, PI - 0.1) == pytest.approx(0.2)


# ── yaw_of · pose2d_of ──
@pytest.mark.parametrize("yaw", [0.0, 0.3, -0.3, PI / 2, -PI / 2, PI - 0.01, -PI + 0.01])
def test_yaw_of_recovers_z_rotation(yaw):
    assert yaw_of(quat_z(yaw)) == pytest.approx(yaw, abs=1e-12)


def test_yaw_of_matches_original_patrol_formula_on_random_quats():
    rng = random.Random(7)
    for _ in range(200):
        q = random_unit_quat(rng)
        ref = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))   # patrol.pose 원식
        assert yaw_of(q) == pytest.approx(ref, abs=1e-12)


def test_pose2d_of_unpacks_like_tuple():
    pose = SimpleNamespace(position=SimpleNamespace(x=1.5, y=-2.0, z=0.0), orientation=quat_z(0.7))
    p = pose2d_of(pose)
    assert isinstance(p, Pose2D)
    x, y, yaw = p
    assert (x, y) == (1.5, -2.0)
    assert yaw == pytest.approx(0.7)


# ── stamp · dist · heading ──
def test_stamp_to_sec():
    assert stamp_to_sec(SimpleNamespace(sec=12, nanosec=500_000_000)) == pytest.approx(12.5)
    assert stamp_to_sec(SimpleNamespace(sec=0, nanosec=0)) == 0.0


def test_dist_and_heading():
    assert dist((0, 0), (3, 4)) == 5.0
    assert dist((3, 4), (0, 0)) == 5.0
    assert heading_to((0, 0), (0, 1)) == pytest.approx(PI / 2)
    assert heading_to((1, 1), (0, 1)) == pytest.approx(PI)
