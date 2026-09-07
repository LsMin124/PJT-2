"""sil_ros.twist_ramp 단위 테스트 — 표준 math 만(rclpy 불필요).
  cd sil && python3 -m pytest -q tests/test_sil_ros_twist_ramp.py
"""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))   # …/sil
from sil_ros.twist_ramp import SafetyField, clamp, front_clearance, ramp  # noqa: E402


def test_clamp():
    assert clamp(5, -1, 1) == 1 and clamp(-5, -1, 1) == -1 and clamp(0.3, -1, 1) == 0.3


def test_ramp_limits_step_then_converges():
    v = 0.0
    seq = []
    for _ in range(5):
        v = ramp(v, 0.6, 0.5 * 0.05)       # A_LIN 0.5 · dt 0.05 → 스텝 0.025
        seq.append(v)
    assert seq == pytest.approx([0.025, 0.05, 0.075, 0.1, 0.125])
    assert ramp(0.6, 0.6, 0.1) == 0.6
    assert ramp(0.6, 0.0, 0.1) == pytest.approx(0.5)
    assert ramp(0.02, 0.0, 0.1) == 0.0


def ref_front(ranges, angle_min, inc, rmin, rmax, lat_half):
    # follower.cb_scan 원식
    best = float("inf")
    ang = angle_min
    for r in ranges:
        if rmin < r < rmax and abs(ang) < 1.4:
            fx = r * math.cos(ang)
            if 0.0 < fx < best and abs(r * math.sin(ang)) < lat_half:
                best = fx
        ang += inc
    return best


def test_front_clearance_rectangular_field():
    inc = math.radians(1)
    angle_min = -math.pi
    n = 360
    ranges = [10.0] * n
    ranges[180] = 2.0                      # 정면 2 m
    ranges[180 + 60] = 1.2                 # 60° 옆 1.2 m → 측면 1.04 m > 0.45 제외
    ranges[180 + 5] = 1.5                  # 5° 앞 1.5 m → 측면 0.13 m 포함, 전방 1.494
    got = front_clearance(ranges, angle_min, inc, 1.0, 20.0, 0.45, 1.4)
    assert got == pytest.approx(1.5 * math.cos(math.radians(5)))
    assert got == pytest.approx(ref_front(ranges, angle_min, inc, 1.0, 20.0, 0.45))
    assert front_clearance([0.5] * n, angle_min, inc, 1.0, 20.0, 0.45, 1.4) == math.inf   # range_min 미만은 안 보임


def test_safety_field_hysteresis_and_blocked_warning():
    f = SafetyField(blocked_warn_s=20.0)
    assert f.step(3.0, 1.2, 1.4, t=0.0) is None and not f.stopped
    assert f.step(1.1, 1.2, 1.4, t=1.0) == SafetyField.STOP and f.stopped and f.stops == 1
    assert f.step(1.3, 1.2, 1.4, t=2.0) is None and f.stopped          # 해제 거리 미만 → 유지
    assert f.step(1.3, 1.2, 1.4, t=21.5) == SafetyField.BLOCKED and f.stop_t == 21.5
    assert f.step(1.3, 1.2, 1.4, t=30.0) is None                        # 경고 간격 재시작
    assert f.step(1.5, 1.2, 1.4, t=31.0) == SafetyField.RESUME and not f.stopped
    assert f.step(1.0, 1.2, 1.4, t=32.0) == SafetyField.STOP and f.stops == 2
