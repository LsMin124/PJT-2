"""포즈·속도 샘플링과 쿼터니언 유틸 (numpy 만 사용 — Isaac 없이 테스트 가능)."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np


def yaw_from_wxyz(q) -> float:
    w, x, y, z = (float(v) for v in q[:4])
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def pitch_from_wxyz(q) -> float:
    w, x, y, z = (float(v) for v in q[:4])
    return math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))


def wxyz_to_ros(q) -> list:
    """실험 API (w,x,y,z) → ROS (x,y,z,w)."""
    return [float(q[1]), float(q[2]), float(q[3]), float(q[0])]


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


@dataclass
class PoseSample:
    t: float
    p: np.ndarray      # (3,) 월드 위치
    q: np.ndarray      # (4,) (w, x, y, z)
    lv: np.ndarray     # (3,) 선속도
    av: np.ndarray     # (3,) 각속도

    @property
    def yaw(self) -> float:
        return yaw_from_wxyz(self.q)

    @property
    def pitch(self) -> float:
        return pitch_from_wxyz(self.q)

    @property
    def q_ros(self) -> list:
        return wxyz_to_ros(self.q)


class PoseTracker:
    """로봇 하나의 월드 포즈 + 속도. get_velocities 가 없으면 유한차분으로 폴백 (T1 검증 패턴)."""

    def __init__(self) -> None:
        self._prev: Optional[tuple] = None   # (t, p, yaw)

    def finite_difference(self, t: float, p: np.ndarray, q: np.ndarray) -> tuple:
        yaw = yaw_from_wxyz(q)
        if self._prev is None or t <= self._prev[0]:
            lv, av = np.zeros(3), np.zeros(3)
        else:
            t0, p0, yaw0 = self._prev
            dt = t - t0
            lv = (p - p0) / dt
            av = np.array([0.0, 0.0, wrap_angle(yaw - yaw0) / dt])
        self._prev = (t, p.copy(), yaw)
        return lv, av

    def sample(self, robot, t: float) -> PoseSample:
        pos_w, ori_w = robot.get_world_poses()
        p = np.asarray(pos_w.numpy()).reshape(-1)[:3]
        q = np.asarray(ori_w.numpy()).reshape(-1)[:4]
        try:
            lv_w, av_w = robot.get_velocities()
            lv = np.asarray(lv_w.numpy()).reshape(-1)[:3]
            av = np.asarray(av_w.numpy()).reshape(-1)[:3]
        except Exception:  # noqa: BLE001 — 실험 API 부재 시 폴백
            lv, av = self.finite_difference(t, p, q)
        return PoseSample(t=float(t), p=p, q=q, lv=lv, av=av)
