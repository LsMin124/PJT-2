"""로봇 추종 카메라 — 지수 평활로 부드럽게 따라간다 (T1·T2·T3 공통)."""
from __future__ import annotations

from typing import Optional

import numpy as np

WAREHOUSE_OFFSET = (-4.0, -4.0, 7.0)   # 고각 — 스폰 인근 사무실 벽(3 m)에 안 가리게
GRID_OFFSET = (-4.5, -4.5, 3.0)        # T1·T2 격자 환경
DEFAULT_ALPHA = 0.06                   # 프레임당 평활 계수 — 클수록 민첩


class FollowCamera:
    def __init__(self, offset=WAREHOUSE_OFFSET, alpha: float = DEFAULT_ALPHA, target_z: float = 0.3) -> None:
        self.offset = np.asarray(offset, dtype=float)
        self.alpha = alpha
        self.target_z = target_z
        self._eye: Optional[np.ndarray] = None
        self._target: Optional[np.ndarray] = None
        try:
            from isaacsim.core.utils.viewports import set_camera_view
            self._set = set_camera_view
        except Exception as exc:  # noqa: BLE001 — 뷰포트 없는 구성(경량)에서는 조용히 비활성
            self._set = None
            print(f"[sil] camera setup skipped: {exc}", flush=True)

    def look_at(self, eye, target) -> None:
        if self._set:
            self._set(eye=list(eye), target=list(target))

    def update(self, p) -> None:
        if self._set is None:
            return
        want_eye = np.asarray(p, dtype=float) + self.offset
        want_tgt = np.array([p[0], p[1], self.target_z], dtype=float)
        if self._eye is None:
            self._eye, self._target = want_eye.copy(), want_tgt.copy()
        else:
            self._eye += self.alpha * (want_eye - self._eye)
            self._target += self.alpha * (want_tgt - self._target)
        self._set(eye=self._eye.tolist(), target=self._target.tolist())
