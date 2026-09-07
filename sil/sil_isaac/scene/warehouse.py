"""T3 창고 씬 로드와 스폰 좌표 (좌표 규칙은 omni 없이 계산)."""
from __future__ import annotations

import os
import sys
from typing import List, Optional, Sequence, Tuple

SIL_ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
try:  # 씬 빌더와 같은 경로 정의를 쓴다 (scene_builder.config 는 pxr 없이 import 됨)
    if SIL_ROOT not in sys.path:
        sys.path.insert(0, SIL_ROOT)
    from scene_builder.config import USD_PATH as WAREHOUSE_USD
except Exception:  # noqa: BLE001 — 빌더 없이 sil_isaac 만 복사된 경우
    WAREHOUSE_USD = os.path.join(SIL_ROOT, "t3_warehouse", "warehouse_scene.usd")
WAREHOUSE_PRIM = "/World/warehouse"

SINGLE_SPAWN = (34.0, 40.8)     # 남측 코리도, handoff 인근 — 그리드 0·팽창 밖 실측 (단일 모드)
MULTI_SPAWN0 = (36.0, 40.8)     # 다중 모드 첫 로봇 — x 36~60 구간 장애물 이격 ≥ 3.9 m (occupancy_grid 실측)
DEFAULT_SPACING = 3.0           # m, +x 방향


def parse_spawn_list(raw: str) -> List[Tuple[float, float]]:
    """"x,y;x,y;…" → [(x, y), …]."""
    return [tuple(float(v) for v in s.split(",")) for s in raw.split(";") if s.strip()]


def spawn_points(n: int, spacing: float = DEFAULT_SPACING, explicit: Optional[Sequence[Tuple[float, float]]] = None
                 ) -> List[Tuple[float, float]]:
    """로봇 n 대의 스폰 좌표. explicit 가 있으면 개수가 n 과 같아야 한다."""
    if explicit:
        pts = [tuple(p) for p in explicit]
        if len(pts) != n:
            raise ValueError(f"spawn list has {len(pts)} points, expected {n}")
        return pts
    if n == 1:
        return [SINGLE_SPAWN]
    return [(MULTI_SPAWN0[0] + spacing * i, MULTI_SPAWN0[1]) for i in range(n)]


def robot_names(n: int, namespaced: bool) -> List[str]:
    return [f"amr{i + 1:02d}" for i in range(n)] if namespaced else ["iw_hub"]


def load_warehouse(usd_path: str = WAREHOUSE_USD, physics_usd: Optional[str] = None) -> str:
    from ..stage import add_reference, init_stage

    init_stage()
    return add_reference(usd_path, WAREHOUSE_PRIM, physics_usd)
