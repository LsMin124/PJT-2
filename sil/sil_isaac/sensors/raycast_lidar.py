"""물리 레이캐스트 2D 라이다 — 콜라이더 기준이라 map.pgm(동일 그리드)과 원천이 일치, AMCL 정합에 유리.

반드시 '리지드바디 링크'(chassis) 밑에 붙인다. 관절 루트 Xform 에 붙이면 USD 트랜스폼이 시뮬 중
갱신되지 않아(fabric) 센서가 스폰 위치에 동결 + 로봇 자체도 점진 정지 (실측 — 서쪽 레이 19.60 고정).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Tuple


@dataclass(frozen=True)
class LidarSpec:
    """2D 360° 라이다 사양. 기본값 = iw.hub 실측 스택 (720빔 0.5°, 0.4~20 m)."""
    h_res_deg: float = 0.5
    r_min: float = 0.4
    r_max: float = 20.0
    z: float = 0.45          # 링크 원점 기준 높이
    frame_id: str = "laser"

    @property
    def n_rays(self) -> int:
        return int(round(360.0 / self.h_res_deg))

    @property
    def azimuth_range(self) -> Tuple[float, float]:
        return (-180.0, 180.0)


def ray_table(spec: LidarSpec) -> Tuple[List[list], List[list]]:
    """레이 원점(전부 0)과 방향 — -180° 부터 h_res 간격, 수평면."""
    origins, directions = [], []
    for i in range(spec.n_rays):
        a = math.radians(-180.0 + 360.0 * i / spec.n_rays)
        origins.append([0.0, 0.0, 0.0])
        directions.append([math.cos(a), math.sin(a), 0.0])
    return origins, directions


def create_raycast_lidar(link_path: str, spec: LidarSpec, name: str = "lidar") -> str:
    """link_path (리지드바디 링크) 밑에 Raycast 센서를 만들고 프림 경로를 돌려준다."""
    from isaacsim.sensors.experimental.physics import Raycast

    origins, directions = ray_table(spec)
    lidar = Raycast.create(f"{link_path}/{name}", min_range=spec.r_min, max_range=spec.r_max,
                           ray_origins=origins, ray_directions=directions,
                           translations=[[0.0, 0.0, spec.z]])
    return lidar.paths[0]
