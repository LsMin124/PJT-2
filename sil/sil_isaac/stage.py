"""스테이지 초기화와 참조 로드 — 경량 프로파일의 물리 전용 스테이지 치환도 여기서."""
from __future__ import annotations

from typing import Optional


def init_stage(up_axis: str = "Z", meters_per_unit: float = 1.0) -> None:
    from isaacsim.core.experimental.utils import stage as stage_utils

    stage_utils.set_stage_up_axis(up_axis)
    stage_utils.set_stage_units(meters_per_unit=meters_per_unit)


def add_reference(usd_path: str, prim_path: str, physics_usd: Optional[str] = None) -> str:
    """usd_path 를 prim_path 에 참조. physics_usd 가 주어지면 그것으로 치환(경량 d 이상). 실제 참조한 경로를 돌려준다."""
    from isaacsim.core.experimental.utils import stage as stage_utils

    if physics_usd:
        print(f"[sil] 스테이지 치환 → {physics_usd}", flush=True)
        usd_path = physics_usd
    stage_utils.add_reference_to_stage(usd_path=usd_path, path=prim_path)
    return usd_path


def current_stage():
    import omni.usd

    return omni.usd.get_context().get_stage()
