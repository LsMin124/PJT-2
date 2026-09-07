"""SimulationApp 기동 · 확장 · 물리 셋업 · 경량(lean) 프로파일.

경량 프로파일은 병렬 실측(pages/working/isaac_parallel_measure.html)의 a~g 구성을 그대로 옮긴 것이다.
Kit 설정 인자는 SimulationApp 생성 전에 sys.argv 에 붙여야 적용된다.
"""
from __future__ import annotations

import os
import sys
from dataclasses import dataclass, field
from typing import Any, Optional, Sequence

DEFAULT_DT = 1.0 / 60.0
DEFAULT_WINDOW = (1280, 720)

# 인스턴스 i 의 포트: WebRTC 49100+i / 47998+i, MJPEG 8211+i (병렬 실측 규약)
WEBRTC_SIGNAL_PORT = 49100
WEBRTC_STREAM_PORT = 47998
MJPEG_PORT = 8211

RTX_OFF_ARGS = (
    "--/rtx/post/dlss/execMode=0", "--/rtx/post/aa/op=0", "--/rtx/reflections/enabled=false",
    "--/rtx/indirectDiffuse/enabled=false", "--/rtx/ambientOcclusion/enabled=false",
    "--/rtx/shadows/enabled=false", "--/rtx/directLighting/sampledLighting/enabled=false",
    "--/rtx/translucency/enabled=false", "--/rtx/raytracing/subsurface/enabled=false",
    "--/rtx-transient/resourcemanager/enableTextureStreaming=true",
    "--/rtx-transient/resourcemanager/texturestreaming/memoryBudget=0.05",
)


@dataclass(frozen=True)
class LeanProfile:
    """경량 구성 한 벌. name 은 실측 보고서의 a~g 와 같다."""
    name: str = "a"
    rtx_off: bool = False            # c 이상: RTX 효과 off + 텍스처 예산 0.05
    resolution: Optional[tuple] = None  # b 이상: 렌더 타깃 축소 (예: (320, 180))
    livestream: bool = True          # b 이상: WebRTC 확장 생략
    mjpeg: bool = True               # b 이상: MJPEG 캡처 생략
    viewport_updates: bool = True    # e·g: 뷰포트 렌더 중단
    physics_cpu: bool = False        # f·g: PhysX CPU (cudaDevice=-1)
    physics_usd: Optional[str] = None  # d 이상: 물리 전용 스테이지로 치환

    def kit_args(self, instance: int = 0) -> list:
        args = [f"--/exts/omni.kit.livestream.app/primaryStream/signalPort={WEBRTC_SIGNAL_PORT + instance}",
                f"--/exts/omni.kit.livestream.app/primaryStream/streamPort={WEBRTC_STREAM_PORT + instance}"]
        if self.rtx_off:
            args += list(RTX_OFF_ARGS)
        return args

    def launch_config(self, headless: bool = True, window: tuple = DEFAULT_WINDOW) -> dict:
        w, h = self.resolution or window
        cfg: dict = {"headless": headless, "hide_ui": False, "window_width": w, "window_height": h}
        if self.resolution:
            cfg.update(width=w, height=h)
        if self.physics_cpu:
            cfg["physics_gpu"] = -1
        return cfg


def lean_profile(name: str, physics_usd: Optional[str] = None, resolution: tuple = (320, 180)) -> LeanProfile:
    """실측 보고서의 a~g 구성을 이름으로 만든다. a = 원본 그대로."""
    name = (name or "a").lower()
    if name not in "abcdefg" or len(name) != 1:
        raise ValueError(f"unknown lean profile {name!r} (a~g)")
    lean = name != "a"
    return LeanProfile(
        name=name,
        rtx_off=name in "cdefg",
        resolution=resolution if lean else None,
        livestream=not lean,
        mjpeg=not lean,
        viewport_updates=name not in "eg",
        physics_cpu=name in "fg",
        physics_usd=physics_usd if name in "defg" else None,
    )


def profile_from_env() -> tuple:
    """WSIM_LEAN=a~g · WSIM_INST=i · WSIM_PHYS_USD=<usda> · WSIM_RES=WxH → (LeanProfile, instance)."""
    res = tuple(int(x) for x in os.environ.get("WSIM_RES", "320x180").split("x"))
    prof = lean_profile(os.environ.get("WSIM_LEAN", "a"), os.environ.get("WSIM_PHYS_USD") or None, res)
    return prof, int(os.environ.get("WSIM_INST", "0"))


@dataclass
class IsaacApp:
    """SimulationApp 핸들 + 프로파일. 모든 Isaac 엔트리가 이 객체 하나로 시작한다."""
    sim: Any
    profile: LeanProfile
    instance: int = 0
    _viewport_off: bool = field(default=False, init=False)

    @property
    def mjpeg_port(self) -> int:
        return MJPEG_PORT + self.instance

    def update(self) -> None:
        self.sim.update()

    def is_running(self) -> bool:
        return self.sim.is_running()

    def close(self) -> None:
        self.sim.close()

    def disable_viewport_updates(self) -> None:
        """e·g 프로파일: 렌더는 두되 뷰포트 갱신만 끈다 (VRAM 고정 몫 절감). 한 번만."""
        if self._viewport_off or self.profile.viewport_updates:
            return
        self._viewport_off = True
        try:
            from omni.kit.viewport.utility import get_active_viewport
            get_active_viewport().updates_enabled = False
            print("[sil] 뷰포트 렌더 중단", flush=True)
        except Exception as e:  # noqa: BLE001 — 관전 옵션 실패가 시뮬을 막으면 안 됨
            print(f"[sil] viewport off 실패: {e}", flush=True)


def launch(profile: Optional[LeanProfile] = None, instance: int = 0, headless: bool = True,
           window: tuple = DEFAULT_WINDOW, ros2: bool = True, extra_kit_args: Sequence[str] = ()) -> IsaacApp:
    """SimulationApp 을 만들고 확장을 켠다. 이 함수 뒤에야 omni/isaacsim 모듈을 import 할 수 있다."""
    profile = profile or LeanProfile()
    sys.argv = [sys.argv[0]] + profile.kit_args(instance) + list(extra_kit_args)
    from isaacsim import SimulationApp

    sim = SimulationApp(launch_config=profile.launch_config(headless, window))
    sim.set_setting("/app/window/drawMouse", True)
    import faulthandler

    faulthandler.enable()
    from isaacsim.core.experimental.utils.app import enable_extension

    if profile.livestream:
        enable_extension("omni.kit.livestream.app")
    else:
        print("[sil] livestream 확장 생략", flush=True)
    if ros2:
        enable_extension("isaacsim.ros2.bridge")
    if profile.physics_cpu:
        print("[sil] PhysX → CPU (cudaDevice=-1)", flush=True)
    print(f"[sil] app up — profile={profile.name} instance={instance} ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID')}",
          flush=True)
    return IsaacApp(sim=sim, profile=profile, instance=instance)


def setup_physics(dt: float = DEFAULT_DT, device: str = "cpu", gpu_dynamics: bool = False) -> None:
    """물리 스텝·디바이스. 로봇·센서를 다 만든 뒤 호출한다 (원본 순서 유지)."""
    from isaacsim.core.simulation_manager import SimulationManager

    SimulationManager.setup_simulation(dt=dt, device=device)
    SimulationManager.get_physics_scenes()[0].set_enabled_gpu_dynamics(gpu_dynamics)
