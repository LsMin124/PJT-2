"""WheeledRobot 스폰 + 차동 컨트롤러 + (선택) 에셋 수술 → RobotHandle."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Optional, Tuple

import numpy as np

from .iwhub import apply_asset_fixes
from .kinematics import PoseSample, PoseTracker
from .profile import RobotProfile


@dataclass
class RobotHandle:
    name: str
    root: str                 # 프림 경로 (/World/amr01)
    profile: RobotProfile
    robot: Any                # WheeledRobot
    controller: Any           # DifferentialController
    tracker: PoseTracker = field(default_factory=PoseTracker)
    lidar_path: Optional[str] = None
    debug: dict = field(default_factory=dict)

    @property
    def chassis_path(self) -> str:
        return f"{self.root}/{self.profile.chassis_link}"

    def clamp(self, v: float, w: float) -> Tuple[float, float]:
        p = self.profile
        return float(np.clip(v, -p.max_lin, p.max_lin)), float(np.clip(w, -p.max_ang, p.max_ang))

    def drive(self, v: float, w: float) -> Tuple[float, float]:
        """선속·각속 지령(클램프 적용) → 바퀴 속도 목표. 실제로 적용한 (v, w)를 돌려준다."""
        v, w = self.clamp(v, w)
        self.robot.apply_wheel_actions(self.controller.forward([v, w]))
        return v, w

    def sample(self, t: float) -> PoseSample:
        return self.tracker.sample(self.robot, t)

    def dof_positions(self) -> np.ndarray:
        return np.asarray(self.robot.get_dof_positions().numpy()).reshape(-1)


def spawn_robot(profile: RobotProfile, name: str, xy: Tuple[float, float], stage=None,
                root: Optional[str] = None, fixes: bool = True, spawn_z: Optional[float] = None) -> RobotHandle:
    from isaacsim.robot.experimental.wheeled_robots.controllers import DifferentialController
    from isaacsim.robot.experimental.wheeled_robots.robots import WheeledRobot

    root = root or f"/World/{name}"
    z = profile.spawn_z if spawn_z is None else spawn_z
    robot = WheeledRobot(paths=root, wheel_dof_names=list(profile.wheel_dof_names), usd_path=profile.usd_path,
                         positions=[float(xy[0]), float(xy[1]), z])
    if fixes:
        if stage is None:
            from ..stage import current_stage
            stage = current_stage()
        apply_asset_fixes(stage, root, profile)
    controller = DifferentialController(wheel_radius=profile.wheel_radius, wheel_base=profile.track)
    return RobotHandle(name=name, root=root, profile=profile, robot=robot, controller=controller)


def find_chassis_link(stage, root: str, wheel_joint: str = "left_wheel_joint") -> str:
    """구동 조인트의 body 관계에서 섀시(바퀴 아닌 쪽) 링크 경로를 찾는다 — 에셋 구조를 모를 때."""
    from pxr import UsdPhysics

    for prim in stage.Traverse():
        if prim.GetName() == wheel_joint and str(prim.GetPath()).startswith(root):
            joint = UsdPhysics.RevoluteJoint(prim)
            cands = [str(t) for rel in (joint.GetBody0Rel(), joint.GetBody1Rel()) for t in rel.GetTargets()]
            hit = next((c for c in cands if "wheel" not in c.lower()), None)
            if hit:
                return hit
    raise RuntimeError(f"{root}: 섀시 링크를 찾지 못함 ({wheel_joint} body 관계 확인)")
