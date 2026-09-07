"""T2 GT 방 — 좌표를 아는 벽 4·박스 3 (SLAM 지도 오차 비교용) + 토픽으로 움직이는 장애물."""
from __future__ import annotations

from typing import Tuple

# (이름, 중심 xyz, 전체 치수 dx dy dz) — 12 x 9 m, 벽 두께 0.2
ROOM = [
    ("wall_e", (6.0, 0.0, 0.5), (0.2, 9.4, 1.0)),
    ("wall_w", (-6.0, 0.0, 0.5), (0.2, 9.4, 1.0)),
    ("wall_n", (0.0, 4.5, 0.5), (12.4, 0.2, 1.0)),
    ("wall_s", (0.0, -4.5, 0.5), (12.4, 0.2, 1.0)),
    ("box_a", (-2.5, 2.5, 0.4), (1.0, 1.0, 0.8)),
    ("box_b", (-3.0, -2.0, 0.4), (0.8, 1.6, 0.8)),
    ("box_c", (2.0, -2.6, 0.4), (1.2, 0.6, 0.8)),  # ROOM v2: 남측 회랑 1.5 m 확보(1.1 m는 통행 불가 실측 판정)
]
OBSTACLE_HOME = (20.0, 20.0, 0.7)
OBSTACLE_SIZE = (0.6, 0.6, 1.4)   # 라이다 평면(~0.7 m)보다 높게 — 0.6 m 큐브는 빔이 위로 지나감(실측)


def _add_cube(stage, path: str, center, size):
    from pxr import Gf, UsdGeom, UsdPhysics

    cube = UsdGeom.Cube.Define(stage, path)
    cube.GetSizeAttr().Set(1.0)
    xf = UsdGeom.Xformable(cube.GetPrim())
    translate = xf.AddTranslateOp()
    translate.Set(Gf.Vec3d(*center))
    xf.AddScaleOp().Set(Gf.Vec3f(*size))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    return translate


def build_gt_room(stage, root: str = "/World/room") -> None:
    for name, center, size in ROOM:
        _add_cube(stage, f"{root}/{name}", center, size)


class MovableObstacle:
    """/obstacle_cmd (Twist) — linear.z > 0.5 일 때 linear.x, y 로 순간이동."""

    def __init__(self, stage, path: str = "/World/obstacle") -> None:
        self._translate = _add_cube(stage, path, OBSTACLE_HOME, OBSTACLE_SIZE)

    def apply(self, cmd: Tuple[float, float, float]) -> None:
        from pxr import Gf

        if cmd[2] > 0.5:
            self._translate.Set(Gf.Vec3d(float(cmd[0]), float(cmd[1]), OBSTACLE_HOME[2]))
