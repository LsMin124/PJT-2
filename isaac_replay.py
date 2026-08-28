# -*- coding: utf-8 -*-
# ============================================================
# Isaac Sim 재생 스크립트 — 그리드 시뮬(sim_v1) 결과의 검증 재생
#
# 역할(HandOff 2단 충실도): 그리드 층이 만든 경로를 실제 몸체 크기
# (1,440 x 641 mm)로 재생해 회전·클리어런스·근접 통과를 눈으로/충돌로 검증.
#
# 준비: 그리드 시뮬에서 내보내기 먼저 실행 —
#   (amr_test 환경)  $env:SIM_EXPORT_ISAAC="1"
#                    python sim_v1_tasks.py wallA 6 40 42 1
#   → isaac_export_wallA/scene.json + trajectories.json 생성
#
# 실행: 반드시 Isaac Sim에 딸린 Python으로 실행 (일반 파이썬 아님)
#   Windows:  "%ISAAC_PATH%\python.bat" isaac_replay.py --export-dir isaac_export_wallA
#   Linux:    ~/isaacsim/python.sh     isaac_replay.py --export-dir isaac_export_wallA
#   옵션: --speed 4 (재생 배속), --headless
#
# [홈서버 확장] X 서버가 없는 헤드리스 서버에서 실물 씬 안에 재생:
#   데이터:  sil/t3_warehouse/export_replay.py (v5.6 맵 → isaac_export_t3)
#   실행:    ~/isaacsim/python.sh isaac_replay.py --export-dir isaac_export_t3 \
#              --stage sil/t3_warehouse/warehouse_scene.usd --livestream --loop
#   접속:    WebRTC Streaming Client → LAN 192.168.0.6 / tailscale 100.89.12.112
#   --stage  = 박스 씬 대신 기존 USD 씬을 열어 그 안에서 재생 (검증된 실물 씬)
#   --livestream = WebRTC 스트리밍 (뷰어 겸용) / --loop = 재생 반복
#
# 좌표계: scene/trajectories 모두 크롭 기준 [m], z-up. 창고 바닥 = z0 평면.
# 로봇은 kinematic 재생(물리 off) — 셀 중심 간 선형 보간 + 진행 방향 yaw.
# 다음 단계로 실제 거동 검증을 하려면 VisualCuboid 대신 differential-drive
# 로봇 에셋을 스폰하고 waypoint follower로 교체하면 됨 (교체 지점 주석 참조).
# ============================================================
import argparse
import json
import math
import os
import time

parser = argparse.ArgumentParser()
parser.add_argument("--export-dir", default="isaac_export_wallA")
parser.add_argument("--speed", type=float, default=2.0, help="재생 배속")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--stage", default=None, help="기존 USD 씬 경로 — 박스 씬 대신 그 안에서 재생")
parser.add_argument("--livestream", action="store_true", help="WebRTC 스트리밍 (헤드리스 서버 관전)")
parser.add_argument("--loop", action="store_true", help="재생 반복 (스트리밍 관전용)")
args = parser.parse_args()

EXP = os.path.abspath(args.export_dir)
scene = json.load(open(os.path.join(EXP, "scene.json"), encoding="utf-8"))
traj = json.load(open(os.path.join(EXP, "trajectories.json"), encoding="utf-8"))

# ---- Isaac Sim 앱 기동 (임포트 순서 중요: SimulationApp가 가장 먼저) ----
try:
    from isaacsim import SimulationApp          # Isaac Sim 4.x
except ImportError:
    from omni.isaac.kit import SimulationApp    # 구버전
if args.livestream:                             # 스트리밍 = 헤드리스 강제 + 720p (릴레이 대역폭)
    sim_app = SimulationApp({"headless": True, "hide_ui": False,
                             "window_width": 1280, "window_height": 720})
    sim_app.set_setting("/app/window/drawMouse", True)
    from isaacsim.core.experimental.utils.app import enable_extension
    enable_extension("omni.kit.livestream.app")
else:
    sim_app = SimulationApp({"headless": args.headless})

import numpy as np

try:                                            # Isaac Sim 4.5+
    from isaacsim.core.api import World
    from isaacsim.core.api.objects import FixedCuboid, VisualCuboid
except ImportError:                             # Isaac Sim 4.0~4.2 / 2023.x
    from omni.isaac.core import World
    from omni.isaac.core.objects import FixedCuboid, VisualCuboid

L, Wd, Hh = scene["meta"]["robot_dim_m"]
ROBOT_COLORS = [(0.9, 0.2, 0.2), (0.2, 0.5, 0.9), (0.2, 0.8, 0.4), (0.95, 0.7, 0.1),
                (0.7, 0.3, 0.8), (0.3, 0.8, 0.8), (0.9, 0.4, 0.6), (0.5, 0.5, 0.2)]

if args.stage:
    # ---- [씬 모드] 검증된 USD 씬을 열어 그 안에서 재생 (World 미사용) ----
    # 장애물·마커는 씬에 이미 실물로 존재 — 로봇 큐브(+진행 방향 표식)만 스폰.
    import omni.usd
    from omni.kit.viewport.utility import get_active_viewport
    from pxr import Gf, UsdGeom

    ctx = omni.usd.get_context()
    ctx.open_stage(os.path.abspath(args.stage))
    while ctx.get_stage_loading_status()[2] > 0:
        sim_app.update()
    for _ in range(30):
        sim_app.update()
    stage_usd = ctx.get_stage()

    robots = {}
    for rid, wp in traj["robots"].items():
        color = ROBOT_COLORS[int(rid) % len(ROBOT_COLORS)]
        root = UsdGeom.Xform.Define(stage_usd, f"/World/replay/robot_{rid}")
        xf = UsdGeom.Xformable(root.GetPrim())
        tr = xf.AddTranslateOp()
        rz = xf.AddRotateZOp()
        tr.Set(Gf.Vec3d(wp[0][1], wp[0][2], 0.01))
        body = UsdGeom.Cube.Define(stage_usd, f"/World/replay/robot_{rid}/body")
        body.GetSizeAttr().Set(1.0)
        bx = UsdGeom.Xformable(body.GetPrim())
        bx.AddTranslateOp().Set(Gf.Vec3d(0, 0, Hh / 2))
        bx.AddScaleOp().Set(Gf.Vec3f(L, Wd, Hh))
        body.GetDisplayColorAttr().Set([Gf.Vec3f(*color)])
        nose = UsdGeom.Cube.Define(stage_usd, f"/World/replay/robot_{rid}/nose")
        nose.GetSizeAttr().Set(1.0)                       # 진행 방향 표식 (yaw 육안 확인)
        nx = UsdGeom.Xformable(nose.GetPrim())
        nx.AddTranslateOp().Set(Gf.Vec3d(L * 0.33, 0, Hh + 0.04))
        nx.AddScaleOp().Set(Gf.Vec3f(0.3, 0.18, 0.08))
        nose.GetDisplayColorAttr().Set([Gf.Vec3f(0.95, 0.95, 0.95)])
        robots[rid] = dict(tr=tr, rz=rz, wp=wp, idx=0, yaw=0.0)

    cam = UsdGeom.Camera.Define(stage_usd, "/World/replay_cam")   # 초기 조감 — 접속 후 자유 조작
    cam.CreateFocalLengthAttr(18.0)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.1, 2000))
    cx = UsdGeom.Xformable(cam.GetPrim())
    cx.AddTranslateOp().Set(Gf.Vec3d(20, 20, 22))
    cx.AddRotateXYZOp().Set(Gf.Vec3f(62, 0, -38))
    get_active_viewport().camera_path = "/World/replay_cam"
    n_prim = 0

    def set_pose(st, x, y, yaw):
        st["tr"].Set(Gf.Vec3d(x, y, 0.01))
        st["rz"].Set(math.degrees(yaw))

    def step():
        sim_app.update()

else:
    # ---- [박스 씬 모드] scene.json에서 장애물·마커·로봇을 직접 세운다 (원본 동작) ----
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()

    # 1) 장면: 장애물 박스
    PALETTE = {"structure": (0.45, 0.45, 0.45),
               "racks": (0.85, 0.5, 0.1),
               "conveyor_table": (0.5, 0.3, 0.7)}
    n_prim = 0
    for name, spec in scene["obstacles"].items():
        h = spec["height"]
        color = np.array(PALETTE.get(name, (0.5, 0.5, 0.5)))
        for i, (x, y, w, d) in enumerate(spec["rects"]):
            world.scene.add(FixedCuboid(
                prim_path=f"/World/obs/{name}_{i}", name=f"{name}_{i}",
                position=np.array([x + w / 2, y + d / 2, h / 2]),
                scale=np.array([max(w, 0.05), max(d, 0.05), h]), color=color))
            n_prim += 1

    # 2) 스테이션 마커 (얇은 판, 통행 무관)
    for kind, pts in scene["stations"].items():
        for i, (x, y) in enumerate(pts):
            world.scene.add(VisualCuboid(
                prim_path=f"/World/st/{kind}_{i}", name=f"st_{kind}_{i}",
                position=np.array([x, y, 0.01]),
                scale=np.array([1.0, 1.0, 0.02]), color=np.array([0.1, 0.7, 0.3])))

    # 3) 로봇 (실측 몸체, kinematic 재생)
    robots = {}
    for rid, wp in traj["robots"].items():
        x0, y0 = wp[0][1], wp[0][2]
        # ★교체 지점: 실기 검증 시 VisualCuboid 대신 differential-drive 에셋 스폰
        cub = world.scene.add(VisualCuboid(
            prim_path=f"/World/robot_{rid}", name=f"robot_{rid}",
            position=np.array([x0, y0, Hh / 2]),
            scale=np.array([L, Wd, Hh]),
            color=np.array(ROBOT_COLORS[int(rid) % len(ROBOT_COLORS)])))
        robots[rid] = dict(prim=cub, wp=wp, idx=0, yaw=0.0)

    world.reset()

    def set_pose(st, x, y, yaw):
        quat = np.array([math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)])  # w,x,y,z
        st["prim"].set_world_pose(np.array([x, y, Hh / 2]), quat)

    def step():
        world.step(render=not args.headless)

print(f"[isaac_replay] 장애물 {n_prim}개, 로봇 {len(robots)}대, "
      f"배속 x{args.speed}", flush=True)


def pose_at(state, t):
    """웨이포인트 [(t,x,y),...] 선형 보간. 정지 구간은 위치 유지, yaw는 진행 방향."""
    wp = state["wp"]
    i = state["idx"]
    while i + 1 < len(wp) and wp[i + 1][0] <= t:
        i += 1
    state["idx"] = i
    t0, x0, y0 = wp[i]
    if i + 1 >= len(wp):
        return x0, y0, state["yaw"]
    t1, x1, y1 = wp[i + 1]
    a = 0.0 if t1 <= t0 else min(max((t - t0) / (t1 - t0), 0.0), 1.0)
    x, y = x0 + (x1 - x0) * a, y0 + (y1 - y0) * a
    if abs(x1 - x0) > 1e-6 or abs(y1 - y0) > 1e-6:
        state["yaw"] = math.atan2(y1 - y0, x1 - x0)
    return x, y, state["yaw"]


# 재생 시계는 벽시계 기반 — 프레임당 고정 증가(1/60)는 헤드리스에서 프레임률이
# 60을 넘으면 과속 재생됨 (92s 재생이 10s 만에 끝난 실측). dt 클램프 0.1s는
# 로딩 스톨 직후 점프 방지.
T_END = max(st["wp"][-1][0] for st in robots.values())
sim_t = 0.0
n_play = 0
last = time.monotonic()
while sim_app.is_running():
    if sim_t > T_END + 2.0:
        if not args.loop:
            break
        sim_t = 0.0                                # 반복 재생 — 보간 상태 리셋
        n_play += 1
        for st in robots.values():
            st["idx"] = 0
            st["yaw"] = 0.0
        print(f"[isaac_replay] 재생 반복 {n_play}회", flush=True)
    for rid, st in robots.items():
        x, y, yaw = pose_at(st, sim_t)
        set_pose(st, x, y, yaw)
    step()
    now = time.monotonic()
    sim_t += min(now - last, 0.1) * args.speed
    last = now

print(f"[isaac_replay] 재생 종료 (시뮬 {T_END:.0f}s)")
sim_app.close()
