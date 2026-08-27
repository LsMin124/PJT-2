"""T3 씬 빌더 — occupancy grid + rack_rows를 Isaac USD 씬으로 세운다.

원칙: 그리드가 곧 씬이다 (DES와 SIL의 단일 소스).
  - 벽·기둥(셀값 1): 그리디 메싱으로 병합한 박스 (높이 3.0m)
  - 컨베이어(셀값 5): 병합 박스 (높이 0.9m — 라이다 평면 위)
  - 렉(rack_rows.npy): NVIDIA 부품 조립 — SM_RackFrame_03(0.127x1.0x3.0) 5개
    + SM_RackShelf_01(4.0x1.08 데크) 4베이 x 3데크, 유닛 16.13m, 더블(등맞대기 2줄)
  - V&V: isaacsim.asset.gen.omap으로 씬→점유맵 재생성 후 원 그리드와 diff

실행:
  cd ~/isaacsim && ./python.sh <repo>/sil/t3_warehouse/build_scene.py
전제: sil/t3_warehouse_map/에서 map_gen.py 실행 완료(npy 존재).
"""

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import os
import time

from isaacsim.core.experimental.utils.app import enable_extension

import numpy as np
import omni.physx
import omni.timeline
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics

HERE = os.path.dirname(os.path.abspath(__file__))
MAP_DIR = os.path.join(HERE, "..", "t3_warehouse_map")
OUT_DIR = os.path.join(HERE, "out")
os.makedirs(OUT_DIR, exist_ok=True)

ASSETS = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0"
SHELF_USD = ASSETS + "/Isaac/Environments/Simple_Warehouse/Props/SM_RackShelf_01.usd"
FRAME_USD = ASSETS + "/Isaac/Environments/Simple_Warehouse/Props/SM_RackFrame_03.usd"

CELL = 0.1                      # m/셀 (map_gen과 동일)
WALL_H = 3.0
CONV_H = 0.9                    # 라이다 평면(0.7m) 위 — 가시
RACK_UNIT_L = 16.13
RACK_D = 1.08
BAY = 4.0                       # 베이 피치 (16.13 = 4x4.0 + 0.13 프레임 마진)
DECK_Z = (1.0, 1.75, 2.5)       # 데크 높이 3 + 바닥 = 4단 (RACK_LEVELS)

grid = np.load(os.path.join(MAP_DIR, "occupancy_grid.npy"))
rack_rows = np.load(os.path.join(MAP_DIR, "rack_rows.npy"))
ROWS, COLS = grid.shape
print(f"[in] grid {ROWS}x{COLS}, rack_rows {len(rack_rows)}행(블록)")


def greedy_rects(mask):
    """이진 마스크 → 병합 직사각형 (r0, c0, h, w) 목록. 행 런 → 동일 런 수직 병합."""
    rects = []
    open_runs = {}                      # (c0, c1) → [r_start, r_last]
    for r in range(mask.shape[0]):
        row = mask[r]
        runs = []
        c = 0
        while c < mask.shape[1]:
            if row[c]:
                c0 = c
                while c < mask.shape[1] and row[c]:
                    c += 1
                runs.append((c0, c))
            else:
                c += 1
        nxt = {}
        for run in runs:
            if run in open_runs and open_runs[run][1] == r - 1:
                open_runs[run][1] = r
                nxt[run] = open_runs[run]
            else:
                nxt[run] = [r, r]
        for run, (r0, r1) in open_runs.items():
            if run not in nxt:
                rects.append((r0, run[0], r1 - r0 + 1, run[1] - run[0]))
        open_runs = nxt
    for run, (r0, r1) in open_runs.items():
        rects.append((r0, run[0], r1 - r0 + 1, run[1] - run[0]))
    return rects


def add_box(stage, path, x, y, w, h, z0, z1):
    """(x,y)~(x+w,y+h), 높이 z0~z1 박스 + 콜라이더."""
    cube = UsdGeom.Cube.Define(stage, path)
    cube.GetSizeAttr().Set(1.0)
    xf = UsdGeom.Xformable(cube.GetPrim())
    xf.AddTranslateOp().Set(Gf.Vec3d(x + w / 2, y + h / 2, (z0 + z1) / 2))
    xf.AddScaleOp().Set(Gf.Vec3f(w, h, z1 - z0))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())


t0 = time.time()
# 저작은 렌더러와 분리된 순수 USD 스테이지에서 — 참조를 하나씩 붙이며 app.update()를
# 돌리면 로딩 중 hydra 경로에서 간헐 세그폴트(3회 재현). 완성 파일을 한 번에 열면 안정.
usd_path = os.path.join(HERE, "warehouse_scene.usd")
if os.path.exists(usd_path):
    os.remove(usd_path)
stage = Usd.Stage.CreateNew(usd_path)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
UsdPhysics.Scene.Define(stage, Sdf.Path("/physicsScene"))
UsdGeom.Xform.Define(stage, "/World")
stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))

# 바닥 + 조명
add_box(stage, "/World/ground", -5, -5, COLS * CELL + 10, ROWS * CELL + 10, -0.1, 0.0)
light = UsdLux.DistantLight.Define(stage, "/World/sun")
light.CreateIntensityAttr(2500)
UsdGeom.Xformable(light.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(0, -35, 25))
dome = UsdLux.DomeLight.Define(stage, "/World/dome")
dome.CreateIntensityAttr(400)

# 1) 벽·기둥 (셀값 1)
walls = greedy_rects(grid == 1)
UsdGeom.Xform.Define(stage, "/World/walls")
for i, (r0, c0, h, w) in enumerate(walls):
    add_box(stage, f"/World/walls/w_{i}", c0 * CELL, r0 * CELL, w * CELL, h * CELL, 0.0, WALL_H)
print(f"[1] 벽·기둥 박스 {len(walls)}개")

# 2) 컨베이어 (셀값 5)
convs = greedy_rects(grid == 5)
UsdGeom.Xform.Define(stage, "/World/conveyors")
for i, (r0, c0, h, w) in enumerate(convs):
    add_box(stage, f"/World/conveyors/c_{i}", c0 * CELL, r0 * CELL, w * CELL, h * CELL, 0.0, CONV_H)
print(f"[2] 컨베이어 박스 {len(convs)}개")

# 3) 렉 — 부품 조립 (rack_rows: [x시작, y하단, 유닛수] / 행폭 2xRACK_D 더블)
UsdGeom.Xform.Define(stage, "/World/racks")
n_frame = n_shelf = 0
for bi, (xs, yb, n_units) in enumerate(rack_rows):
    n_units = int(n_units)
    for line in range(2):                              # 등맞대기 2줄
        yc = yb + RACK_D * (line + 0.5)                # 줄 중심 y
        for u in range(n_units):
            x0 = xs + u * RACK_UNIT_L
            root = f"/World/racks/b{bi}_l{line}_u{u}"
            UsdGeom.Xform.Define(stage, root)
            for k in range(5):                         # 프레임 5 (베이 경계)
                p = UsdGeom.Xform.Define(stage, f"{root}/frame_{k}")
                p.GetPrim().GetReferences().AddReference(FRAME_USD)
                UsdGeom.Xformable(p.GetPrim()).AddTranslateOp().Set(
                    Gf.Vec3d(x0 + 0.065 + BAY * k, yc, 0.0))
                n_frame += 1
            for k in range(4):                         # 베이 4 x 데크 3
                cx = x0 + 0.065 + BAY * k + BAY / 2
                for dz in DECK_Z:
                    p = UsdGeom.Xform.Define(stage, f"{root}/shelf_{k}_{int(dz*100)}")
                    p.GetPrim().GetReferences().AddReference(SHELF_USD)
                    # 회전 불필요 — 참조 컴포즈된 데크는 이미 4.0(x)x1.08(y) 행 정렬.
                    # (에셋 파일을 직접 열어 잰 bbox는 축이 뒤바뀐 값을 줌 — 반드시
                    # 참조로 컴포즈한 상태에서 잴 것. omap V&V가 이 오배치를 잡아냈다)
                    UsdGeom.Xformable(p.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(cx, yc, dz))
                    n_shelf += 1
print(f"[3] 렉 조립: 프레임 {n_frame} + 데크 {n_shelf}", flush=True)

# 4) 저작 저장 → 완성 파일을 컨텍스트로 오픈 (참조 일괄 로딩)
stage.GetRootLayer().Save()
del stage
print(f"[4] 저장: {usd_path} ({os.path.getsize(usd_path)//1024}KB) · 저작 {time.time()-t0:.0f}s", flush=True)

ctx = omni.usd.get_context()
ctx.open_stage(usd_path)
while ctx.get_stage_loading_status()[2] > 0:
    app.update()
for _ in range(30):
    app.update()
stage = ctx.get_stage()

# 렉 메시 콜라이더 보장 (에셋에 이미 있으면 0건)
n_col = 0
for prim in stage.Traverse():
    if prim.IsA(UsdGeom.Mesh) and str(prim.GetPath()).startswith("/World/racks"):
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI.Apply(prim)
            n_col += 1
print(f"[4b] 오픈·로딩 완료, 렉 메시 콜라이더 적용 {n_col}건 · {time.time()-t0:.0f}s", flush=True)

# 5) V&V — omap으로 씬→점유맵 재생성, 원 그리드와 diff
# ※ omap 확장은 반드시 스테이지 로딩 "후"에 활성화 — 선활성화 상태로 대형 씬을
#   열면 로딩 중 간헐 세그폴트 (기본 비활성 확장)
enable_extension("isaacsim.asset.gen.omap")
for _ in range(5):
    app.update()
from isaacsim.asset.gen.omap.bindings import _omap

timeline = omni.timeline.get_timeline_interface()
timeline.play()
for _ in range(10):
    app.update()
gen = _omap.Generator(omni.physx.get_physx_interface(), ctx.get_stage_id())
gen.update_settings(CELL, 4, 5, 6)                     # 셀 0.1 / 점유4 자유5 미지6
gen.set_transform((0, 0, 0), (0, 0, 0.2), (COLS * CELL, ROWS * CELL, 1.2))
gen.generate2d()
buf = np.array(gen.get_buffer())
timeline.stop()
occ = None
if buf.size:
    dims = (int(round(ROWS)), int(round(COLS)))
    for shape in (dims, dims[::-1]):
        if buf.size == shape[0] * shape[1]:
            occ = (buf.reshape(shape) == 4)
            if shape != dims:
                occ = occ.T
            occ = np.fliplr(occ)                       # omap 버퍼는 x 미러 (실측: 보정 시 벽 재현율 100%)
            break
if occ is None:
    print(f"[5] omap 버퍼 크기 불일치({buf.size}) — diff 생략")
else:
    from scipy.ndimage import binary_dilation
    st = (grid == 1) | (grid == 5)                     # 벽·컨베이어: 셀 단위 일치 기대
    tol = binary_dilation(occ, iterations=2)
    cover = tol[st].mean() * 100
    rackmask = np.zeros_like(st)
    for xs, yb, n_units in rack_rows:
        r0, r1 = int(yb / CELL), int((yb + 2 * RACK_D) / CELL)
        c0, c1 = int(xs / CELL), int((xs + int(n_units) * RACK_UNIT_L) / CELL)
        rackmask[r0:r1, c0:c1] = True
    rack_hit = occ[rackmask].mean() * 100
    fp = occ & ~binary_dilation(st | rackmask, iterations=3)
    print(f"[5] V&V — 벽·컨베이어 재현율 {cover:.1f}% · 렉 풋프린트 내 점유(다리·데크 두께) {rack_hit:.1f}%"
          f" · 풋프린트 밖 오검출 {fp.sum()}셀")
    np.save(os.path.join(OUT_DIR, "omap_occ.npy"), occ)

# 6) 스크린샷 — 탑뷰 + 퍼스펙티브
from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport

cam = UsdGeom.Camera.Define(stage, "/World/cam")
cam.CreateClippingRangeAttr(Gf.Vec2f(0.1, 2000))
vp = get_active_viewport()
vp.camera_path = "/World/cam"


def shoot(pos, rot, fname, focal=18.0):
    cam.CreateFocalLengthAttr(focal)
    xf = UsdGeom.Xformable(cam.GetPrim())
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    xf.AddRotateXYZOp().Set(Gf.Vec3f(*rot))
    for _ in range(45):
        app.update()
    path = os.path.join(OUT_DIR, fname)
    capture_viewport_to_file(vp, path)
    for _ in range(200):                               # 캡처는 비동기 저장 — 파일 생성까지 폴링
        app.update()
        if os.path.exists(path):
            break
    print(f"[6] {fname} {'OK' if os.path.exists(path) else '캡처 실패'}")


shoot((COLS * CELL / 2, ROWS * CELL / 2, 120), (0, 0, 0), "scene_top.png", focal=24.0)
shoot((20, 20, 22), (62, 0, -38), "scene_persp.png")
shoot((60, 47, 1.2), (87, 0, -90 + 12), "scene_aisle.png", focal=14.0)

app.close()
print(f"[done] 총 {time.time()-t0:.0f}s")
