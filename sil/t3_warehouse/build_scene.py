"""T3 씬 빌더 — occupancy grid + rack_units를 Isaac USD 씬으로 세운다 (v5.6 맵).

원칙: 그리드가 곧 씬이다 (DES와 SIL의 단일 소스).
  - 벽·기둥(셀값 1): 그리디 메싱으로 병합한 박스 (높이 8.0m)
  - 컨베이어·작업대(셀값 5): 폭으로 구분 — 폭 0.9m 컨베이어는 투명 콜라이더
    + ConveyorBelt_A08 비주얼, 폭 1.2m+ 작업대는 가시 박스 (높이 0.9m — 라이다 평면 위)
  - 렉(rack_units.npy): 세로형 더블로우 — [x중심, y시작, 유닛수], 유닛 4.0m =
    SM_RackShelf_01 베이 1개와 정확 일치. 부품을 z축 90° 회전해 y축 정렬 조립
  - V&V: isaacsim.asset.gen.omap으로 씬→점유맵 재생성 후 원 그리드와 diff

실행:
  cd ~/isaacsim && ./python.sh <repo>/sil/t3_warehouse/build_scene.py
전제: sil/t3_warehouse_map/map/에서 warehouse_layout_v5_5_final.py 실행 완료(npy 존재).
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
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

HERE = os.path.dirname(os.path.abspath(__file__))
MAP_DIR = os.path.join(HERE, "..", "t3_warehouse_map", "map")
OUT_DIR = os.path.join(HERE, "out")
os.makedirs(OUT_DIR, exist_ok=True)

ASSETS = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0"
SHELF_USD = ASSETS + "/Isaac/Environments/Simple_Warehouse/Props/SM_RackShelf_01.usd"
FRAME_USD = ASSETS + "/Isaac/Environments/Simple_Warehouse/Props/SM_RackFrame_03.usd"

MAT_DIR = ASSETS + "/Isaac/Environments/Simple_Warehouse/Materials"

CELL = 0.1                      # m/셀 (맵 스크립트와 동일)
WALL_H = 8.0                    # 산업 표준고 가정 (도면에 층고 정보 없음 — 시각용, 라이다 무관)
CONV_H = 0.9                    # 라이다 평면(0.7m) 위 — 가시(콜라이더)
TABLE_H = 0.9                   # 작업대 높이 — 라이다 평면 위 (0.6m 큐브 관통 사고 교훈)

# 컨베이어 비주얼 — ConveyorBelt_A08 직선 섹션 (컴포즈 실측 2.719x1.15x1.17, 피벗 동측 끝)
CONV_USD = ASSETS + "/Isaac/Props/Conveyors/ConveyorBelt_A08.usd"
CONV_SEC_L = 2.719
# (x0, y0, 길이, 세로 여부) — 맵 v5.6 CONVEYORS/CONVEYORS_V와 동일 좌표
CONVEYORS_VIS = [(14.6, 38.9, 16.0, False), (14.6, 70.9, 16.0, False),
                 (94.6, 43.1, 15.0, False), (94.6, 75.1, 15.0, False),
                 (101.0, 34.4, 7.4, False),                 # 패킹→출고 연결 (동진)
                 (107.5, 35.3, 7.8, True)]                  # 패킹→출고 연결 (북상)
UNIT_L = 4.0                    # 렉 유닛 길이 (y) = SM_RackShelf 베이 1개
RACK_D = 1.08                   # 데크 실측 깊이 (그리드 선언 1.2 — 풋프린트 내 배치)
RACK_W = 2.4                    # 더블로우 그리드 폭 (맵 RACK_UNIT_D 1.2 x 2)
DECK_Z = (1.0, 1.75, 2.5)       # 데크 높이 3 + 바닥 = 4단 (RACK_LEVELS)

grid = np.load(os.path.join(MAP_DIR, "occupancy_grid.npy"))
rack_units = np.load(os.path.join(MAP_DIR, "rack_units.npy"))
ROWS, COLS = grid.shape
print(f"[in] grid {ROWS}x{COLS}, rack_units {len(rack_units)}세그(세로형)")


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


def add_box(stage, path, x, y, w, h, z0, z1, collide=True):
    """(x,y)~(x+w,y+h), 높이 z0~z1 박스 (+콜라이더)."""
    cube = UsdGeom.Cube.Define(stage, path)
    cube.GetSizeAttr().Set(1.0)
    xf = UsdGeom.Xformable(cube.GetPrim())
    xf.AddTranslateOp().Set(Gf.Vec3d(x + w / 2, y + h / 2, (z0 + z1) / 2))
    xf.AddScaleOp().Set(Gf.Vec3f(w, h, z1 - z0))
    if collide:
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    return cube


def add_box_mesh(stage, path, x, y, w, h, z0, z1, uv_scale=4.0):
    """타일링 UV를 가진 박스 메시(옆 4면+윗면) + 콜라이더 — Cube prim은 UV가 없어
    텍스처 재질이 단색으로 뭉개진다(실측). 벽·기둥용."""
    mesh = UsdGeom.Mesh.Define(stage, path)
    x1, y1 = x + w, y + h
    pts = [(x, y, z0), (x1, y, z0), (x1, y1, z0), (x, y1, z0),
           (x, y, z1), (x1, y, z1), (x1, y1, z1), (x, y1, z1)]
    faces = [(0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7), (4, 5, 6, 7)]
    mesh.CreatePointsAttr(pts)
    mesh.CreateFaceVertexCountsAttr([4] * 5)
    mesh.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
    mesh.CreateExtentAttr([(x, y, z0), (x1, y1, z1)])
    mesh.CreateDoubleSidedAttr(True)
    hz = (z1 - z0) / uv_scale
    st = []
    for d in (w, h, w, h):
        u = d / uv_scale
        st += [(0, 0), (u, 0), (u, hz), (0, hz)]
    st += [(0, 0), (w / uv_scale, 0), (w / uv_scale, h / uv_scale), (0, h / uv_scale)]
    UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying).Set(st)
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    return mesh


def add_quad(stage, path, x0, y0, x1, y1, z, uv_scale, flip=False):
    """수평 사각 메시 + 타일링 UV (uv_scale m당 텍스처 1회) — 바닥·천장 시각용."""
    mesh = UsdGeom.Mesh.Define(stage, path)
    pts = [(x0, y0, z), (x1, y0, z), (x1, y1, z), (x0, y1, z)]
    mesh.CreatePointsAttr(pts)
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 3, 2, 1] if flip else [0, 1, 2, 3])
    mesh.CreateExtentAttr([(x0, y0, z - 0.01), (x1, y1, z + 0.01)])
    mesh.CreateDoubleSidedAttr(True)
    u, v = (x1 - x0) / uv_scale, (y1 - y0) / uv_scale
    st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
    st.Set([(0, 0), (u, 0), (u, v), (0, v)])
    return mesh


def bind_omnipbr(stage, prim, name, diffuse_tex, normal_tex, tint):
    """OmniPBR + 원본 텍스처 + 틴트 — 순정 MDL은 톤 조절 입력을 못 믿어 직접 조립."""
    path = f"/World/Looks/{name}"
    mtl = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + "/Shader")
    sh.CreateImplementationSourceAttr(UsdShade.Tokens.sourceAsset)
    sh.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
    sh.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    sh.CreateInput("diffuse_texture", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(diffuse_tex))
    sh.CreateInput("normalmap_texture", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(normal_tex))
    sh.CreateInput("diffuse_tint", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*tint))
    sh.CreateInput("reflection_roughness_constant", Sdf.ValueTypeNames.Float).Set(0.75)
    out = sh.CreateOutput("out", Sdf.ValueTypeNames.Token)
    mtl.CreateSurfaceOutput("mdl").ConnectToSource(out)
    UsdShade.MaterialBindingAPI.Apply(prim.GetPrim()).Bind(mtl)


def bind_mdl(stage, prim, name, mdl_file):
    """Simple_Warehouse MDL을 머티리얼로 정의해 prim(하위 상속)에 바인딩."""
    path = f"/World/Looks/{name}"
    mtl = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + "/Shader")
    sh.CreateImplementationSourceAttr(UsdShade.Tokens.sourceAsset)
    sh.SetSourceAsset(Sdf.AssetPath(mdl_file), "mdl")
    sh.SetSourceAssetSubIdentifier(name, "mdl")
    out = sh.CreateOutput("out", Sdf.ValueTypeNames.Token)
    mtl.CreateSurfaceOutput("mdl").ConnectToSource(out)
    UsdShade.MaterialBindingAPI.Apply(prim.GetPrim() if hasattr(prim, "GetPrim") else prim).Bind(mtl)


def add_asset(stage, path, usd, x, y, z=0.0, rot_z=None):
    """참조 에셋 배치 — 에셋 루트에 자체 xformOpOrder가 있어도 안전하도록
    래퍼 Xform이 이동·회전을 담당한다 (A08에서 실측한 예외의 일반화)."""
    w = UsdGeom.Xform.Define(stage, path)
    xf = UsdGeom.Xformable(w.GetPrim())
    xf.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
    if rot_z is not None:
        xf.AddRotateZOp().Set(rot_z)
    a = UsdGeom.Xform.Define(stage, path + "/asset")
    a.GetPrim().GetReferences().AddReference(usd)
    return w


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

# 바닥(충돌 박스는 표면 아래로 내려 시각 메시와 z-파이팅 방지) + 재질 + 조명
W, H = COLS * CELL, ROWS * CELL
add_box(stage, "/World/ground", -5, -5, W + 10, H + 10, -0.11, -0.01)
floor = add_quad(stage, "/World/floor", -5, -5, W + 5, H + 5, 0.0, uv_scale=4.0)
bind_mdl(stage, floor, "MI_Floor_01", MAT_DIR + "/MI_Floor_01.mdl")

# 개방 지붕 — 태양광(그림자·전역 밝기) + 조명 그리드(현수등, z 7.8~7.95 라이다 밴드 밖)
sun = UsdLux.DistantLight.Define(stage, "/World/sun")
sun.CreateIntensityAttr(1200)
UsdGeom.Xformable(sun.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(0, -35, 25))
UsdGeom.Xform.Define(stage, "/World/lights")
n_light = 0
for gx in range(18, 111, 12):
    for gy in range(28, 89, 12):
        L = UsdLux.RectLight.Define(stage, f"/World/lights/L_{gx}_{gy}")
        L.CreateIntensityAttr(30000)
        L.CreateExposureAttr(5.0)                         # x32 — 30000 단독으론 실내가 암흑 (실측)
        L.CreateWidthAttr(1.2)
        L.CreateHeightAttr(0.6)
        xfl = UsdGeom.Xformable(L.GetPrim())
        xfl.AddTranslateOp().Set(Gf.Vec3d(gx, gy, 7.8))   # RectLight는 기본 -Z(하향) 방사 — 회전 금지
        add_box(stage, f"/World/lights/fix_{gx}_{gy}", gx - 0.7, gy - 0.35, 1.4, 0.7,
                7.85, 7.95, collide=False)
        n_light += 1
fixtures = stage.GetPrimAtPath("/World/lights")
bind_mdl(stage, fixtures, "M_Glow", MAT_DIR + "/M_Glow.mdl")
dome = UsdLux.DomeLight.Define(stage, "/World/dome")
dome.CreateIntensityAttr(250)                             # 하늘 보조광
print(f"[0] 드레싱: 바닥 재질 + 태양 + 현수등 {n_light}기")

# 1) 벽·기둥 (셀값 1 — v5.6은 기둥도 구조체에 포함, 크기로 구분)
walls = greedy_rects(grid == 1)
walls_xf = UsdGeom.Xform.Define(stage, "/World/walls")
cols_xf = UsdGeom.Xform.Define(stage, "/World/columns")
n_colbox = 0
for i, (r0, c0, h, w) in enumerate(walls):
    small = w * CELL < 2.2 and h * CELL < 2.2             # 기둥(심볼 파편) — 별도 재질로 구분
    parent = "columns" if small else "walls"
    add_box_mesh(stage, f"/World/{parent}/w_{i}", c0 * CELL, r0 * CELL, w * CELL, h * CELL, 0.0, WALL_H)
    n_colbox += small
# 밝은 순백 벽이 눈부심 → 베이지 톤 (텍스처 디테일 유지, 기둥은 한 단계 짙게 구분)
TEX = MAT_DIR + "/Textures"
bind_omnipbr(stage, walls_xf, "WallBeige",
             TEX + "/T_WallA_01_D.png", TEX + "/T_WallA_01_N.png", (0.85, 0.78, 0.64))
bind_omnipbr(stage, cols_xf, "ColumnBeige",
             TEX + "/T_WallBoard_01_D.png", TEX + "/T_WallBoard_01_N.png", (0.78, 0.71, 0.59))
print(f"[1] 벽 {len(walls) - n_colbox} + 기둥 {n_colbox} (타일링 UV 메시, 베이지 톤)")

# 2) 컨베이어·작업대 (셀값 5) — rect 중심이 컨베이어 라인 밴드 위인지로 구분.
#    (폭 기준은 함정: qc 작업대가 컨베이어와 셀이 붙어 그리디 분할되면 0.6m 조각이
#    되어 컨베이어로 오분류 — 14/16개 실측 후 좌표 기준으로 교체)


def on_conveyor(cx, cy):
    for x0, y0, L, vert in CONVEYORS_VIS:
        if vert and x0 - 0.3 <= cx <= x0 + 1.2 and y0 - 0.3 <= cy <= y0 + L + 0.3:
            return True
        if not vert and x0 - 0.3 <= cx <= x0 + L + 0.3 and y0 - 0.3 <= cy <= y0 + 1.2:
            return True
    return False


convs = greedy_rects(grid == 5)
UsdGeom.Xform.Define(stage, "/World/conveyors")
UsdGeom.Xform.Define(stage, "/World/worktables")
n_conv = n_tab = 0
for i, (r0, c0, h, w) in enumerate(convs):
    if on_conveyor((c0 + w / 2) * CELL, (r0 + h / 2) * CELL):   # 컨베이어 — 콜라이더 전용(비주얼은 A08)
        b = add_box(stage, f"/World/conveyors/c_{i}", c0 * CELL, r0 * CELL,
                    w * CELL, h * CELL, 0.0, CONV_H)
        UsdGeom.Imageable(b.GetPrim()).MakeInvisible()
        n_conv += 1
    else:                                                 # 작업대 — 가시 박스 (V&V·플래너 동일 소스)
        b = add_box(stage, f"/World/worktables/t_{i}", c0 * CELL, r0 * CELL,
                    w * CELL, h * CELL, 0.0, TABLE_H)
        UsdGeom.Gprim(b.GetPrim()).CreateDisplayColorAttr([(0.35, 0.38, 0.42)])
        n_tab += 1
# 정적 비주얼 — 기능 없는 모양용 (V&V·플래너는 위 콜라이더 박스 기준 그대로)
n_sec = 0
for ci, (cx0, cy0, clen, vert) in enumerate(CONVEYORS_VIS):
    n = int(clen // CONV_SEC_L)
    s0 = (clen - n * CONV_SEC_L) / 2
    for k in range(n):
        off = s0 + k * CONV_SEC_L + CONV_SEC_L            # 에셋 피벗 = 진행측 끝 (x -2.719~0)
        if vert:                                          # z 90° 회전 → 스팬이 -y 방향
            add_asset(stage, f"/World/conveyors/vis{ci}_{k}", CONV_USD,
                      cx0 + 0.45, cy0 + off, rot_z=90.0)
        else:
            add_asset(stage, f"/World/conveyors/vis{ci}_{k}", CONV_USD,
                      cx0 + off, cy0 + 0.45)
        n_sec += 1
print(f"[2] 컨베이어 콜라이더 {n_conv}(투명) + 비주얼 섹션 {n_sec} · 작업대 {n_tab}")

# 3) 렉 — 부품 조립 (rack_units: [x중심, y시작, 유닛수] — 세로형 더블로우)
#    유닛 4.0m = 데크(베이) 1개와 정확 일치. 컴포즈된 데크는 4.0(x)x1.08(y)로
#    가로 정렬이므로 세로형에선 z축 90° 회전이 "필요"하다 (v6.3 가로형은 회전 금지
#    였던 것과 반대 — 근거는 동일하게 참조 컴포즈 bbox 실측).
UsdGeom.Xform.Define(stage, "/World/racks")
n_frame = n_shelf = 0
for bi, (xc, ys, n_units) in enumerate(rack_units):
    n_units = int(n_units)
    for line in range(2):                              # 등맞대기 2줄 (x = xc ± 0.54, 등이 맞닿음)
        lx = xc + RACK_D * (line - 0.5)
        root = f"/World/racks/b{bi}_l{line}"
        UsdGeom.Xform.Define(stage, root)
        for k in range(n_units + 1):                   # 프레임 — 유닛 경계 공유 (n+1개)
            add_asset(stage, f"{root}/frame_{k}", FRAME_USD,
                      lx, ys + UNIT_L * k, rot_z=90.0)
            n_frame += 1
        for u in range(n_units):                       # 유닛당 데크 3 (+바닥 = 4단)
            yc = ys + UNIT_L * u + UNIT_L / 2
            for dz in DECK_Z:
                add_asset(stage, f"{root}/shelf_{u}_{int(dz*100)}", SHELF_USD,
                          lx, yc, z=dz, rot_z=90.0)
                n_shelf += 1
print(f"[3] 렉 조립: 프레임 {n_frame} + 데크 {n_shelf}", flush=True)

# 바닥 마킹 — 렉 세그먼트 안전선(황) + 스테이션 마킹(청): 시인성 + 레이아웃 데이터의 시각화
UsdGeom.Xform.Define(stage, "/World/markings")
n_mark = 0
LW = 0.12
for bi, (xc, ys, nu) in enumerate(rack_units):
    bx0, by0 = xc - RACK_W / 2 - 0.27, ys - 0.27
    bx1, by1 = xc + RACK_W / 2 + 0.27, ys + int(nu) * UNIT_L + 0.27
    for j, (qx0, qy0, qx1, qy1) in enumerate([
            (bx0, by0, bx1, by0 + LW), (bx0, by1 - LW, bx1, by1),
            (bx0, by0, bx0 + LW, by1), (bx1 - LW, by0, bx1, by1)]):
        q = add_quad(stage, f"/World/markings/rk{bi}_{j}", qx0, qy0, qx1, qy1, 0.01, 4.0)
        UsdGeom.Gprim(q.GetPrim()).CreateDisplayColorAttr([(1.0, 0.78, 0.05)])
        n_mark += 1
for i, (r0, c0, hh, ww) in enumerate(greedy_rects(grid == 3)):
    q = add_quad(stage, f"/World/markings/st{i}", c0 * CELL, r0 * CELL,
                 (c0 + ww) * CELL, (r0 + hh) * CELL, 0.012, 4.0)
    UsdGeom.Gprim(q.GetPrim()).CreateDisplayColorAttr([(0.15, 0.45, 0.9)])
    n_mark += 1
print(f"[3c] 바닥 마킹 {n_mark}개 (안전선·스테이션)")

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
    st = (grid == 1) | (grid == 5)                     # 벽·컨베이어·작업대: 셀 단위 일치 기대
    tol = binary_dilation(occ, iterations=2)
    cover = tol[st].mean() * 100
    rackmask = np.zeros_like(st)
    for xc, ys, n_units in rack_units:
        r0, r1 = int(ys / CELL), int((ys + int(n_units) * UNIT_L) / CELL)
        c0, c1 = int((xc - RACK_D) / CELL), int((xc + RACK_D) / CELL)
        rackmask[r0:r1, c0:c1] = True
    rack_hit = occ[rackmask].mean() * 100
    fp = occ & ~binary_dilation(st | rackmask, iterations=3)
    print(f"[5] V&V — 벽·컨베이어·작업대 재현율 {cover:.1f}% · 렉 풋프린트 내 점유(다리·데크 두께) {rack_hit:.1f}%"
          f" · 풋프린트 밖 오검출 {fp.sum()}셀")
    np.save(os.path.join(OUT_DIR, "omap_occ.npy"), occ)

# 6) 스크린샷 — 탑뷰 + 퍼스펙티브 + 통로 뷰(세로형 → 북향)
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
shoot((22, 38, 4.5), (75, 0, -35), "scene_persp.png")     # 남서측 실내 조감 (작업 라인+렉)
shoot((59.1, 45.5, 1.2), (87, 0, 0), "scene_aisle.png", focal=14.0)   # 통로5 북향

app.close()
print(f"[done] 총 {time.time()-t0:.0f}s")
