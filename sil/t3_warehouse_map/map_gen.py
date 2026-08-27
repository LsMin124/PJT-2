# ============================================================
# DXF 도면 → Occupancy Grid → 렉/AMR 배치 (v6.2 — 검증 반영)
#
# 변경점(v5 → v6):
#   - 렉 유닛을 NVIDIA 에셋 실측값으로 확정: 16.13 x 1.08 m (4m x 4베이)
#   - 렉 통로(AISLE) 2.5 m 로 지정
#   - 피킹 스테이션을 주 통로 한복판 → 렉 통로 입구 버퍼(양측 6개)로 이동,
#     충전독 앞 2.3m에 도킹 대기점 6개 추가
#   - 16.13 m 유닛은 기둥 행 사이(y방향 약 9m)에 들어가지 않으므로
#     렉 행을 90도 회전해 x방향(96m 길이 방향)으로 배치.
#     실제 기둥은 기둥 '행'(y=57.3)에만 있으므로 행 y위치만 피하면
#     x방향으로는 기둥 간섭이 없음 (도면의 얇은 축선은 장애물이 아님).
#
# 변경점(v6.1 → v6.2, 그리드 검증 반영 2026-08-27):
#   - 축선 레이어(DWGshare.com_4) 제외 — 6m 격자 축선이 장애물로 래스터화되어
#     팽창 후 자유공간이 143조각으로 파편화되던 문제 (제외 후 실내 단일 성분·통로 전 관통)
#   - 기둥 행 갭 통로(y 57.3) 폐지 — 실기둥 18개소 + 팽창으로 전 구간 차단.
#     기둥-랙면 물리 폭 1.24/1.44m < 최소 통로 폭 1.5m (T2 실측 규칙)
#   - 인계W-S 스테이션 (31.6,39.35)→(31.6,40.8) — 기둥(축선 x32.1)·컨베이어
#     팽창역 내부였음. 새 지점은 동서북 6m+ 여유, 남측 1.4m(도킹 방향)
#   - 시각화 파일 저장(map_layout.png / map_planner.png) — 헤드리스 서버 대응.
#     플래너 뷰(팽창 마스크+도달성+스테이션 자가검증) 신설
#
# 셀 값: 0=빈 공간, 1=구조체, 2=렉, 3=AMR 스테이션, 4=출입문, 5=컨베이어
# ============================================================
import time

import ezdxf
from ezdxf import path
import numpy as np
from scipy.ndimage import binary_dilation, label
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.patches import Patch

# ------------------------------------------------------------
# 설정
# ------------------------------------------------------------
DXF_FILE = "Factory_Building_Design.dxf"
CELL = 100                                  # mm/cell
X0, X1 = 660_000, 775_000                   # 크롭 (mm)
Y0, Y1 = 782_000, 888_000
SKIP_TYPES = {"TEXT", "MTEXT", "DIMENSION", "LEADER", "HATCH", "INSERT"}
SKIP_LAYERS = {"DWGshare.com_4"}            # 축선(6m 격자) 레이어 — 물리 장애물 아님

# --- NVIDIA 렉 에셋 (실측 확정) ---
RACK_UNIT_L = 16.13   # 렉 길이 [m] (4m x 4베이)
RACK_UNIT_D = 1.08    # 렉 깊이 [m]
DOUBLE_ROW = True     # 등맞대기 2줄
AISLE = 4.0           # ★ 렉 사이 통로 [m] (교행 가능)
ROW_W = RACK_UNIT_D * 2 if DOUBLE_ROW else RACK_UNIT_D   # 행 폭 2.16 m

# --- 배치 구획 ---
BANK = (47.0, 68.0)          # 중앙 렉 뱅크 y구간 (주 통로 사이)
COL_ROW_Y = 57.3             # 내부 기둥 행 y
COL_KEEP = 0.9               # 기둥 행 상하 이격 (기둥 0.4/2 + 클리어 0.5 + 여유)
STAGE_L, STAGE_R = 30.0, 102.0

# --- AMR (사용자 스펙) ---
AMR_L, AMR_W = 1.44, 0.641
AMR_R = (AMR_L**2 + AMR_W**2) ** 0.5 / 2    # 0.79 m
INFLATE = 0.8

# --- 문 / 컨베이어 / 스테이션 ---
DOORS = [(xw, yd, 1.4, 6.4) for xw in (13.2, 109.6) for yd in (38.3, 70.3)]
CONV_W = 0.9
CONVEYORS = [(14.6, 38.9, 16.0, CONV_W), (14.6, 70.9, 16.0, CONV_W),
             (94.6, 44.0 - CONV_W, 15.0, CONV_W), (94.6, 76.0 - CONV_W, 15.0, CONV_W)]
# 렉 통로 중심선: 통로1 y 51.78 (행1-2 사이), 통로2 y 63.02 (행3-4 사이)
# ※ 기둥 행 갭(y 57.3)은 통로로 못 씀 — 실기둥+팽창으로 전 구간 차단, 물리 폭
#   1.24/1.44m < 최소 1.5m (T2 실측 규칙). 통로 간 횡단은 렉 뱅크 양끝으로 우회
AISLE_Y = (51.78, 63.02)
AMR_STATIONS = (
    [(31.6, 40.8), (31.6, 71.35), (93.6, 43.55), (93.6, 75.55)]    # 인계(컨베이어 끝단; W-S는 기둥·컨베이어 팽창역 북측)
    + [(107.8, 50 + i * 3) for i in range(6)]                      # 충전독
    + [(105.5, 50 + i * 3) for i in range(6)]                      # 충전 대기점(독 앞 2.3m)
    + [(32.2, y) for y in AISLE_Y]                                 # 렉 통로 서측 입구 버퍼
    + [(99.8, y) for y in AISLE_Y]                                 # 렉 통로 동측 입구 버퍼
)
PALLET_PITCH = 1.15
RACK_LEVELS = 4

# ------------------------------------------------------------
# 1) DXF → 그리드
# ------------------------------------------------------------
t0 = time.time()
doc = ezdxf.readfile(DXF_FILE)
msp = doc.modelspace()
segments = []
for e in msp:
    if e.dxftype() in SKIP_TYPES or e.dxf.layer in SKIP_LAYERS:
        continue
    try:
        pts = [(v.x, v.y) for v in path.make_path(e).flattening(distance=CELL / 2)]
    except Exception:
        continue
    pts = [(x, y) for x, y in pts if X0 <= x <= X1 and Y0 <= y <= Y1]
    if len(pts) >= 2:
        segments.append(np.asarray(pts))

cols = int((X1 - X0) / CELL) + 1
rows = int((Y1 - Y0) / CELL) + 1
grid = np.zeros((rows, cols), dtype=np.uint8)
for pts in segments:
    for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
        n = max(int(np.hypot(x2 - x1, y2 - y1) / (CELL / 2)), 1)
        t = np.linspace(0.0, 1.0, n + 1)
        cs = np.clip(((x1 + (x2 - x1) * t - X0) / CELL).astype(int), 0, cols - 1)
        rs = np.clip(((y1 + (y2 - y1) * t - Y0) / CELL).astype(int), 0, rows - 1)
        grid[rs, cs] = 1
print(f"[1] 래스터화: {rows} x {cols}, {time.time() - t0:.1f}초")

M = 1000 / CELL


def rect_slice(x, y, w, h):
    return (slice(int(y * M), int((y + h) * M)), slice(int(x * M), int((x + w) * M)))


def add_rect(g, x, y, w, h, val):
    r, c = rect_slice(x, y, w, h)
    sub = g[r, c]
    sub[sub == 0] = val


def add_point(g, x, y, val, s=1.2):
    r, c = rect_slice(x - s / 2, y - s / 2, s, s)
    g[r, c] = val


# ------------------------------------------------------------
# 2) 렉 행 배치 (x방향, 기둥 행 회피, 유닛 스냅)
# ------------------------------------------------------------
strips = [(BANK[0], COL_ROW_Y - COL_KEEP), (COL_ROW_Y + COL_KEEP, BANK[1])]
rack_rows = []   # (x_start, y_bottom, n_units)
span = STAGE_R - STAGE_L
n_units = int(span // RACK_UNIT_L)
used_x = n_units * RACK_UNIT_L
xs = STAGE_L + (span - used_x) / 2
for (s0, s1) in strips:
    H = s1 - s0
    n_rows = int((H + AISLE) // (ROW_W + AISLE))
    used_y = n_rows * ROW_W + (n_rows - 1) * AISLE
    off = (H - used_y) / 2
    for k in range(n_rows):
        yb = s0 + off + k * (ROW_W + AISLE)
        add_rect(grid, xs, yb, used_x, ROW_W, 2)
        rack_rows.append((round(xs, 2), round(yb, 2), n_units))

# ------------------------------------------------------------
# 3) 컨베이어 / 스테이션 / 문
# ------------------------------------------------------------
for cv in CONVEYORS:
    add_rect(grid, *cv, val=5)
for s in AMR_STATIONS:
    add_point(grid, *s, val=3)
for d in DOORS:
    add_rect(grid, *d, val=4)

# ------------------------------------------------------------
# 통계 / 저장
# ------------------------------------------------------------
total_units = sum(n for _, _, n in rack_rows)
assets = total_units * (2 if DOUBLE_ROW else 1)
row_len = total_units * RACK_UNIT_L
pallets = int(row_len / PALLET_PITCH) * 2 * RACK_LEVELS
print(f"[2] 렉 행 {len(rack_rows)}개(더블), 유닛 {total_units} → 에셋 {assets}개, 연장 {row_len:.1f}m")
print(f"    파렛트 추정 {pallets:,}개")
print(f"    렉통로 {AISLE}m → 팽창({INFLATE}m) 후 유효 {AISLE - 2 * INFLATE:.1f}m (양방향 교행 가능)")

obstacle = binary_dilation((grid == 1) | (grid == 2) | (grid == 5),
                           iterations=int(INFLATE * M))
np.save("occupancy_grid.npy", grid)
np.save("obstacle_mask.npy", obstacle)
np.save("rack_rows.npy", np.array(rack_rows))   # Isaac Sim 배치 좌표 (x시작, y하단, 유닛수)
print("[3] occupancy_grid / obstacle_mask / rack_rows 저장")

# ------------------------------------------------------------
# 시각화
# ------------------------------------------------------------
cmap = ListedColormap(["white", "#222222", "#e67e22", "#2e86de", "#27ae60", "#8e44ad"])
plt.figure(figsize=(11, 11))
plt.imshow(grid, cmap=cmap, vmin=0, vmax=5, origin="lower",
           extent=[0, (X1 - X0) / 1000, 0, (Y1 - Y0) / 1000])
plt.legend(handles=[
    Patch(color="#222222", label="structure"),
    Patch(color="#e67e22", label=f"NVIDIA rack 16.13x1.08m x{assets}"),
    Patch(color="#2e86de", label="AMR station"),
    Patch(color="#27ae60", label="door 6.4x4.5m"),
    Patch(color="#8e44ad", label="conveyor"),
], loc="upper right")
plt.title(f"NVIDIA rack layout - aisle {AISLE}m, {assets} assets, ~{pallets:,} pallets")
plt.xlabel("m")
plt.ylabel("m")
plt.tight_layout()
plt.savefig("map_layout.png", dpi=140)

# ------------------------------------------------------------
# 플래너 뷰 — 팽창 마스크 + 도달성 + 스테이션 자가검증
# (시드 = 통로1 중앙. 흰색=도달 가능 자유공간, 살구색=자유지만 미도달, 검정=팽창 장애물)
# ------------------------------------------------------------
lab, _ = label(~obstacle)
seed = lab[int(AISLE_Y[0] * M), int(60 * M)]
reach = lab == seed
n_ok = sum(1 for x, y in AMR_STATIONS if lab[int(y * M), int(x * M)] == seed)
view = np.zeros_like(grid)
view[~reach] = 1
view[obstacle] = 2
plt.figure(figsize=(11, 11))
plt.imshow(view, cmap=ListedColormap(["white", "#f2c9a0", "#222222"]), vmin=0, vmax=2,
           origin="lower", extent=[0, (X1 - X0) / 1000, 0, (Y1 - Y0) / 1000])
plt.plot([x for x, _ in AMR_STATIONS], [y for _, y in AMR_STATIONS],
         "s", color="#2e86de", ms=5)
plt.title(f"planner view - reachable {reach.sum() / M / M:,.0f} m2, "
          f"stations {n_ok}/{len(AMR_STATIONS)} OK")
plt.xlabel("m")
plt.ylabel("m")
plt.tight_layout()
plt.savefig("map_planner.png", dpi=140)
print(f"[4] 시각화 저장: map_layout.png / map_planner.png · 스테이션 도달성 {n_ok}/{len(AMR_STATIONS)}")

plt.show()