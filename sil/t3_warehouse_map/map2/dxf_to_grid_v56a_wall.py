# -*- coding: utf-8 -*-
# ============================================================
# DXF 도면 → Occupancy Grid (v5.6-A: "y38 벽 실재" 시나리오, 렉 무손실 재배치)
#
# 기준본 dxf_to_grid_v56.py와의 차이 — "y=38 이중선이 실제 벽"일 때의 A안:
#   [A-1] y=38.0~38.3m 전폭 이중선을 장애물로 유지 (v5.6은 통행 처리).
#   [A-2] 렉 뱅크 y47→47.6 시작으로 0.6m 북상. 래스터 실측 기준 하단
#         세그먼트 여유가 0.7m라(산술치 1m 아님) 0.6m가 무손실 한계.
#         유닛 48개 유지, 벽~렉 사이 남측 통로 8.7→9.3m 확보.
#   [A-3] 벽 남측에 있던 AMR 스테이션 12개소(인덕션2·합류2·패킹5·VAS3)를
#         벽 북측 면으로 이전: 작업대는 벽에 붙여 y38.45~, 도킹은 y41.4.
#         벽 남측은 인력 전용 구역(포장재·사무실 등)으로 남김.
#
# 검수 반영 4건 (2026-08-28):
#   - 도킹 라인 y40.4→41.4 일괄: VAS 작업대(깊이 1.3)의 팽창 상단이 40.55라
#     VAS 도킹 3개소가 자기 작업대 팽창역 안(free=False 실측)이었고,
#     인덕션~패킹도 여유가 1셀(0.05m)뿐이었다. 41.4는 작업대 팽창(40.55)과
#     출고 컨베이어 팽창(42.3) 밴드의 중앙.
#   - handoff W-S (31.6, 39.35)→(31.6, 40.8): 기둥(32.1, 38.0)+실벽 팽창역
#     간섭 — v6.3/v5.6에서 두 번 실측 확정된 좌표로 복귀(세 번째 회귀 차단).
#   - 자가검증을 원시 그리드→팽창 obstacle 기준으로 교체(위 4건이
#     원시 그리드 연결성 검사로는 전부 통과됐던 원인). 실패 시 exit 1.
#   - DXF 경로를 ../map/ 상대참조로(레포 구조에서 재현 가능하게).
#   [A-4] 패킹→출고 L자 컨베이어(벽 남측 경유) 제거, 벽 북측 연결
#         컨베이어(x106.4, y39.9~43.1)로 대체. 렉통로 남측 입구버퍼
#         y45.6→46.6 (렉과의 상대 간격 1.4m 유지).
#
# 산출물은 기준본과 분리: *_wallA.npy / stations_wallA.json /
# grid_v56a_wall_full.png (ROS map은 변형판에서 생략)
#
# 셀 값: 0=빈공간, 1=구조체, 2=렉, 3=스테이션, 4=문, 5=컨베이어·작업대
# 좌표계: 그리드[m], origin="lower" (원점 왼쪽 아래, y↑=북)
# ============================================================
import json
import os
import time

import ezdxf
from ezdxf import path
import numpy as np
from scipy.ndimage import binary_dilation

OUT = os.path.dirname(os.path.abspath(__file__))
DXF_FILE = os.path.join(OUT, "..", "map", "Factory_Building_Design.dxf")
KOREAN_FONT = next((p for p in (
    os.path.join(OUT, "font", "NotoSansCJK-Regular.ttc"),
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
) if os.path.exists(p)), None)
CELL = 100                                   # mm/cell (0.1 m)
X0, X1, Y0, Y1 = 660_000, 775_000, 782_000, 888_000
SKIP_TYPES = {"TEXT", "MTEXT", "DIMENSION", "LEADER", "HATCH", "INSERT"}
# 축선(기둥 중심선)·축선 번호 원 = 주석. 장애물 아님. (v5.6 수정1과 동일)
AXIS_LAYERS = {"DWGshare.com_4", "DWGshare.com_15"}

# 렉 — [A-2] 뱅크 시작 47→48 (유닛 무손실 1m 시프트)
RACK_UNIT_L, RACK_UNIT_D = 4.0, 1.2
DOUBLE_ROW = True
RACK_W = RACK_UNIT_D * 2 if DOUBLE_ROW else RACK_UNIT_D
CLEAR, THICK_THRESH = 0.5, 5
RACK_BANKS = [(47.6, 68)]
STAGE_L, STAGE_R = 30, 102

# AMR (1,440 x 641 mm → 외접반경 0.79 m)
INFLATE = 0.8

DOORS = [(xw, yd, 1.4, 6.4) for xw in (13.2, 109.6) for yd in (38.3, 70.3)]
CONV_W = 0.9
CONVEYORS = [
    (14.6, 38.9, 16.0, CONV_W),              # 입고 1 (벽 북측, 유지)
    (14.6, 70.9, 16.0, CONV_W),              # 입고 2
    (94.6, 44.0 - CONV_W, 15.0, CONV_W),     # 출고 1 (문1로)
    (94.6, 76.0 - CONV_W, 15.0, CONV_W),     # 출고 2 (파렛트존 보조)
]
# [A-4] 패킹→출고1 연결 (벽 북측, 세로 x106.4, y39.9→43.1)
CONVEYORS_V = [(106.4, 39.9, 0.9, 3.2)]
COL_ROWS_Y = [25.5, 31.8, 38.0, 57.3, 76.5, 83.0, 89.5]
PALLET_PITCH, RACK_LEVELS = 1.15, 4

# ------------------------------------------------------------
# 1) DXF → 그리드 ([A-1] y38 이중선 스킵 없음 = 벽으로 유지)
# ------------------------------------------------------------
t0 = time.time()
doc = ezdxf.readfile(DXF_FILE)
msp = doc.modelspace()
segments = []          # 장애물 (축선 제외)
axis_segments = []     # 축선 — 기둥 라인 검출 전용
for e in msp:
    if e.dxftype() in SKIP_TYPES:
        continue
    try:
        pts = [(v.x, v.y) for v in path.make_path(e).flattening(distance=CELL / 2)]
    except Exception:
        continue
    pts = [(x, y) for x, y in pts if X0 <= x <= X1 and Y0 <= y <= Y1]
    if len(pts) < 2:
        continue
    arr = np.asarray(pts)
    (axis_segments if e.dxf.layer in AXIS_LAYERS else segments).append(arr)


def rasterize(seg_list):
    g = np.zeros((rows, cols), dtype=np.uint8)
    for pts in seg_list:
        for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
            n = max(int(np.hypot(x2 - x1, y2 - y1) / (CELL / 2)), 1)
            t = np.linspace(0.0, 1.0, n + 1)
            cs = np.clip(((x1 + (x2 - x1) * t - X0) / CELL).astype(int), 0, cols - 1)
            rs = np.clip(((y1 + (y2 - y1) * t - Y0) / CELL).astype(int), 0, rows - 1)
            g[rs, cs] = 1
    return g


cols = int((X1 - X0) / CELL) + 1
rows = int((Y1 - Y0) / CELL) + 1
grid = rasterize(segments)
print(f"[1] 래스터화: {rows} x {cols}, {time.time() - t0:.1f}초 (y38 벽 유지)")

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
# 2) 기둥 라인 검출(축선 래스터) + 기둥 좌표 확정 (v5.6과 동일)
# ------------------------------------------------------------
occ = grid == 1
grid_axis = rasterize(axis_segments)
hist = (grid_axis == 1)[int(30 * M):int(85 * M), :].sum(axis=0)
peaks = []
for c in range(2, cols - 2):
    if hist[c] > 25 and hist[c] == hist[max(0, c - 15):c + 15].max():
        if not peaks or c - peaks[-1] > 30:
            peaks.append(c)
raw_x = [p / M for p in peaks]
base = raw_x[0]
col_x = [x for x in raw_x if min(abs(x - base - 6 * k) for k in range(20)) < 0.5]

FOOT = 1.6
columns = []
for x in col_x + [col_x[-1] + 6]:
    for y in COL_ROWS_Y:
        r, c = rect_slice(x - FOOT / 2, y - FOOT / 2, FOOT, FOOT)
        if occ[r, c].sum() > 25:
            columns.append((round(x, 1), round(y, 1)))
columns = np.array(sorted(set(map(tuple, columns))))
print(f"[2] 기둥 라인 {len(col_x)}개 / 기둥 {len(columns)}개 확정")

# ------------------------------------------------------------
# 3) 렉 배치 (뱅크 48~68)
# ------------------------------------------------------------
clear_cells = int(CLEAR * M)
rack_units = []
for x in col_x:
    if x < STAGE_L or x > STAGE_R:
        continue
    for y0, y1 in RACK_BANKS:
        r, c = rect_slice(x - RACK_W / 2, y0, RACK_W, y1 - y0)
        blocked = (grid[r, c] == 1).sum(axis=1) > THICK_THRESH
        b2 = blocked.copy()
        for i in np.where(blocked)[0]:
            b2[max(0, i - clear_cells):i + clear_cells + 1] = True
        free = ~b2
        i = 0
        L = len(free)
        while i < L:
            if free[i]:
                j = i
                while j < L and free[j]:
                    j += 1
                seg_len = (j - i) * CELL / 1000
                n_units = int(seg_len // RACK_UNIT_L)
                if n_units >= 1:
                    used = n_units * RACK_UNIT_L
                    ys = y0 + i * CELL / 1000 + (seg_len - used) / 2
                    rr, cc = rect_slice(x - RACK_W / 2, ys, RACK_W, used)
                    sub = grid[rr, cc]
                    sub[sub == 0] = 2
                    rack_units.append((round(x, 2), round(ys, 2), n_units))
                i = j
            else:
                i += 1
total_units = sum(n for _, _, n in rack_units)
print(f"[3] 렉 세그먼트 {len(rack_units)}개, 유닛 {total_units} (통로 {6 - RACK_W:.1f}m) — v5.6과 동일해야 함(무손실)")

# ------------------------------------------------------------
# 4) 문 / 컨베이어 / 스테이션 ([A-3][A-4] 재배치 반영)
# ------------------------------------------------------------
for cv in CONVEYORS:
    add_rect(grid, *cv, val=5)
for (x, y, w, h) in CONVEYORS_V:
    add_rect(grid, x, y, w, h, val=5)
for d in DOORS:
    add_rect(grid, *d, val=4)

AISLE_CX = [35.1 + 6 * k for k in range(11)]
# [A-4] 남측 입구버퍼 45.6→46.2 (렉 시작 47.6과의 간격 1.4m 유지)
AISLE_BUFFERS = [(x, 46.2 if k % 2 == 0 else 69.4) for k, x in enumerate(AISLE_CX)]
# [A-3] 벽 북측 면 배치: 작업대 y38.45~, 도킹 y41.4 / 서→동:
#   입고벨트(14.6~30.6) · handoff(31.6) · 인덕션(36,42) · 합류(50,56)
#   · 패킹(64+6k) · VAS(94,99,104) · 연결컨베이어(106.4)
STATIONS = {
    "handoff": [(31.6, 40.8), (31.6, 71.35), (93.6, 43.55), (93.6, 75.55)],
    "charger": [(107.8, 50 + i * 3) for i in range(6)],
    "charge_q": [(105.5, 50 + i * 3) for i in range(6)],
    "aisle_buf": AISLE_BUFFERS,
    "inbound_buf": [(24, 52 + i * 6) for i in range(3)],
    "qc": [(22.2, 41.6), (22.2, 73.6)],
    "induction": [(36, 41.4), (42, 41.4)],
    "consol": [(50, 41.4), (56, 41.4)],
    "packing": [(64 + 6 * k, 41.4) for k in range(5)],
    "vas": [(94, 41.4), (99, 41.4), (104, 41.4)],
    "returns": [(36, 78.9), (42, 78.9)],
}
# 작업대(장애물, 값5) — 인덕션·합류·패킹·VAS는 벽에 붙여 배치
for x, y in STATIONS["qc"]:
    add_rect(grid, x - 1.2, y - 2.3, 2.4, 1.2, 5)
for x, y in STATIONS["induction"] + STATIONS["consol"]:
    add_rect(grid, x - 1.2, 38.45, 2.4, 1.2, 5)
for x, y in STATIONS["packing"]:
    add_rect(grid, x - 1.5, 38.45, 3.0, 1.2, 5)
for x, y in STATIONS["vas"]:
    add_rect(grid, x - 1.6, 38.45, 3.2, 1.3, 5)
for x, y in STATIONS["returns"]:
    add_rect(grid, x - 1.2, y + 1.3, 2.4, 1.2, 5)
# 도킹 포인트(값3)
for pts in STATIONS.values():
    for x, y in pts:
        add_point(grid, x, y, 3)
n_st = sum(len(v) for v in STATIONS.values())
print(f"[4] 스테이션 {n_st}개소 배치 (아웃바운드 라인 벽 북측 이전)")

# ------------------------------------------------------------
# 5) 저장 — 기준본과 분리된 파일명 (_wallA)
# ------------------------------------------------------------
obstacle = binary_dilation(np.isin(grid, (1, 2, 5)), iterations=int(INFLATE * M))
np.save(os.path.join(OUT, "occupancy_grid_wallA.npy"), grid)
np.save(os.path.join(OUT, "obstacle_mask_wallA.npy"), obstacle)
np.save(os.path.join(OUT, "rack_units_wallA.npy"), np.array(rack_units))
np.save(os.path.join(OUT, "columns_wallA.npy"), columns)
json.dump(STATIONS, open(os.path.join(OUT, "stations_wallA.json"), "w"), indent=1)
print(f"[5] 저장 완료 (*_wallA) / 총 {time.time() - t0:.1f}초")

# ------------------------------------------------------------
# 6) 자가 검증: "팽창 obstacle" 기준 스테이션 도달성 — 원시 그리드
#    연결성 검사는 팽창역 안 도킹(handoff W-S·VAS 3, free=False)을
#    전부 통과시켰다(검수 실측). 도킹 셀이 팽창 장애물 밖이고
#    피킹 시드와 한 성분이어야 통과.
# ------------------------------------------------------------
from scipy import ndimage

lab, n_comp = ndimage.label(~obstacle)
main_lab = lab[int(57.3 * M), int(35.1 * M)]      # 통로1 피킹 지점 시드
bad = [f"{k}[{i}]({x},{y})" for k, pts in STATIONS.items()
       for i, (x, y) in enumerate(pts)
       if not main_lab or lab[int(y * M), int(x * M)] != main_lab]
verify_ok = not bad
if verify_ok:
    print(f"[6] 검증 OK: 스테이션 {n_st}개 전부 팽창 기준 도달 (자유성분 {n_comp}개)")
else:
    print(f"[6] ★검증 실패: 팽창 기준 미도달 {len(bad)}개 {bad}")

# ------------------------------------------------------------
# 7) 시각화 — v5.6과 동일 스타일 + 벽 강조
# ------------------------------------------------------------
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.patches import Rectangle
from matplotlib import font_manager

try:
    fp = font_manager.FontProperties(fname=KOREAN_FONT)
except Exception:
    fp = None

assets = total_units * (2 if DOUBLE_ROW else 1)
pallets = int(total_units * RACK_UNIT_L / PALLET_PITCH) * 2 * RACK_LEVELS

ZONES = [
    (14.5, 46, 14, 21, "입고 스테이징\n(벨트변 검수)", "#3498db"),
    (33, 48, 66.5, 20, "보관존 + 분산 피킹", "#e67e22"),
    (33.5, 38.4, 11.5, 3.7, "토트 인덕션", "#16a085"),
    (47.5, 38.4, 11.5, 3.7, "주문 합류", "#2980b9"),
    (61.5, 38.4, 29.5, 3.7, "패킹 라인 (x5)", "#9b59b6"),
    (91.5, 38.4, 14.6, 3.7, "VAS", "#e91e63"),
    (33, 26.3, 60, 11.2, "포장재·소모품·예비 (인력 전용, 벽 남측)", "#95a5a6"),
    (102.2, 36.5, 7.6, 11.5, "박스 출고 도크(문1)", "#c0392b"),
    (102.2, 66.5, 7.6, 11, "파렛트 출고(문2)", "#8e44ad"),
    (103.5, 49, 6.5, 16, "충전존", "#27ae60"),
    (33, 77.8, 14, 5.2, "반품 처리존", "#d35400"),
    (50, 77.8, 42, 5.2, "예비 보관/시즌", "#7f8c8d"),
    (14.5, 77, 15, 12, "사무실/중이층", "#34495e"),
    (14.5, 26.5, 15, 11, "사무실/중이층", "#34495e"),
]
cmap = ListedColormap(["white", "#888888", "#e67e22", "#2e86de", "#27ae60", "#8e44ad"])
fig, ax = plt.subplots(figsize=(12.5, 11.5))
ax.imshow(grid, cmap=cmap, vmin=0, vmax=5, origin="lower",
          extent=[0, (X1 - X0) / 1000, 0, (Y1 - Y0) / 1000])
for x, y, w_, h_, label, color in ZONES:
    ax.add_patch(Rectangle((x, y), w_, h_, facecolor=color, alpha=0.16,
                           edgecolor=color, lw=1.6))
    ax.text(x + w_ / 2, y + h_ / 2, label, ha="center", va="center",
            fontsize=9, color=color, fontproperties=fp, fontweight="bold")
# y38 벽 강조 (A안의 핵심 가정)
ax.plot([14.1, 110.1], [38.15, 38.15], color="#e74c3c", lw=2.2, alpha=0.85)
ax.text(60, 36.6, "실벽 가정 (개구부 없음 — 팀 확인 대상)", ha="center", va="top",
        fontsize=9.5, color="#e74c3c", fontproperties=fp, fontweight="bold")
for k in range(11):                          # 분산 피킹 위치 표시
    ax.text(35.1 + 6 * k, 57.3, "P", ha="center", va="center", fontsize=8,
            color="#c0392b", fontweight="bold",
            bbox=dict(boxstyle="circle,pad=0.15", fc="white", ec="#c0392b", lw=1.1))
ax.set_title(f"3PL Fulfillment Layout v5.6-A (y38 wall, rack no-loss) — "
             f"{assets} rack assets, ~{pallets:,} pallets, "
             f"stations {n_st - len(bad)}/{n_st} {'OK' if verify_ok else 'FAIL'}",
             fontsize=12)
ax.set_xlabel("m")
ax.set_ylabel("m")
plt.tight_layout()
plt.savefig(os.path.join(OUT, "grid_v56a_wall_full.png"), dpi=115)
print("[7] 시각화 저장: grid_v56a_wall_full.png")

import sys
sys.exit(0 if verify_ok else 1)
