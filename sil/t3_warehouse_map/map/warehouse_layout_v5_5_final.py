# ============================================================
# DXF 도면 → Occupancy Grid → 3PL 풀필먼트 배치 (v5.6)
#
# v5.7 → v5.8 (2026-08-28, 설계도 전체 확인 — 철골 포털프레임 구형 창고):
#   - 바닥 파렛트 보관 밴드(셀값 6) 추가 — 구형 창고의 블록 스태킹 캐릭터.
#     포장재존 2·예비존 2·출고 스테이징 블록 1·입고 서편 1 (총 6밴드,
#     전부 스테이션 팽창역과 간섭 없음 — 자가검증 46/46 유지 확인)
#   - 씬은 밴드 rect에서 래핑 파렛트 더미를 자동 생성, 랙은 박스로 채움
#
# v5.6 → v5.7 (2026-08-28, 시뮬 파트 결정 — "벽 없음" 시나리오 확정):
#   - 기둥 최소화: 랙에 흡수되는 열(y=57.3, 랙 x범위)만 유지하고 나머지
#     기둥 심볼(자유 바닥·외벽 파일라스터) 제거 — 물류창고 룩
#   - y≈76.5 장선(예비존 경계)도 상부 구조선으로 제거 → 예비 보관존 개방
#   - 사무실 2개소(남서·북서): DXF 잔재(내부 벽·기둥 파편) 클리어 후
#     재구축 — 외곽벽 0.2m+출입구 1.4m, 회의실 1실. 인력 전용(스테이션 없음)
#
# v5.5 → v5.6 검수 반영:
#   - 축선 레이어(DWGshare.com_4) 장애물 제외 — 6m 격자 축선이 팽창 후
#     1.6m 벽이 되어 통로를 고립시키던 문제 (기둥 검출에는 계속 사용)
#   - y≈38.1 전폭 단선(레이어 _3, 상부 구조선) 장애물 제외 — 남측 작업
#     라인(인덕션·합류·패킹·VAS)과 메인 홀 사이 통행로 확보
#   - induction/consol/packing 도킹 y 33.2→33.8, 작업대 오프셋 1.0→1.3
#     (기둥 행 y=31.8 팽창역 0.8m 간섭 회피)
#   - handoff W-S (31.6, 39.35)→(31.6, 40.8) (기둥 (32.1, 38.0) 팽창역 회피)
#   - 자가검증 추가: 자유공간 연결성 + 스테이션 도달성 46/46 확인
#
# 파이프라인:
#   1) DXF 래스터화 (주석류·축선 제외, 크롭)
#   2) 기둥 라인(x, 6m 모듈) 검출 + 기둥 105개 좌표 확정 (라인 x 행 교차 검증)
#   3) 렉 배치: 세로형, 4.0x1.2m 유닛 스냅, 더블로우, 기둥 기초 회피
#   4) 문/컨베이어(입고2·출고2 + 패킹→출고 L자 연결)
#   5) 스테이션: 인계4, 충전6+대기6, 렉통로 입구버퍼11(일방통행 교대),
#      입고버퍼3, 벨트변 검수2, 토트 인덕션2, 주문 합류2, 패킹5, VAS3, 반품2
#   6) 저장: occupancy_grid / obstacle_mask / rack_units / columns / map.pgm+yaml
#   7) 시각화: 구역 오버레이(한글 라벨) 포함
#
# 셀 값: 0=빈공간, 1=구조체(기둥 포함), 2=렉, 3=스테이션, 4=문, 5=컨베이어·작업대,
#        6=바닥 파렛트 보관 (기둥 좌표는 columns.npy로 별도 제공)
# 좌표계: 그리드[m]. 건물 좌하단 = (14.1, 25.5), 건물 96 x 64 m
# 운영 모델: P2G 하이브리드 (피커가 렉 통로에서 피킹, AMR은 운반 전용)
# ============================================================
import time

import ezdxf
from ezdxf import path
import numpy as np
from scipy.ndimage import binary_dilation
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.patches import Patch, Rectangle
from matplotlib import font_manager

# ------------------------------------------------------------
# 설정
# ------------------------------------------------------------
DXF_FILE = "Factory_Building_Design.dxf"
CELL = 100                                   # mm/cell (0.1 m)
X0, X1, Y0, Y1 = 660_000, 775_000, 782_000, 888_000   # 크롭 (mm)
SKIP_TYPES = {"TEXT", "MTEXT", "DIMENSION", "LEADER", "HATCH", "INSERT"}
SKIP_LAYERS = {"DWGshare.com_4"}             # 6m 격자 축선(장축선·버블) — 실물 아님
OVERHEAD_YS = (820_100, 858_470)             # y≈38.1(전폭 이중선)·y≈76.5(예비존 경계) 월드 y (mm)
OVERHEAD_TOL = 400


def is_overhead_line(e):
    """기둥 축선 행(y=38.0)을 따라 전폭(97m)을 관통하는 단선 판정.
    벽 두께 표현·개구부가 전혀 없고, 남측 스트립에 별도 출입구가 없으며(문 4개
    전부 북측), 설계상 컨베이어가 이 선을 가로지름 → 바닥 장애물이 아닌
    상부 구조선(처마/지붕선)으로 판단. 장애물에서 제외하고 기둥 검출에만 사용."""
    if e.dxf.layer != "DWGshare.com_3":
        return False
    t = e.dxftype()
    try:
        if t == "LINE":
            xs = (e.dxf.start.x, e.dxf.end.x)
            ys = (e.dxf.start.y, e.dxf.end.y)
        elif t == "LWPOLYLINE":
            pts = list(e.get_points())
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
        else:
            return False
    except Exception:
        return False
    return (max(ys) - min(ys) < 300 and max(xs) - min(xs) > 3_000
            and any(abs(min(ys) - oy) < OVERHEAD_TOL for oy in OVERHEAD_YS))

# 렉 (Isaac Sim 에셋 실측값으로 교체 가능 — 유닛 정수배 스냅)
RACK_UNIT_L, RACK_UNIT_D = 4.0, 1.2
DOUBLE_ROW = True
RACK_W = RACK_UNIT_D * 2 if DOUBLE_ROW else RACK_UNIT_D
MIN_SEG = RACK_UNIT_L
CLEAR, THICK_THRESH = 0.5, 5
RACK_BANKS = [(47, 68)]
STAGE_L, STAGE_R = 30, 102

# AMR (사용자 스펙 1,440 x 641 x 220 mm)
AMR_L, AMR_W = 1.44, 0.641
AMR_R = (AMR_L**2 + AMR_W**2) ** 0.5 / 2     # 회전 외접반경 0.79 m
INFLATE = 0.8

# 문 (입면도 판독: 6.4 x 4.5 m, 양 끝벽 2개소씩)
#   좌측(1축) = 입고, 우측(17축) = 출고 / 문1(남)=컨베이어 전용, 문2(북)=차량
DOORS = [(xw, yd, 1.4, 6.4) for xw in (13.2, 109.6) for yd in (38.3, 70.3)]

# 컨베이어: 입고 2 + 출고 2 + 패킹→출고 L자 연결
CONV_W = 0.9
CONVEYORS = [
    (14.6, 38.9, 16.0, CONV_W),              # 입고 1 (문 하단 가장자리)
    (14.6, 70.9, 16.0, CONV_W),              # 입고 2
    (94.6, 44.0 - CONV_W, 15.0, CONV_W),     # 출고 1 (문1로)
    (94.6, 76.0 - CONV_W, 15.0, CONV_W),     # 출고 2 (파렛트존 보조)
    (101.0, 34.4, 7.4, CONV_W),              # 패킹→출고 연결 (동진)
]
CONVEYORS_V = [(107.5, 35.3, 0.9, 7.8)]      # 패킹→출고 연결 (북상, 세로)

# 기둥 행 (y) — 도면 판독 7행
COL_ROWS_Y = [25.5, 31.8, 38.0, 57.3, 76.5, 83.0, 89.5]

PALLET_PITCH, RACK_LEVELS = 1.15, 4
KOREAN_FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"

# ------------------------------------------------------------
# 1) DXF → 그리드
# ------------------------------------------------------------
t0 = time.time()
doc = ezdxf.readfile(DXF_FILE)
msp = doc.modelspace()
segments = []
for e in msp:
    if e.dxftype() in SKIP_TYPES:
        continue
    try:
        pts = [(v.x, v.y) for v in path.make_path(e).flattening(distance=CELL / 2)]
    except Exception:
        continue
    pts = [(x, y) for x, y in pts if X0 <= x <= X1 and Y0 <= y <= Y1]
    if len(pts) >= 2:
        segments.append((np.asarray(pts),
                         e.dxf.layer in SKIP_LAYERS or is_overhead_line(e)))

cols = int((X1 - X0) / CELL) + 1
rows = int((Y1 - Y0) / CELL) + 1
grid = np.zeros((rows, cols), dtype=np.uint8)
axis_grid = np.zeros_like(grid)              # 축선 전용 — 장애물 제외, 기둥 검출 보조
for pts, is_axis in segments:
    target = axis_grid if is_axis else grid
    for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
        n = max(int(np.hypot(x2 - x1, y2 - y1) / (CELL / 2)), 1)
        t = np.linspace(0.0, 1.0, n + 1)
        cs = np.clip(((x1 + (x2 - x1) * t - X0) / CELL).astype(int), 0, cols - 1)
        rs = np.clip(((y1 + (y2 - y1) * t - Y0) / CELL).astype(int), 0, rows - 1)
        target[rs, cs] = 1
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
# 2) 기둥 라인(x) 검출 + 기둥 좌표 확정
# ------------------------------------------------------------
occ = (grid == 1) | (axis_grid == 1)         # 검출은 축선 포함(v5.5과 동일 조건)
hist = occ[int(30 * M):int(85 * M), :].sum(axis=0)
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
for x in col_x + [col_x[-1] + 6]:            # 마지막 축 포함
    for y in COL_ROWS_Y:
        r, c = rect_slice(x - FOOT / 2, y - FOOT / 2, FOOT, FOOT)
        if occ[r, c].sum() > 25:
            columns.append((round(x, 1), round(y, 1)))
columns = np.array(sorted(set(map(tuple, columns))))
print(f"[2] 기둥 라인 {len(col_x)}개 / 기둥 {len(columns)}개 확정")

# ------------------------------------------------------------
# 2.5) 기둥 최소화 (v5.7) — 랙에 흡수되는 열(y=57.3, 랙 x범위)만 유지.
#      나머지 기둥 심볼은 그리드에서 제거하되, 외벽 파일라스터는 외벽
#      라인 밴드만 보존하고 돌출부를 깎는다 (문 개구부 보존을 위해
#      외벽을 다시 그리지 않고 밴드 마스크로 지운다).
# ------------------------------------------------------------
wall_band = np.zeros_like(grid, dtype=bool)
wall_band[int(25.3 * M):int(25.8 * M), :] = True          # 남측 외벽 25.4/25.6
wall_band[int(89.1 * M):int(89.7 * M), :] = True          # 북측 외벽
wall_band[:, int(14.0 * M):int(14.6 * M)] = True          # 서측 외벽 14.2/14.4
wall_band[:, int(109.8 * M):int(110.4 * M)] = True        # 동측 외벽

keep_cols, drop_cols = [], []
for x, y in columns:
    if abs(y - 57.3) < 1.0 and STAGE_L <= x <= STAGE_R:
        keep_cols.append((x, y))
    else:
        drop_cols.append((x, y))
for x, y in drop_cols:
    # 심볼이 십자형(팔 y±2m대)이라 4.4m 박스로 지운다 (2.2로는 팔 조각 22개 잔존 실측)
    r, c = rect_slice(x - 2.2, y - 2.2, 4.4, 4.4)
    sub = grid[r, c]
    sub[(sub == 1) & ~wall_band[r, c]] = 0
# 외벽 인접 파일라스터 스윕 — 미검출 심볼·큰 심볼 잔재까지 제거 (벽 라인 밴드는 보존)
# ※ ~wall_band 가드 필수: 밴드 전폭을 지우면 직교 외벽이 코너에서 1.6m씩
#   뚫림 (v5.8까지 코너 8곳 개구 실측 — 씬 외벽 구멍의 원인)
for r0, r1 in ((23.6, 25.3), (25.8, 27.4), (87.5, 89.1), (89.7, 91.3)):
    rs = slice(int(r0 * M), int(r1 * M))
    band = grid[rs, :]
    band[(band == 1) & ~wall_band[rs, :]] = 0
for c0, c1 in ((12.4, 14.0), (14.6, 16.2), (108.2, 109.8), (110.4, 112.0)):
    cs = slice(int(c0 * M), int(c1 * M))
    band = grid[:, cs]
    band[(band == 1) & ~wall_band[:, cs]] = 0
columns = np.array(keep_cols)
print(f"[2.5] 기둥 최소화: {len(drop_cols)}개 제거 → 유지 {len(columns)}개 (랙 흡수 열)")

# ------------------------------------------------------------
# 2.6) 외벽 봉합 (v5.9) — DXF 도법상 파일라스터 심볼이 벽 라인을 대체한 자리
#      (남벽 프레임 축 0.3m 단절 6곳 실측)를 메운다. 단면에 벽(1)도 문(4)도
#      전혀 없는 스캔라인만 채움 — 문 개구부는 그대로 보존.
# ------------------------------------------------------------


def seal_wall(axis, lo, hi, s0, s1):
    """axis 'x'=동서벽(x 밴드 고정, y 스캔) / 'y'=남북벽(y 밴드 고정, x 스캔)."""
    n = 0
    for k in range(int(s0 * M), int(s1 * M)):
        if axis == "x":
            cross = grid[k, int(lo * M):int(hi * M)]
        else:
            cross = grid[int(lo * M):int(hi * M), k]
        if not np.any((cross == 1) | (cross == 4)):
            cross[1:-1] = 1                    # 밴드 안쪽 0.4m만 채움 (라인 두께 유지)
            n += 1
    return n


sealed = (seal_wall("x", 14.0, 14.6, 25.3, 89.7) + seal_wall("x", 109.8, 110.4, 25.3, 89.7)
          + seal_wall("y", 25.3, 25.8, 14.0, 110.4) + seal_wall("y", 89.1, 89.7, 14.0, 110.4))
print(f"[2.6] 외벽 봉합: 빈 스캔라인 {sealed}줄 채움 (문 개구부 보존)")

# ------------------------------------------------------------
# 2.7) 사무실 재구축 (v5.7) — DXF 잔재 클리어 후 실제 사무실 배치.
#      외곽벽 0.2m(출입구 1.4m x 2면) + 회의실 1실. 서·남(북)면은 건물
#      외벽이 그대로 벽 역할. 인력 전용 — AMR 스테이션 없음.
#      가구는 씬 빌더가 시각 전용으로 배치(그리드·플래너 무관).
# ------------------------------------------------------------
OFFICES = [(14.6, 25.7, 29.5, 37.5), (14.6, 77.0, 29.5, 89.1)]   # (x0,y0,x1,y1)


def hwall(x0, x1, y):
    r, c = rect_slice(x0, y, x1 - x0, 0.2)
    grid[r, c] = 1


def vwall(x, y0, y1):
    r, c = rect_slice(x, y0, 0.2, y1 - y0)
    grid[r, c] = 1


for x0, y0, x1, y1 in OFFICES:
    r, c = rect_slice(x0, y0, x1 - x0, y1 - y0)
    sub = grid[r, c]
    sub[(sub == 1) & ~wall_band[r, c]] = 0
# 남서 사무실 (25.7~37.5): 북벽 y37.3(문 x25.9~27.3) + 동벽 x29.3(문 y31.2~32.6)
hwall(14.6, 25.9, 37.3)
hwall(27.3, 29.5, 37.3)
vwall(29.3, 25.7, 31.2)
vwall(29.3, 32.6, 37.5)
vwall(20.7, 25.7, 31.3)                       # 회의실 (남서 코너, 문 x17.2~18.4)
hwall(14.6, 17.2, 31.1)
hwall(18.4, 20.9, 31.1)
# 북서 사무실 (77.0~89.1): 남벽 y77.0(문 x25.9~27.3) + 동벽 x29.3(문 y82.4~83.8)
hwall(14.6, 25.9, 77.0)
hwall(27.3, 29.5, 77.0)
vwall(29.3, 77.0, 82.4)
vwall(29.3, 83.8, 89.1)
vwall(20.7, 84.6, 89.1)                       # 회의실 (북서 코너, 문 x17.2~18.4)
hwall(14.6, 17.2, 84.6)
hwall(18.4, 20.9, 84.6)
print(f"[2.7] 사무실 재구축 2개소 (외곽벽+출입구, 회의실 1실씩)")

# ------------------------------------------------------------
# 3) 렉 배치 (세로, 유닛 스냅, 기둥 기초 회피)
# ------------------------------------------------------------
clear_cells = int(CLEAR * M)
rack_units = []                               # (x중심, y시작, 유닛수)
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
assets = total_units * (2 if DOUBLE_ROW else 1)
pallets = int(total_units * RACK_UNIT_L / PALLET_PITCH) * 2 * RACK_LEVELS
print(f"[3] 렉 세그먼트 {len(rack_units)}개, 유닛 {total_units} → 에셋 {assets}개, "
      f"파렛트 약 {pallets:,}개 (통로 {6 - RACK_W:.1f}m)")

# ------------------------------------------------------------
# 4) 문 / 컨베이어
# ------------------------------------------------------------
for cv in CONVEYORS:
    add_rect(grid, *cv, val=5)
for (x, y, w, h) in CONVEYORS_V:
    add_rect(grid, x, y, w, h, val=5)
for d in DOORS:
    add_rect(grid, *d, val=4)

# ------------------------------------------------------------
# 5) 스테이션 (총 46개소)
# ------------------------------------------------------------
AISLE_CX = [35.1 + 6 * k for k in range(11)]              # 렉 통로 중심 x
AISLE_BUFFERS = [(x, 45.6 if k % 2 == 0 else 69.4)        # 일방통행: 짝수=북행
                 for k, x in enumerate(AISLE_CX)]

STATIONS = {
    "handoff":  [(31.6, 40.8), (31.6, 71.35), (93.6, 43.55), (93.6, 75.55)],
    "charger":  [(107.8, 50 + i * 3) for i in range(6)],
    "charge_q": [(105.5, 50 + i * 3) for i in range(6)],
    "aisle_buf": AISLE_BUFFERS,
    "inbound_buf": [(24, 52 + i * 6) for i in range(3)],
    "qc":       [(22.2, 41.6), (22.2, 73.6)],             # 벨트변 검수
    "induction": [(38, 33.8), (44, 33.8)],                # 토트 인덕션
    "consol":   [(62, 33.8), (68, 33.8)],                 # 주문 합류
    "packing":  [(77 + 5.2 * k, 33.8) for k in range(5)],
    "vas":      [(89 + 5.5 * k, 30.2) for k in range(3)],
    "returns":  [(36, 78.9), (42, 78.9)],
}
# 작업대(장애물, 값5)
for x, y in STATIONS["qc"]:
    add_rect(grid, x - 1.2, y - 2.3, 2.4, 1.2, 5)
for x, y in STATIONS["induction"] + STATIONS["consol"]:
    add_rect(grid, x - 1.2, y + 1.3, 2.4, 1.2, 5)
for x, y in STATIONS["packing"]:
    add_rect(grid, x - 1.5, y + 1.3, 3.0, 1.2, 5)
for x, y in STATIONS["vas"]:
    add_rect(grid, x - 1.6, y - 2.6, 3.2, 1.3, 5)
for x, y in STATIONS["returns"]:
    add_rect(grid, x - 1.2, y + 1.3, 2.4, 1.2, 5)
# 도킹 포인트(값3)
for pts in STATIONS.values():
    for x, y in pts:
        add_point(grid, x, y, 3)
n_st = sum(len(v) for v in STATIONS.values())
print(f"[4] 스테이션 {n_st}개소 배치")

# ------------------------------------------------------------
# 5.5) 바닥 파렛트 보관 밴드 (셀값 6, v5.8) — 구형 창고 블록 스태킹.
#      씬 빌더가 이 rect들에서 래핑 파렛트 더미를 자동 배치한다.
# ------------------------------------------------------------
PALLET_BANDS = [
    (33.0, 27.0, 84.0, 29.0), (33.0, 30.4, 84.0, 31.4),   # 포장재·빈파렛트존
    (50.0, 78.4, 92.0, 80.4), (50.0, 81.4, 92.0, 82.4),   # 예비 보관/시즌존
    (103.0, 67.0, 108.6, 74.0),                            # 파렛트 출고 스테이징 블록
    (15.6, 47.0, 17.6, 66.0),                              # 입고 서편 벽면
]
for x0, y0, x1, y1 in PALLET_BANDS:
    add_rect(grid, x0, y0, x1 - x0, y1 - y0, 6)
print(f"[4.5] 바닥 파렛트 밴드 {len(PALLET_BANDS)}개 (셀값 6)")

# ------------------------------------------------------------
# 6) 저장 (그리드 / 마스크 / 좌표 / ROS 맵)
# ------------------------------------------------------------
obstacle = binary_dilation(np.isin(grid, (1, 2, 5, 6)), iterations=int(INFLATE * M))
np.save("occupancy_grid.npy", grid)
np.save("obstacle_mask.npy", obstacle)
np.save("rack_units.npy", np.array(rack_units))
np.save("columns.npy", columns)
import json
json.dump({k: v for k, v in STATIONS.items()}, open("stations.json", "w"), indent=1)

# ROS map (건물 범위 크롭, 원점 = 건물 좌하단)
bx0, bx1, by0, by1 = 14.1, 110.1, 25.5, 89.5
sub = grid[int(by0 * M):int(by1 * M), int(bx0 * M):int(bx1 * M)]
img = np.where(np.isin(sub, (1, 2, 5, 6)), 0, 254).astype(np.uint8)[::-1, :]
h, w = img.shape
with open("map.pgm", "wb") as f:
    f.write(f"P5\n{w} {h}\n255\n".encode())
    f.write(img.tobytes())
open("map.yaml", "w").write(
    "image: map.pgm\nmode: trinary\nresolution: 0.1\norigin: [0.0, 0.0, 0.0]\n"
    "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
print(f"[5] 저장 완료 (ROS map {w}x{h}px 포함)")

# ------------------------------------------------------------
# 6.5) 자가검증 — 자유공간 연결성 + 스테이션 도달성
#      (도킹 포인트가 팽창 장애물 밖이고, 피킹 시드와 한 성분이어야 통과)
# ------------------------------------------------------------
from scipy.ndimage import label as cc_label

labels, n_comp = cc_label(~obstacle)
main_label = labels[int(57.3 * M), int(35.1 * M)]     # 통로1 피킹 지점 시드
station_ok, station_bad = 0, []
for name, pts in STATIONS.items():
    for x, y in pts:
        if main_label and labels[int(y * M), int(x * M)] == main_label:
            station_ok += 1
        else:
            station_bad.append((name, x, y))
verify_ok = station_ok == n_st
print(f"[6] 자가검증: 도달 {station_ok}/{n_st}, 자유성분 {n_comp}개"
      + (" — OK" if verify_ok else f" — FAIL {station_bad[:6]}"))

# ------------------------------------------------------------
# 7) 시각화 — 셀 레이어 + 구역 오버레이
# ------------------------------------------------------------
try:
    fp = font_manager.FontProperties(fname=KOREAN_FONT)
except Exception:
    fp = None

ZONES = [
    (14.5, 46, 14, 21, "입고 스테이징\n(벨트변 검수)", "#3498db"),
    (33, 47, 66.5, 21, "보관존 + 분산 피킹", "#e67e22"),
    (34, 32.6, 13, 4.6, "토트 인덕션", "#16a085"),
    (57, 32.6, 14, 4.6, "주문 합류", "#2980b9"),
    (73.5, 32.6, 31, 4.6, "패킹 라인 (x5)", "#9b59b6"),
    (33, 26.3, 51, 5.5, "포장재·빈파렛트", "#95a5a6"),
    (85.5, 26.3, 19, 5.7, "VAS 작업장", "#e91e63"),
    (102.2, 36.5, 7.6, 11.5, "박스 출고 도크(문1)", "#c0392b"),
    (102.2, 66.5, 7.6, 11, "파렛트 출고(문2)", "#8e44ad"),
    (103.5, 49, 6.5, 16, "충전존", "#27ae60"),
    (33, 77.8, 14, 5.2, "반품 처리존", "#d35400"),
    (50, 77.8, 42, 5.2, "예비 보관/시즌", "#7f8c8d"),
    (14.6, 77, 14.9, 12.1, "사무실 (재구축)", "#34495e"),
    (14.6, 25.7, 14.9, 11.8, "사무실 (재구축)", "#34495e"),
]
cmap = ListedColormap(["white", "#888888", "#e67e22", "#2e86de", "#27ae60", "#8e44ad", "#a97142"])
fig, ax = plt.subplots(figsize=(12.5, 11.5))
ax.imshow(grid, cmap=cmap, vmin=0, vmax=6, origin="lower",
          extent=[0, (X1 - X0) / 1000, 0, (Y1 - Y0) / 1000])
for x, y, w_, h_, label, color in ZONES:
    ax.add_patch(Rectangle((x, y), w_, h_, facecolor=color, alpha=0.16,
                           edgecolor=color, lw=1.6))
    ax.text(x + w_ / 2, y + h_ / 2, label, ha="center", va="center",
            fontsize=9, color=color, fontproperties=fp, fontweight="bold")
for k in range(11):                          # 분산 피킹 위치 표시
    ax.text(35.1 + 6 * k, 57.3, "P", ha="center", va="center", fontsize=8,
            color="#c0392b", fontweight="bold",
            bbox=dict(boxstyle="circle,pad=0.15", fc="white", ec="#c0392b", lw=1.1))
ax.set_title(f"3PL Fulfillment Layout v5.8 — {assets} rack assets, "
             f"~{pallets:,} pallets, stations {station_ok}/{n_st} "
             f"{'OK' if verify_ok else 'FAIL'}, columns {len(columns)}", fontsize=13)
ax.set_xlabel("m")
ax.set_ylabel("m")
plt.tight_layout()
plt.savefig("grid_full.png", dpi=115)                      # 버전 무관 고정 파일명
plt.show()

import sys
sys.exit(0 if verify_ok else 1)
