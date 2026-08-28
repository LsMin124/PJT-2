"""v5.6 맵 → isaac_replay.py 재생 데이터 생성 (팀원 익스포트 포맷 호환).

팀원의 sim_v1 익스포트(isaac_export_wallA)가 우리 쪽에 없으므로, 같은 포맷의
데이터를 우리 맵(occupancy_grid + stations.json)에서 직접 만든다:
  - trajectories.json: 스테이션 간 미션 6건 — 팽창 장애물(obstacle_mask) 기준
    A*(8방, 코너 컷 금지) + 가시선 단순화, 등속 1.5 m/s, 경유지 6s 정차,
    출발 시각 스태거. 자가검증: 전 세그먼트 0.05m 샘플링 → 침범 0 필수(exit 1).
  - scene.json: 팀원 스크립트의 박스 씬 모드용(벽/랙/작업대 rect) — 우리는
    --stage 모드로 실제 USD 안에서 재생하므로 참고용이지만, 팀원이 자기
    Windows에서 우리 맵을 재생해볼 수 있게 포맷을 채워 둔다.

실행: miniforge python (Isaac 불필요)
  python export_replay.py   → ../../isaac_export_t3/{scene,trajectories}.json
"""
import heapq
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
MAP_DIR = os.path.join(HERE, "..", "t3_warehouse_map", "map")
OUT_DIR = os.path.join(HERE, "..", "..", "isaac_export_t3")
os.makedirs(OUT_DIR, exist_ok=True)

CELL = 0.1
SPEED = 1.5                    # m/s — iw.hub 공칭 주행속도
DWELL = 6.0                    # 경유 스테이션 정차 (s)
STAGGER = 5.0                  # 로봇별 출발 간격 (s)
ROBOT_DIM = [1.44, 0.641, 0.35]  # iw.hub 실측 (isaac_replay 몸체 스케일)

grid = np.load(os.path.join(MAP_DIR, "occupancy_grid.npy"))
obst = np.load(os.path.join(MAP_DIR, "obstacle_mask.npy")).astype(bool)
stations = json.load(open(os.path.join(MAP_DIR, "stations.json"), encoding="utf-8"))
free = ~obst
ROWS, COLS = grid.shape


def cell(p):
    return int(p[1] / CELL), int(p[0] / CELL)              # (row, col)


def astar(a, b):
    """팽창 자유 그리드 8방 A* — 대각은 양측 직교 셀도 자유일 때만(코너 컷 금지)."""
    (r0, c0), (r1, c1) = cell(a), cell(b)
    if not (free[r0, c0] and free[r1, c1]):
        return None
    D = [(-1, 0, 1), (1, 0, 1), (0, -1, 1), (0, 1, 1),
         (-1, -1, 2**0.5), (-1, 1, 2**0.5), (1, -1, 2**0.5), (1, 1, 2**0.5)]
    g = {(r0, c0): 0.0}
    came = {}
    pq = [(0.0, (r0, c0))]
    while pq:
        _, cur = heapq.heappop(pq)
        if cur == (r1, c1):
            path = [cur]
            while cur in came:
                cur = came[cur]
                path.append(cur)
            return [((c + 0.5) * CELL, (r + 0.5) * CELL) for r, c in reversed(path)]
        r, c = cur
        for dr, dc, w in D:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < ROWS and 0 <= nc < COLS and free[nr, nc]):
                continue
            if dr and dc and not (free[r + dr, c] and free[r, c + dc]):
                continue
            ng = g[cur] + w
            if ng < g.get((nr, nc), 1e18):
                g[(nr, nc)] = ng
                came[(nr, nc)] = cur
                h = math.hypot(nr - r1, nc - c1)
                heapq.heappush(pq, (ng + h, (nr, nc)))
    return None


def seg_clear(p, q):
    """세그먼트를 0.05m 간격 샘플링해 팽창 장애물 침범 검사."""
    d = math.hypot(q[0] - p[0], q[1] - p[1])
    for k in range(int(d / 0.05) + 2):
        a = min(k * 0.05 / d, 1.0) if d else 0.0
        x, y = p[0] + (q[0] - p[0]) * a, p[1] + (q[1] - p[1]) * a
        if obst[int(y / CELL), int(x / CELL)]:
            return False
    return True


def simplify(path):
    """가시선 단순화 — 다음으로 보이는 가장 먼 점만 남긴다 (계단 제거)."""
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not seg_clear(path[i], path[j]):
            j -= 1
        out.append(path[j])
        i = j
    return out


# ── 미션 6건: (스테이션 종류, 인덱스) 경유 열 — 입고→피킹→합류→패킹 서사 절단면 ──
S = stations
MISSIONS = [
    [("handoff", 0), ("aisle_buf", 4), ("handoff", 2)],           # 서 핸드오프 → 통로5 → 동 핸드오프
    [("inbound_buf", 1), ("aisle_buf", 1), ("handoff", 1)],       # 입고 버퍼 → 북측 통로 → 핸드오프
    [("induction", 0), ("consol", 1), ("packing", 2)],            # 인덕션 → 합류 → 패킹 (남측 작업 라인)
    [("charger", 0), ("aisle_buf", 10), ("vas", 1)],              # 충전 → 통로 → VAS
    [("returns", 1), ("aisle_buf", 3), ("inbound_buf", 2)],       # 반품 → 통로 → 입고 버퍼
    [("handoff", 3), ("aisle_buf", 7), ("aisle_buf", 5), ("charger", 5)],  # 순회 후 충전 복귀
]

robots = {}
total_len = 0.0
for ri, legs in enumerate(MISSIONS):
    pts = [tuple(S[k][i]) for k, i in legs]
    t = ri * STAGGER
    wp = [[0.0, pts[0][0], pts[0][1]], [round(t, 2), pts[0][0], pts[0][1]]]
    for a, b in zip(pts, pts[1:]):
        path = astar(a, b)
        if path is None:
            print(f"[export] 경로 실패: robot{ri} {a}→{b}")
            sys.exit(1)
        sp = simplify(path)
        for p, q in zip(sp, sp[1:]):
            if not seg_clear(p, q):
                print(f"[export] 침범 세그먼트: robot{ri} {p}→{q}")
                sys.exit(1)
            t += math.hypot(q[0] - p[0], q[1] - p[1]) / SPEED
            total_len += math.hypot(q[0] - p[0], q[1] - p[1])
            wp.append([round(t, 2), round(q[0], 2), round(q[1], 2)])
        t += DWELL                                          # 경유 정차 (도착 대기 포함)
        wp.append([round(t, 2), round(b[0], 2), round(b[1], 2)])
    robots[str(ri)] = wp


# ── scene.json (팀원 박스 씬 모드용 — 값별 그리디 rect) ──
def greedy_rects(mask):
    rects = []
    open_runs = {}
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


def rects_m(val):
    return [[round(c0 * CELL, 2), round(r0 * CELL, 2), round(w * CELL, 2), round(h * CELL, 2)]
            for r0, c0, h, w in greedy_rects(grid == val)]


scene = {
    "meta": {"robot_dim_m": ROBOT_DIM, "source": "t3 v5.6 full-grid frame [m]"},
    "obstacles": {"structure": {"height": 9.0, "rects": rects_m(1)},
                  "racks": {"height": 3.0, "rects": rects_m(2)},
                  "conveyor_table": {"height": 0.9, "rects": rects_m(5)},
                  "pallet_zone": {"height": 2.0, "rects": rects_m(6)}},
    "stations": stations,
}
json.dump(scene, open(os.path.join(OUT_DIR, "scene.json"), "w", encoding="utf-8"))
json.dump({"robots": robots}, open(os.path.join(OUT_DIR, "trajectories.json"), "w", encoding="utf-8"))
t_end = max(wp[-1][0] for wp in robots.values())
print(f"[export] 로봇 {len(robots)}대 · 총 주행 {total_len:.0f}m · 시뮬 {t_end:.0f}s · 침범 0 → {OUT_DIR}")
