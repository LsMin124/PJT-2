"""v5.6 맵 → isaac_replay.py 재생 데이터 생성 (팀원 익스포트 포맷 호환).

팀원의 sim_v1 익스포트(isaac_export_wallA)가 우리 쪽에 없으므로, 같은 포맷의
데이터를 우리 맵(occupancy_grid + stations.json)에서 직접 만든다:
  - trajectories.json: 스테이션 간 미션 6건 — 팽창 장애물(obstacle_mask) 기준
    A*(8방, 코너 컷 금지) + 가시선 단순화(sil_ros.gridmap.GridMap — patrol 과 동일 플래너),
    등속 1.5 m/s, 경유지 6s 정차, 출발 시각 스태거. 자가검증: 전 세그먼트 0.05m 샘플링 → 침범 0 필수(exit 1).
  - scene.json: 팀원 스크립트의 박스 씬 모드용(벽/랙/작업대 rect) — 우리는
    --stage 모드로 실제 USD 안에서 재생하므로 참고용이지만, 팀원이 자기
    Windows에서 우리 맵을 재생해볼 수 있게 포맷을 채워 둔다.

실행: miniforge python (Isaac 불필요, numpy 만)
  python export_replay.py   → ../../isaac_export_t3/{scene,trajectories}.json      # sil/t3_warehouse/ 의 호환 셔ム
  python -m sil_ros.tools.export_replay [--map-dir DIR] [--out DIR]                 # sil/ 에서 직접
"""
import argparse
import json
import os
import sys

from sil_ros import SIL_ROOT
from sil_ros.geometry import dist
from sil_ros.gridmap import DEFAULT_MAP_DIR, ENV_MAP_DIR, GridMap

DEFAULT_OUT_DIR = os.path.normpath(os.path.join(SIL_ROOT, "..", "isaac_export_t3"))   # <repo>/isaac_export_t3

SPEED = 1.5                    # m/s — iw.hub 공칭 주행속도
DWELL = 6.0                    # 경유 스테이션 정차 (s)
STAGGER = 5.0                  # 로봇별 출발 간격 (s)
ROBOT_DIM = [1.44, 0.641, 0.35]  # iw.hub 실측 (isaac_replay 몸체 스케일)

# occupancy_grid 값 → 씬 요소 (높이 m)
GRID_STRUCTURE, GRID_RACK, GRID_CONVEYOR, GRID_PALLET = 1, 2, 5, 6
H_STRUCTURE, H_RACK, H_CONVEYOR, H_PALLET = 9.0, 3.0, 0.9, 2.0
SCENE_SOURCE = "t3 v5.6 full-grid frame [m]"

# ── 미션 6건: (스테이션 종류, 인덱스) 경유 열 — 입고→피킹→합류→패킹 서사 절단면 ──
MISSIONS = [
    [("handoff", 0), ("aisle_buf", 4), ("handoff", 2)],           # 서 핸드오프 → 통로5 → 동 핸드오프
    [("inbound_buf", 1), ("aisle_buf", 1), ("handoff", 1)],       # 입고 버퍼 → 북측 통로 → 핸드오프
    [("induction", 0), ("consol", 1), ("packing", 2)],            # 인덕션 → 합류 → 패킹 (남측 작업 라인)
    [("charger", 0), ("aisle_buf", 10), ("vas", 1)],              # 충전 → 통로 → VAS
    [("returns", 1), ("aisle_buf", 3), ("inbound_buf", 2)],       # 반품 → 통로 → 입고 버퍼
    [("handoff", 3), ("aisle_buf", 7), ("aisle_buf", 5), ("charger", 5)],  # 순회 후 충전 복귀
]


class ExportError(RuntimeError):
    """경로 실패 · 침범 세그먼트 — 메시지가 그대로 "[export] …" 로 출력된다."""


def mission_waypoints(gm, ri, legs, t0):
    """미션 1건 → ([[t, x, y], ...], 주행 거리 m). 경로 실패 · 침범 시 ExportError."""
    pts = [gm.station(k, i) for k, i in legs]
    t = t0
    wp = [[0.0, pts[0][0], pts[0][1]], [round(t, 2), pts[0][0], pts[0][1]]]
    length = 0.0
    for a, b in zip(pts, pts[1:]):
        path = gm.astar(a, b)
        if path is None:
            raise ExportError(f"경로 실패: robot{ri} {a}→{b}")
        sp = gm.simplify(path)
        for p, q in zip(sp, sp[1:]):
            if not gm.seg_clear(p, q):
                raise ExportError(f"침범 세그먼트: robot{ri} {p}→{q}")
            t += dist(p, q) / SPEED
            length += dist(p, q)
            wp.append([round(t, 2), round(q[0], 2), round(q[1], 2)])
        t += DWELL                                          # 경유 정차 (도착 대기 포함)
        wp.append([round(t, 2), round(b[0], 2), round(b[1], 2)])
    return wp, length


def build_trajectories(gm):
    """MISSIONS → ({robot_id: waypoints}, 총 주행 거리 m)."""
    robots = {}
    total_len = 0.0
    for ri, legs in enumerate(MISSIONS):
        wp, length = mission_waypoints(gm, ri, legs, ri * STAGGER)
        robots[str(ri)] = wp
        total_len += length
    return robots, total_len


# ── scene.json (팀원 박스 씬 모드용 — 값별 그리디 rect) ──
def _row_runs(row):
    """한 행의 True 연속 구간 [(c0, c1), ...] (c1 배타)."""
    runs = []
    c = 0
    n = len(row)
    while c < n:
        if row[c]:
            c0 = c
            while c < n and row[c]:
                c += 1
            runs.append((c0, c))
        else:
            c += 1
    return runs


def greedy_rects(mask):
    """같은 열 구간이 연속 행에 이어지면 하나의 rect 로 — (r0, c0, 높이, 너비) 셀 단위."""
    rects = []
    open_runs = {}
    for r in range(mask.shape[0]):
        nxt = {}
        for run in _row_runs(mask[r]):
            if run in open_runs and open_runs[run][1] == r - 1:
                open_runs[run][1] = r
                nxt[run] = open_runs[run]
            else:
                nxt[run] = [r, r]
        rects += [(r0, run[0], r1 - r0 + 1, run[1] - run[0])
                  for run, (r0, r1) in open_runs.items() if run not in nxt]
        open_runs = nxt
    rects += [(r0, run[0], r1 - r0 + 1, run[1] - run[0]) for run, (r0, r1) in open_runs.items()]
    return rects


def rects_m(grid, val, cell):
    """grid == val 인 셀들의 그리디 rect → [x, y, w, h] (m)."""
    return [[round(c0 * cell, 2), round(r0 * cell, 2), round(w * cell, 2), round(h * cell, 2)]
            for r0, c0, h, w in greedy_rects(grid == val)]


def build_scene(gm):
    def rm(val):
        return rects_m(gm.grid, val, gm.cell_size)
    return {
        "meta": {"robot_dim_m": ROBOT_DIM, "source": SCENE_SOURCE},
        "obstacles": {"structure": {"height": H_STRUCTURE, "rects": rm(GRID_STRUCTURE)},
                      "racks": {"height": H_RACK, "rects": rm(GRID_RACK)},
                      "conveyor_table": {"height": H_CONVEYOR, "rects": rm(GRID_CONVEYOR)},
                      "pallet_zone": {"height": H_PALLET, "rects": rm(GRID_PALLET)}},
        "stations": gm.stations,
    }


def write_json(path, obj):
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(obj, fh)


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description="v5.6 맵 → isaac_replay 재생 JSON")
    ap.add_argument("--map-dir", default=None,
                    help=f"맵 디렉토리 (기본 {DEFAULT_MAP_DIR}, 환경변수 {ENV_MAP_DIR})")
    ap.add_argument("--out", default=DEFAULT_OUT_DIR, help="출력 디렉토리")
    return ap.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    gm = GridMap.load(args.map_dir)
    if gm.grid is None:
        print(f"[export] occupancy_grid.npy 없음: {gm.map_dir}")
        return 1
    os.makedirs(args.out, exist_ok=True)
    try:
        robots, total_len = build_trajectories(gm)
    except ExportError as e:
        print(f"[export] {e}")
        return 1
    write_json(os.path.join(args.out, "scene.json"), build_scene(gm))
    write_json(os.path.join(args.out, "trajectories.json"), {"robots": robots})
    t_end = max(wp[-1][0] for wp in robots.values())
    print(f"[export] 로봇 {len(robots)}대 · 총 주행 {total_len:.0f}m · 시뮬 {t_end:.0f}s · 침범 0 → {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
