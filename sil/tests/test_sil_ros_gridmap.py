"""sil_ros.gridmap 단위 테스트 — numpy 만 필요(rclpy 불필요).
  cd sil && python3 -m pytest -q tests/test_sil_ros_gridmap.py

합성 마스크로 A* · seg_clear · simplify · densify 를 검증하고, patrol.py 원본의 플래너 사본과
무작위 마스크에서 결과가 동일한지 대조한다. 실제 맵 파일(*.npy 는 git 미추적)이 있으면
스테이션이 자유 셀인지 · 투어 구간이 계획되는지도 확인한다.
"""
import heapq
import math
import os
import random
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))   # …/sil
from sil_ros.gridmap import (CELL, DEFAULT_MAP_DIR, DENSIFY_STEP_M, ENV_MAP_DIR, OBSTACLE_FILE,  # noqa: E402
                             STATIONS_FILE, GridMap, resolve_map_dir)

REAL_MAP = all(os.path.exists(os.path.join(DEFAULT_MAP_DIR, f)) for f in (OBSTACLE_FILE, STATIONS_FILE))


# ── 원본 patrol.py 플래너 사본 (대조용, 로직 변경 없음) ──
def reference_planner(obst):
    free = ~obst
    ROWS, COLS = obst.shape

    def cell(p):
        return int(p[1] / CELL), int(p[0] / CELL)

    def astar(a, b):
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
                    heapq.heappush(pq, (ng + math.hypot(nr - r1, nc - c1), (nr, nc)))
        return None

    def seg_clear(p, q):
        d = math.hypot(q[0] - p[0], q[1] - p[1])
        for k in range(int(d / 0.05) + 2):
            a = min(k * 0.05 / d, 1.0) if d else 0.0
            if obst[int((p[1] + (q[1] - p[1]) * a) / CELL),
                    int((p[0] + (q[0] - p[0]) * a) / CELL)]:
                return False
        return True

    def simplify(path):
        out = [path[0]]
        i = 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1 and not seg_clear(path[i], path[j]):
                j -= 1
            out.append(path[j])
            i = j
        return out

    return astar, seg_clear, simplify


def center(r, c):
    return (c + 0.5) * CELL, (r + 0.5) * CELL


def open_map(rows=20, cols=20):
    return GridMap(np.zeros((rows, cols), dtype=bool), stations={"a": [[0.55, 0.55], [1.55, 0.55]]})


def wall_map():
    """20×20, 열 10 에 세로 벽(행 0~14), 아래쪽(행 15~19)만 뚫림."""
    m = np.zeros((20, 20), dtype=bool)
    m[0:15, 10] = True
    return GridMap(m)


# ── 셀 변환 · free · station ──
def test_cell_center_roundtrip():
    gm = open_map()
    assert gm.cell((0.55, 0.25)) == (2, 5)
    assert gm.center((2, 5)) == pytest.approx((0.55, 0.25))
    assert gm.cell(gm.center((7, 3))) == (7, 3)


def test_free_respects_bounds_and_obstacles():
    gm = wall_map()
    assert gm.free(center(3, 3))
    assert not gm.free(center(3, 10))          # 벽
    assert gm.free((-0.05, 0.05))              # int() 절삭: -0.5 → 셀 0 (원본과 같은 동작)
    assert not gm.free((-0.15, 0.05))          # 셀 -1 = 맵 밖(음수) — 원본은 뒤에서 감겼다
    assert not gm.free((5.0, 0.05))            # 맵 밖(초과)


def test_station_returns_tuple():
    gm = open_map()
    assert gm.station("a", 1) == (1.55, 0.55)
    assert isinstance(gm.station("a", 0), tuple)


# ── A* ──
def test_astar_open_grid_is_straight_and_ends_on_cell_centers():
    gm = open_map()
    path = gm.astar(center(2, 2), center(2, 12))
    assert len(path) == 11
    assert path[0] == pytest.approx(center(2, 2))
    assert path[-1] == pytest.approx(center(2, 12))
    assert all(gm.free(p) for p in path)


def test_astar_same_cell_returns_single_point():
    gm = open_map()
    assert gm.astar(center(4, 4), (0.47, 0.43)) == [center(4, 4)]


def test_astar_none_when_endpoint_blocked_or_unreachable():
    gm = wall_map()
    assert gm.astar(center(3, 3), center(3, 10)) is None     # 목표가 벽
    sealed = np.zeros((10, 10), dtype=bool)
    sealed[:, 5] = True                                       # 완전 차단
    assert GridMap(sealed).astar(center(2, 2), center(2, 8)) is None


def test_astar_routes_around_wall_through_gap():
    gm = wall_map()
    path = gm.astar(center(2, 3), center(2, 16))
    assert path is not None
    assert all(gm.free(p) for p in path)
    rows_at_wall = [gm.cell(p)[0] for p in path if gm.cell(p)[1] == 10]
    assert rows_at_wall and min(rows_at_wall) >= 15          # 뚫린 구간으로만 통과


def test_astar_forbids_corner_cutting():
    m = np.zeros((3, 3), dtype=bool)
    m[0, 1] = m[1, 0] = True                                  # (0,0) 에서 (1,1) 로 가는 대각의 양옆이 막힘
    m[2, 0] = m[0, 2] = True
    gm = GridMap(m)
    assert gm.astar(center(0, 0), center(1, 1)) is None


# ── seg_clear · simplify · densify ──
def test_seg_clear_detects_wall_crossing():
    gm = wall_map()
    assert gm.seg_clear(center(3, 3), center(3, 8))
    assert not gm.seg_clear(center(3, 3), center(3, 16))
    assert gm.seg_clear(center(17, 3), center(17, 16))
    assert gm.seg_clear(center(3, 3), center(3, 3))          # 길이 0


def test_simplify_collapses_visible_stairs():
    gm = open_map()
    path = gm.astar(center(1, 1), center(8, 15))
    sp = gm.simplify(path)
    assert sp == [path[0], path[-1]]


def test_simplify_keeps_corner_around_wall():
    gm = wall_map()
    sp = gm.simplify(gm.astar(center(2, 3), center(2, 16)))
    assert len(sp) >= 3
    assert all(gm.seg_clear(p, q) for p, q in zip(sp, sp[1:]))


def test_densify_spacing_and_duplicate_endpoint_quirk():
    pts = [(0.0, 0.0), (10.0, 0.0)]
    out = GridMap.densify(pts, step=DENSIFY_STEP_M)
    xs = [p[0] for p in out]
    assert xs == pytest.approx([0.0, 2.5, 5.0, 7.5, 10.0, 10.0])   # 원본대로: 배수 길이면 끝점 중복
    out2 = GridMap.densify([(0.0, 0.0), (0.0, 6.0)])
    assert [p[1] for p in out2] == pytest.approx([0.0, 2.5, 5.0, 6.0])
    gaps = [math.dist(a, b) for a, b in zip(out2, out2[1:])]
    assert max(gaps) <= DENSIFY_STEP_M + 1e-9


# ── 원본 플래너와 대조 ──
def test_planner_matches_patrol_reference_on_random_masks():
    rng = random.Random(2026)
    checked = 0
    for trial in range(12):
        mask = np.array([[rng.random() < 0.25 for _ in range(30)] for _ in range(30)])
        gm = GridMap(mask)
        ref_astar, ref_seg, ref_simplify = reference_planner(mask)
        for _ in range(6):
            a = center(rng.randrange(30), rng.randrange(30))
            b = center(rng.randrange(30), rng.randrange(30))
            ref = ref_astar(a, b)
            got = gm.astar(a, b)
            assert got == ref, (trial, a, b)
            if ref is not None:
                assert gm.simplify(got) == ref_simplify(ref)
                for p, q in zip(ref, ref[1:]):
                    assert gm.seg_clear(p, q) == ref_seg(p, q)
                checked += 1
    assert checked > 10


# ── 로더 ──
def test_resolve_map_dir_precedence(monkeypatch):
    monkeypatch.delenv(ENV_MAP_DIR, raising=False)
    assert resolve_map_dir() == DEFAULT_MAP_DIR
    monkeypatch.setenv(ENV_MAP_DIR, "/env/map")
    assert resolve_map_dir() == "/env/map"
    assert resolve_map_dir("/arg/map") == "/arg/map"


def test_load_from_synthetic_dir(tmp_path):
    np.save(tmp_path / OBSTACLE_FILE, np.zeros((4, 4), dtype=bool))
    (tmp_path / STATIONS_FILE).write_text('{"a": [[0.15, 0.25]]}', encoding="utf-8")
    gm = GridMap.load(str(tmp_path))
    assert gm.rows == gm.cols == 4 and gm.grid is None and gm.map_dir == str(tmp_path)
    assert gm.station("a", 0) == (0.15, 0.25)


def test_grid_shape_mismatch_rejected():
    with pytest.raises(ValueError):
        GridMap(np.zeros((4, 4), dtype=bool), grid=np.zeros((4, 5), dtype=np.uint8))


# ── 실제 맵 ──
@pytest.mark.skipif(not REAL_MAP, reason="실제 맵 파일 없음 (*.npy 는 git 미추적)")
def test_real_map_stations_are_free_cells():
    gm = GridMap.load()
    bad = [f"{typ}_{i} {p}" for typ, pts in gm.stations.items() for i, p in enumerate(pts) if not gm.free(p)]
    assert not bad, bad


@pytest.mark.skipif(not REAL_MAP, reason="실제 맵 파일 없음 (*.npy 는 git 미추적)")
def test_real_map_patrol_tour_legs_plan_and_stay_clear():
    gm = GridMap.load()
    tour = [("handoff", 0), ("induction", 0), ("packing", 2), ("handoff", 2),
            ("charger", 0), ("handoff", 3), ("returns", 0), ("handoff", 1)]
    for (t0, i0), (t1, i1) in zip(tour, tour[1:] + tour[:1]):
        raw = gm.astar(gm.station(t0, i0), gm.station(t1, i1))
        assert raw is not None, f"{t0}_{i0}→{t1}_{i1}"
        sp = gm.simplify(raw)
        assert all(gm.seg_clear(p, q) for p, q in zip(sp, sp[1:]))
        dense = gm.densify(sp)
        assert max(math.dist(p, q) for p, q in zip(dense, dense[1:])) <= DENSIFY_STEP_M + 1e-9
