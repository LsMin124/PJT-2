"""팽창 장애물 그리드 + 스테이션 로더와 A* 플래너 — patrol.py 와 export_replay.py 의 "동일 플래너"를 하나로.

두 원본의 astar · seg_clear · simplify 는 임시변수 이름만 다르고 로직(8방 이웃 순서 · 코너 컷 금지 ·
휴리스틱 · 힙 타이브레이크)이 완전히 같아 그대로 옮겼다(tests/test_sil_ros_gridmap.py 가 원본 사본과 대조).

사용:
  gm = GridMap.load()                      # sil/t3_warehouse_map/map (또는 SIL_MAP_DIR / 인자)
  path = gm.astar(gm.station("handoff", 0), gm.station("induction", 0))
  pts = gm.densify(gm.simplify(path))
numpy 만 필요하다(rclpy 불필요).
"""
import heapq
import json
import math
import os

import numpy as np

from sil_ros import SIL_ROOT

CELL = 0.1                      # m — 그리드 셀 크기 (map.yaml resolution)
SEG_SAMPLE_M = 0.05             # m — 세그먼트 침범 검사 샘플 간격
DENSIFY_STEP_M = 2.5            # m — 긴 직선 레그 분할 간격 (크로스트랙 이탈 억제)
DIAG = 2 ** 0.5
# (dr, dc, 비용) — 원본과 같은 순서를 유지해야 힙 타이브레이크가 같다
NEIGHBORS_8 = [(-1, 0, 1), (1, 0, 1), (0, -1, 1), (0, 1, 1),
               (-1, -1, DIAG), (-1, 1, DIAG), (1, -1, DIAG), (1, 1, DIAG)]
UNVISITED = 1e18

ENV_MAP_DIR = "SIL_MAP_DIR"
DEFAULT_MAP_DIR = os.path.join(SIL_ROOT, "t3_warehouse_map", "map")
OBSTACLE_FILE = "obstacle_mask.npy"     # 0.8 m 팽창 장애물 (bool)
STATIONS_FILE = "stations.json"         # {종류: [[x, y], ...]}
GRID_FILE = "occupancy_grid.npy"        # 값별 점유 그리드 (export_replay 의 scene.json 용, 선택)


def resolve_map_dir(explicit=None):
    """맵 디렉토리 결정 — 인자 > 환경변수 SIL_MAP_DIR > 기본값."""
    return explicit or os.environ.get(ENV_MAP_DIR) or DEFAULT_MAP_DIR


class GridMap:
    """팽창 장애물 마스크 위의 셀 변환 · A* · 가시선 단순화 · 분할."""

    def __init__(self, obstacle_mask, stations=None, cell=CELL, grid=None):
        self.obst = np.asarray(obstacle_mask).astype(bool)
        self.free_mask = ~self.obst
        self.rows, self.cols = self.obst.shape
        self.stations = stations or {}
        self.cell_size = cell
        self.grid = grid
        self.map_dir = None
        if grid is not None and grid.shape != self.obst.shape:
            raise ValueError(f"occupancy_grid {grid.shape} ≠ obstacle_mask {self.obst.shape}")

    @classmethod
    def load(cls, map_dir=None):
        """map_dir 에서 obstacle_mask · stations (· occupancy_grid 있으면) 을 읽는다."""
        d = resolve_map_dir(map_dir)
        obst = np.load(os.path.join(d, OBSTACLE_FILE))
        with open(os.path.join(d, STATIONS_FILE), encoding="utf-8") as fh:
            stations = json.load(fh)
        grid_path = os.path.join(d, GRID_FILE)
        grid = np.load(grid_path) if os.path.exists(grid_path) else None
        gm = cls(obst, stations, grid=grid)
        gm.map_dir = d
        return gm

    # ── 셀 변환 ──
    def cell(self, p):
        """월드 (x, y) → (row, col)."""
        return int(p[1] / self.cell_size), int(p[0] / self.cell_size)

    def center(self, rc):
        """(row, col) → 셀 중심 (x, y)."""
        r, c = rc
        return (c + 0.5) * self.cell_size, (r + 0.5) * self.cell_size

    def in_bounds(self, rc):
        r, c = rc
        return 0 <= r < self.rows and 0 <= c < self.cols

    def free(self, p):
        """월드 점이 맵 안이고 자유 셀인가. (원본은 범위 검사 없이 인덱싱 — 음수 좌표가 뒤에서 감겼다)"""
        rc = self.cell(p)
        return self.in_bounds(rc) and bool(self.free_mask[rc])

    def station(self, typ, i):
        """stations.json 의 (종류, 인덱스) → (x, y) 튜플."""
        return tuple(self.stations[typ][i])

    # ── 플래너 ──
    def astar(self, a, b):
        """팽창 자유 그리드 8방 A* — 대각은 양측 직교 셀도 자유일 때만(코너 컷 금지). 셀 중심 경로 또는 None."""
        start, goal = self.cell(a), self.cell(b)
        if not (self.free(a) and self.free(b)):
            return None
        g = {start: 0.0}
        came = {}
        pq = [(0.0, start)]
        while pq:
            _, cur = heapq.heappop(pq)
            if cur == goal:
                return [self.center(rc) for rc in self._backtrack(came, cur)]
            for nxt, w in self._neighbors(cur):
                ng = g[cur] + w
                if ng < g.get(nxt, UNVISITED):
                    g[nxt] = ng
                    came[nxt] = cur
                    heapq.heappush(pq, (ng + math.hypot(nxt[0] - goal[0], nxt[1] - goal[1]), nxt))
        return None

    def _neighbors(self, rc):
        """자유 이웃 셀과 이동 비용 — 원본과 같은 순서."""
        r, c = rc
        free = self.free_mask
        for dr, dc, w in NEIGHBORS_8:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < self.rows and 0 <= nc < self.cols and free[nr, nc]):
                continue
            if dr and dc and not (free[r + dr, c] and free[r, c + dc]):
                continue
            yield (nr, nc), w

    @staticmethod
    def _backtrack(came, cur):
        path = [cur]
        while cur in came:
            cur = came[cur]
            path.append(cur)
        return list(reversed(path))

    def seg_clear(self, p, q):
        """세그먼트를 SEG_SAMPLE_M 간격 샘플링해 팽창 장애물 침범 검사. (원본대로 범위 검사 없음 — 맵 안 점 전제)"""
        d = math.hypot(q[0] - p[0], q[1] - p[1])
        for k in range(int(d / SEG_SAMPLE_M) + 2):
            a = min(k * SEG_SAMPLE_M / d, 1.0) if d else 0.0
            x, y = p[0] + (q[0] - p[0]) * a, p[1] + (q[1] - p[1]) * a
            if self.obst[int(y / self.cell_size), int(x / self.cell_size)]:
                return False
        return True

    def simplify(self, path):
        """가시선 단순화 — 다음으로 보이는 가장 먼 점만 남긴다 (계단 제거)."""
        out = [path[0]]
        i = 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1 and not self.seg_clear(path[i], path[j]):
                j -= 1
            out.append(path[j])
            i = j
        return out

    @staticmethod
    def densify(pts, step=DENSIFY_STEP_M):
        """긴 직선 레그를 step 간격으로 분할 — 크로스트랙 이탈 억제. (원본대로 레그 길이가 step 의 배수면 끝점이 중복된다)"""
        out = [pts[0]]
        for a, b in zip(pts, pts[1:]):
            d = math.hypot(b[0] - a[0], b[1] - a[1])
            for k in range(1, int(d / step) + 1):
                t = k * step / d
                out.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
            out.append(b)
        return out
