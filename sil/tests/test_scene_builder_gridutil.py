"""scene_builder.gridutil·config 순수 함수 테스트 — numpy 만 필요 (pxr·Isaac 불필요)."""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))   # sil/

from scene_builder import config  # noqa: E402
from scene_builder.gridutil import (greedy_rects, office_footprint_mask, omap_to_occ,  # noqa: E402
                                    rack_footprint_mask, rnd)


def _rects_to_mask(rects, shape):
    m = np.zeros(shape, dtype=bool)
    for r0, c0, h, w in rects:
        assert not m[r0:r0 + h, c0:c0 + w].any(), "rect 겹침"
        m[r0:r0 + h, c0:c0 + w] = True
    return m


# ── greedy_rects ──
def test_greedy_rects_empty_mask_returns_no_rects():
    assert greedy_rects(np.zeros((3, 4), dtype=bool)) == []


def test_greedy_rects_single_cell():
    m = np.zeros((3, 4), dtype=bool)
    m[1, 2] = True
    assert greedy_rects(m) == [(1, 2, 1, 1)]


def test_greedy_rects_full_block_merges_to_one_rect():
    assert greedy_rects(np.ones((3, 4), dtype=bool)) == [(0, 0, 3, 4)]


def test_greedy_rects_vertical_merge_requires_identical_run():
    # 행 런 (0,2) 가 2행 연속 → 세로 병합, 3행째는 (0,1) 이라 별도 rect
    m = np.array([[1, 1, 0], [1, 1, 0], [1, 0, 0]], dtype=bool)
    assert sorted(greedy_rects(m)) == [(0, 0, 2, 2), (2, 0, 1, 1)]


def test_greedy_rects_two_separate_runs_in_a_row():
    m = np.array([[1, 0, 1, 1]], dtype=bool)
    assert sorted(greedy_rects(m)) == [(0, 0, 1, 1), (0, 2, 1, 2)]


def test_greedy_rects_closes_runs_that_do_not_continue():
    # 1행: (0,1),(2,3) / 2행: (0,3) — 런이 달라져 1행 rect 2개가 닫히고 2행은 새 rect
    m = np.array([[1, 0, 1], [1, 1, 1]], dtype=bool)
    assert sorted(greedy_rects(m)) == [(0, 0, 1, 1), (0, 2, 1, 1), (1, 0, 1, 3)]


def test_greedy_rects_reconstructs_random_masks_exactly():
    rng = np.random.default_rng(7)
    for _ in range(20):
        m = rng.random((17, 23)) < 0.55
        rects = greedy_rects(m)
        assert np.array_equal(_rects_to_mask(rects, m.shape), m)


# ── rnd ──
def test_rnd_is_deterministic_and_in_unit_interval():
    vals = [rnd(i, "a", 0.5) for i in range(200)]
    assert vals == [rnd(i, "a", 0.5) for i in range(200)]
    assert all(0.0 <= v < 1.0 for v in vals)


def test_rnd_distinguishes_key_type_and_order():
    assert rnd(1) != rnd("1")
    assert rnd(1, 2) != rnd(2, 1)
    assert rnd(3, "x") == (__import__("zlib").crc32(repr((3, "x")).encode()) % 10000) / 10000.0


# ── V&V 마스크 ──
def test_rack_footprint_mask_double_row_uses_full_depth():
    shape = (200, 200)
    m = rack_footprint_mask(np.array([[5.0, 3.0, 2]]), shape)      # len 3 → 2열 (half = RACK_D)
    r0, r1 = int(3.0 / config.CELL), int((3.0 + 2 * config.UNIT_L) / config.CELL)
    c0, c1 = int((5.0 - config.RACK_D) / config.CELL), int((5.0 + config.RACK_D) / config.CELL)
    assert m.sum() == (r1 - r0) * (c1 - c0)
    assert m[r0:r1, c0:c1].all()


def test_rack_footprint_mask_single_row_uses_half_depth():
    shape = (200, 200)
    one = rack_footprint_mask(np.array([[5.0, 3.0, 2, 1]]), shape)
    two = rack_footprint_mask(np.array([[5.0, 3.0, 2, 2]]), shape)
    assert 0 < one.sum() < two.sum()
    assert not (one & ~two).any()                                   # 1열 풋프린트 ⊂ 2열 풋프린트


def test_office_footprint_mask_matches_office_rects():
    shape = (1000, 1200)
    m = office_footprint_mask(shape)
    expected = sum((int(y1 / config.CELL) - int(y0 / config.CELL)) * (int(x1 / config.CELL) - int(x0 / config.CELL))
                   for x0, y0, x1, y1 in config.OFFICES)
    assert m.sum() == expected
    x0, y0, x1, y1 = config.OFFICES[0]
    assert m[int((y0 + y1) / 2 / config.CELL), int((x0 + x1) / 2 / config.CELL)]
    assert not m[0, 0]


# ── omap 버퍼 정렬 ──
def test_omap_to_occ_returns_none_for_empty_or_mismatched_buffer():
    assert omap_to_occ(np.array([]), 3, 4) is None
    assert omap_to_occ(np.zeros(5), 3, 4) is None


def test_omap_to_occ_reshapes_and_mirrors_x():
    rows, cols = 3, 4
    buf = np.full(rows * cols, 5)
    buf[1 * cols + 0] = 4                     # (r=1, c=0) 점유 → 미러 후 (1, cols-1)
    occ = omap_to_occ(buf, rows, cols)
    assert occ.shape == (rows, cols) and occ.dtype == bool
    assert occ.sum() == 1 and occ[1, cols - 1]


# ── config 경로·파생 상수 (원본 위치 기준 상대경로 유지) ──
def test_config_paths_keep_original_layout():
    norm = os.path.normpath
    assert norm(config.HERE).endswith(os.path.join("sil", "t3_warehouse"))
    assert norm(config.MAP_DIR).endswith(os.path.join("sil", "t3_warehouse_map", "map"))
    assert norm(config.OUT_DIR).endswith(os.path.join("sil", "t3_warehouse", "out"))
    assert norm(config.USD_PATH).endswith(os.path.join("sil", "t3_warehouse", "warehouse_scene.usd"))


def test_config_derived_constants():
    assert config.DECK_SCALE == pytest.approx(0.75)
    assert config.LOAD_Z == pytest.approx((0.0, 1.38, 2.73))
    assert config.RIDGE_Y == pytest.approx(57.5)
    assert len(config.FRAME_XS) == 15 and config.FRAME_XS[-1] == pytest.approx(104.1)
    assert len(config.OFFICE_WALL) == 18 and len(config.DESKS) == 12
