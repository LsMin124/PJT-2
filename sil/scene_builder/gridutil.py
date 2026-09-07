"""numpy·zlib 순수 함수 — 그리디 메싱, 결정적 난수, V&V 마스크·omap 버퍼 정렬 (pxr 불필요)."""
import zlib

import numpy as np

from .config import CELL, OFFICES, RACK_D, UNIT_L


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


def rnd(*key):
    """결정적 의사난수 [0,1) — 재실행해도 같은 배치 (성공 판정·diff 재현성)."""
    return (zlib.crc32(repr(key).encode()) % 10000) / 10000.0


def rack_footprint_mask(rack_units, shape):
    """V&V — 랙 유닛 풋프린트(그리드 값2)의 셀 마스크 (원본 [5] rackmask)."""
    rackmask = np.zeros(shape, dtype=bool)
    for ru in rack_units:
        xc, ys, n_units = ru[0], ru[1], int(ru[2])
        half = RACK_D if (len(ru) <= 3 or int(ru[3]) == 2) else RACK_D / 2
        r0, r1 = int(ys / CELL), int((ys + n_units * UNIT_L) / CELL)
        c0, c1 = int((xc - half) / CELL), int((xc + half) / CELL)
        rackmask[r0:r1, c0:c1] = True
    return rackmask


def office_footprint_mask(shape):
    """V&V — 사무실 구역 셀 마스크 (원본 [5] office_mask)."""
    office_mask = np.zeros(shape, dtype=bool)
    for x0, y0, x1, y1 in OFFICES:            # 가구 에셋 내장 콜라이더 — 그리드 밖 시각물
        office_mask[int(y0 / CELL):int(y1 / CELL), int(x0 / CELL):int(x1 / CELL)] = True
    return office_mask


def omap_to_occ(buf, rows, cols):
    """omap 1D 버퍼 → 점유(값 4) 마스크 (rows x cols). 크기 불일치면 None (원본 [5])."""
    occ = None
    if buf.size:
        dims = (int(round(rows)), int(round(cols)))
        for shape in (dims, dims[::-1]):
            if buf.size == shape[0] * shape[1]:
                occ = (buf.reshape(shape) == 4)
                if shape != dims:
                    occ = occ.T
                occ = np.fliplr(occ)                       # omap 버퍼는 x 미러 (실측: 보정 시 벽 재현율 100%)
                break
    return occ
