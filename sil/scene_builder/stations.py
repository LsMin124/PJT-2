"""작업 스테이션 — 컨베이어·패킹 테이블(on_conveyor), 지게차, 충전 스테이션, 스테이션 앵커 (pxr 필요)."""
import json
import os

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics
from scipy import ndimage

from .config import (ASSETS, CELL, CONV_H, CONV_SEC_L, CONV_USD, CONVEYORS_VIS, E_WALL, FORK_POS,
                     MAP_DIR, MAT_DIR, PACK_USD, TABLE_H)
from .gridutil import greedy_rects
from .materials import bind_mdl, bind_pbr
from .prims import add_asset, add_box, add_quad


def on_conveyor(cx, cy):
    for x0, y0, L, vert in CONVEYORS_VIS:
        if vert and x0 - 0.3 <= cx <= x0 + 1.2 and y0 - 0.3 <= cy <= y0 + L + 0.3:
            return True
        if not vert and x0 - 0.3 <= cx <= x0 + L + 0.3 and y0 - 0.3 <= cy <= y0 + 1.2:
            return True
    return False


def build_conveyors_and_tables(stage, grid):
    """[2] 컨베이어·작업대 (셀값 5) — 투명 콜라이더 + A08 벨트 비주얼 · 패킹 테이블 비주얼."""
    # 2) 컨베이어·작업대 (셀값 5) — rect 중심이 컨베이어 라인 밴드 위인지로 구분.
    #    (폭 기준은 함정: qc 작업대가 컨베이어와 셀이 붙어 그리디 분할되면 0.6m 조각이
    #    되어 컨베이어로 오분류 — 14/16개 실측 후 좌표 기준으로 교체)
    convs = greedy_rects(grid == 5)
    UsdGeom.Xform.Define(stage, "/World/conveyors")
    UsdGeom.Xform.Define(stage, "/World/worktables")
    n_conv = n_tab = 0
    bench_mask = np.zeros_like(grid, dtype=bool)
    for i, (r0, c0, h, w) in enumerate(convs):
        if on_conveyor((c0 + w / 2) * CELL, (r0 + h / 2) * CELL):   # 컨베이어 — 콜라이더 전용(비주얼은 A08)
            b = add_box(stage, f"/World/conveyors/c_{i}", c0 * CELL, r0 * CELL,
                        w * CELL, h * CELL, 0.0, CONV_H)
            UsdGeom.Imageable(b.GetPrim()).MakeInvisible()
            n_conv += 1
        else:                                                 # 작업대 — 콜라이더는 그리드 rect 그대로
            b = add_box(stage, f"/World/worktables/t_{i}", c0 * CELL, r0 * CELL,
                        w * CELL, h * CELL, 0.0, TABLE_H)     # (투명), 비주얼은 packing_table
            UsdGeom.Imageable(b.GetPrim()).MakeInvisible()
            bench_mask[r0:r0 + h, c0:c0 + w] = True
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
    # 작업대 비주얼 — packing_table (컴포즈 실측 2.47x0.78 h1.08, 벤치 rect 2.3~3.2x1.3).
    # 그리디 분할로 벤치 하나가 2rect가 될 수 있어 연결 성분 중심으로 배치(성분 14 실측)
    blab, n_bench = ndimage.label(bench_mask)
    for k in range(1, n_bench + 1):
        ys, xs = np.where(blab == k)
        bx, by = (xs.mean() + 0.5) * CELL, (ys.mean() + 0.5) * CELL
        for cx0, cy0, cL, cv in CONVEYORS_VIS:                # qc 검수대 — 벨트 비주얼과 겹침
            if not cv and cx0 - 0.5 <= bx <= cx0 + cL + 0.5:  #  방지: 북측으로 0.5m 이격
                top = cy0 + 1.05
                if cy0 - 0.5 < by < top + 0.5:
                    by = top + 0.5
        wtab = add_asset(stage, f"/World/worktables/pt_{k}", PACK_USD, bx, by,
                         rot_z=180.0 if by < 30 else 0.0)     # VAS 열(y≈28.2)은 도킹이 북측
        rb = stage.GetPrimAtPath(f"/World/worktables/pt_{k}/asset/container_h20")
        if rb.IsValid():                                      # 동봉 컨테이너가 리지드바디 — play 중 낙하 방지
            UsdPhysics.RigidBodyAPI(rb).CreateRigidBodyEnabledAttr(False)
        # 에셋 내장 정적 콜라이더 전체 비활성 — 콜라이더 단일 소스는 그리드 투명 박스.
        # (qc 벤치를 벨트에서 이격하자 내장 콜라이더가 팽창 마스크 밖 0.1m 줄로 새어
        #  V&V 오검출 56셀 — 실측 후 원칙대로 차단)
        for p in Usd.PrimRange(wtab.GetPrim()):
            if p.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI(p).CreateCollisionEnabledAttr(False)
    print(f"[2] 컨베이어 콜라이더 {n_conv}(투명) + 비주얼 섹션 {n_sec} · 패킹 테이블 {n_bench}(rect {n_tab})")


def add_forklift(stage):
    """지게차 — 인바운드 파렛트 밴드 안 주차 (/World/pallets 하위, 원본 [2b] 블록)."""
    # 지게차 — 인바운드 파렛트 밴드(셀값 6, 15.6~17.6 x 47~66) 안 주차. 정적 콜라이더
    # 3개(리지드 없음, 컴포즈 실측 1.21x3.49 h2.15)라 st 마스크 안 → V&V 오검출 없음
    add_asset(stage, "/World/pallets/forklift", ASSETS + "/Isaac/Props/Forklift/forklift.usd",
              FORK_POS[0], FORK_POS[1], rot_z=8.0)


def load_stations():
    """stations.json 로드 — {type: [(x, y), ...]} (charger 6기 포함)."""
    with open(os.path.join(MAP_DIR, "stations.json")) as f:
        stations = json.load(f)
    return stations


def build_chargers(stage, stations):
    """[3d] 충전 스테이션 — charger 6기 동벽 벽걸이 캐비닛·LED·케이블 트레이 + 베이 도장 (전부 collide=False)."""
    # 3d) 충전 스테이션 — stations.json charger 6기. 도킹 셀(x 107.8)은 자유 공간(값 3)
    #     이므로 실물 캐비닛은 동벽 실내면 x=109.90(그리드 실측: 동벽 109.90~110.20)에
    #     벽걸이. 전부 collide=False — 스캔 밴드(z 0.2~1.2)와 겹치는 높이라 콜라이더를
    #     켜면 V&V 오검출. 벽 팽창역 0.8m(→x 109.10) 안이라 플래너·그리드 무관.
    chg_xf = UsdGeom.Xform.Define(stage, "/World/chargers")
    UsdGeom.Xform.Define(stage, "/World/chargers_trim")
    led_xf = UsdGeom.Xform.Define(stage, "/World/chargers_led")
    chg_ys = [cy for _, cy in stations["charger"]]
    for ci, (cx, cy) in enumerate(stations["charger"]):
        add_box(stage, f"/World/chargers/body_{ci}", E_WALL - 0.35, cy - 0.40, 0.35, 0.80,
                0.40, 1.70, collide=False)
        b = add_box(stage, f"/World/chargers_trim/cap_{ci}", E_WALL - 0.40, cy - 0.44, 0.40,
                    0.88, 1.70, 1.78, collide=False)
        UsdGeom.Gprim(b.GetPrim()).CreateDisplayColorAttr([(0.95, 0.75, 0.08)])
        p = add_box(stage, f"/World/chargers_trim/panel_{ci}", E_WALL - 0.37, cy - 0.30, 0.02,
                    0.60, 0.55, 1.50, collide=False)
        UsdGeom.Gprim(p.GetPrim()).CreateDisplayColorAttr([(0.10, 0.11, 0.12)])
        add_box(stage, f"/World/chargers_led/led_{ci}", E_WALL - 0.38, cy - 0.05, 0.02, 0.10,
                1.55, 1.60, collide=False)
        q = add_quad(stage, f"/World/markings/chgpad_{ci}", cx - 0.48, cy - 0.48,
                     cx + 0.48, cy + 0.48, 0.014, 2.0)         # 도킹 셀 고무 매트 도장
        UsdGeom.Gprim(q.GetPrim()).CreateDisplayColorAttr([(0.13, 0.13, 0.15)])
    add_box(stage, "/World/chargers/tray", E_WALL - 0.16, min(chg_ys) - 0.5, 0.14,
            max(chg_ys) - min(chg_ys) + 1.0, 1.82, 1.92, collide=False)   # 케이블 트레이
    for k in range(7):                                         # 베이 구획선 (3m 피치 ±1.5)
        by = 48.5 + 3.0 * k
        q = add_quad(stage, f"/World/markings/chgline_{k}", 106.9, by - 0.06, 109.3,
                     by + 0.06, 0.011, 2.0)
        UsdGeom.Gprim(q.GetPrim()).CreateDisplayColorAttr([(1.0, 0.78, 0.05)])
    bind_pbr(stage, chg_xf, "ChargerSteel", (0.24, 0.25, 0.28), rough=0.45, metal=0.35)
    bind_mdl(stage, led_xf, "M_Glow", MAT_DIR + "/M_Glow.mdl")
    print(f"[3d] 충전 스테이션 {len(stations['charger'])}기 (동벽 벽걸이·베이 도장)")


def build_anchors(stage, stations):
    """[3f] 스테이션 앵커 — stations.json 전 지점을 /World/anchors/<type>_<i> Xform으로 (기하 없음)."""
    # 3f) 스테이션 앵커 — stations.json 전 지점을 /World/anchors/<type>_<i> Xform으로.
    #     기하 없음(시각·물리 무관). 이후 프로젝트(재생기·warehouse_sim·FMS)가 좌표를
    #     씬에서 직접 질의하는 표준 통로 — 그리드·씬 이중 관리 방지.
    UsdGeom.Xform.Define(stage, "/World/anchors")
    n_anch = 0
    for typ, pts in stations.items():
        for ai, (ax, ay) in enumerate(pts):
            a = UsdGeom.Xform.Define(stage, f"/World/anchors/{typ}_{ai}")
            UsdGeom.Xformable(a.GetPrim()).AddTranslateOp().Set(
                Gf.Vec3d(float(ax), float(ay), 0.0))
            n_anch += 1
    print(f"[3f] 스테이션 앵커 {n_anch}개 (/World/anchors)")
