"""건물 외피 — 바닥·조명, 외벽·기둥(릿지 H형강), 철골 윈드컬럼·거트, 박공지붕 호출, 바닥 마킹, 셔터 도어 (pxr 필요)."""
import os

import numpy as np
from pxr import Gf, UsdGeom, UsdLux

from .config import (CELL, CENTER_COL_Z, DOOR_FREE_Y, DOOR_SPANS, END_YS, FRAME_XS, GIRT_Z, LW,
                     MAP_DIR, MAT_DIR, OFFICE_H, RACK_W, TEX, UNIT_L, WALL_H)
from .gridutil import greedy_rects
from .materials import bind_mdl, bind_pbr
from .offices import in_office
from .prims import add_box, add_box_mesh, add_quad
from .roof import add_h_col, build_roof


def build_ground_and_lights(stage, rows, cols):
    """[0] 바닥(충돌 박스 + 시각 메시·재질) + 태양 + 현수등 격자 + 돔."""
    # 바닥(충돌 박스는 표면 아래로 내려 시각 메시와 z-파이팅 방지) + 재질 + 조명
    W, H = cols * CELL, rows * CELL
    add_box(stage, "/World/ground", -5, -5, W + 10, H + 10, -0.11, -0.01)
    floor = add_quad(stage, "/World/floor", -5, -5, W + 5, H + 5, 0.0, uv_scale=4.0)
    bind_mdl(stage, floor, "MI_Floor_02b", MAT_DIR + "/MI_Floor_02b.mdl")   # 콘크리트 창고 바닥

    # 태양광은 외부 샷·모니터 개구 채광용 — 지붕이 덮여 실내는 현수등이 주광원
    sun = UsdLux.DistantLight.Define(stage, "/World/sun")
    sun.CreateIntensityAttr(1200)
    UsdGeom.Xformable(sun.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(0, -35, 25))
    UsdGeom.Xform.Define(stage, "/World/lights")
    n_light = 0
    for gx in range(18, 111, 12):
        for gy in range(28, 89, 12):
            L = UsdLux.RectLight.Define(stage, f"/World/lights/L_{gx}_{gy}")
            L.CreateIntensityAttr(30000)
            L.CreateExposureAttr(5.5)                         # 지붕으로 태양 차단 — x32에서 x45로 보충
            L.CreateWidthAttr(1.2)
            L.CreateHeightAttr(0.6)
            xfl = UsdGeom.Xformable(L.GetPrim())
            xfl.AddTranslateOp().Set(Gf.Vec3d(gx, gy, 7.8))   # RectLight는 기본 -Z(하향) 방사 — 회전 금지
            add_box(stage, f"/World/lights/fix_{gx}_{gy}", gx - 0.7, gy - 0.35, 1.4, 0.7,
                    7.85, 7.95, collide=False)
            n_light += 1
    fixtures = stage.GetPrimAtPath("/World/lights")
    bind_mdl(stage, fixtures, "M_Glow", MAT_DIR + "/M_Glow.mdl")
    dome = UsdLux.DomeLight.Define(stage, "/World/dome")
    dome.CreateIntensityAttr(250)                             # 하늘 보조광
    print(f"[0] 드레싱: 바닥 재질 + 태양 + 현수등 {n_light}기")


def build_walls(stage, grid):
    """[1] 벽·기둥 (셀값 1) — 사무실 벽 3.0m 별도 재질, 릿지 기둥열 파편은 투명 콜라이더 + H형강 12곳."""
    # 1) 벽·기둥 (셀값 1) — 사무실 구역 벽은 3.0m 별도 재질, 그 외 8.0m.
    #    v5.7에서 기둥은 랙 흡수 12개만 남음(크기로 구분해 재질만 달리).
    walls = greedy_rects(grid == 1)
    walls_xf = UsdGeom.Xform.Define(stage, "/World/walls")
    cols_xf = UsdGeom.Xform.Define(stage, "/World/columns")
    office_xf = UsdGeom.Xform.Define(stage, "/World/office_walls")
    n_colbox = n_office = 0
    for i, (r0, c0, h, w) in enumerate(walls):
        cx, cy = (c0 + w / 2) * CELL, (r0 + h / 2) * CELL
        if in_office(cx, cy):
            add_box_mesh(stage, f"/World/office_walls/w_{i}", c0 * CELL, r0 * CELL,
                         w * CELL, h * CELL, 0.0, OFFICE_H)
            n_office += 1
            continue
        small = w * CELL < 2.2 and h * CELL < 2.2
        # x 한정 필수: 릿지 라인(y57.3)이 서·동 외벽과 만나는 지점의 벽 파편까지 투명화되면
        # 벽에 시각 구멍(콜라이더는 남아 V&V가 못 잡음 — scene_charger 샷 실측)
        if small and abs(cy - 57.3) < 1.5 and 15.0 < cx < 109.5:   # 릿지 기둥열 십자 파편
            # 콜라이더는 그리드 rect 그대로(단일 소스, 투명). 비주얼 H형강은 루프 뒤
            # columns.npy 실기둥 중심 12곳에 1개씩만 — 파편마다 세우면 십자당 ~37개
            # 군집(2차 빌드 실측 442개), small 전체 변환은 벽 구멍+지붕 관통(1차 실측).
            b = add_box(stage, f"/World/columns/col_{i}", c0 * CELL, r0 * CELL,
                        w * CELL, h * CELL, 0.0, CENTER_COL_Z)
            UsdGeom.Imageable(b.GetPrim()).MakeInvisible()
            continue
        add_box_mesh(stage, f"/World/walls/w_{i}", c0 * CELL, r0 * CELL, w * CELL, h * CELL, 0.0, WALL_H)
    # 릿지 지지 H형강 기둥 — 실기둥 중심(columns.npy) 12곳, 상단 +11.0 (실측)
    for k, (colx, coly) in enumerate(np.load(os.path.join(MAP_DIR, "columns.npy"))):
        add_h_col(stage, add_box, f"/World/columns/h_{k}", float(colx), float(coly),
                  CENTER_COL_Z, depth_axis="y", D=0.45, B=0.40, tf=0.06, tw=0.06)
        n_colbox += 1
    # 철제 창고 룩 (설계도: 철골 포털프레임 + 패널 외벽) — 외벽은 밝은 회백
    # 샌드위치 패널(패널 노멀만 사용), 기둥은 랙 프레임과 동일한 아연도금 스틸
    bind_pbr(stage, walls_xf, "SteelPanel", (0.74, 0.77, 0.80),
             normal_tex=TEX + "/T_WallBoard_01_N.png", rough=0.42, metal=0.25)
    bind_mdl(stage, cols_xf, "MI_FrameA_01", MAT_DIR + "/MI_FrameA_01.mdl")
    # 벽돌(T_WallA)은 철제 창고 안 가설 사무실과 이질적(사용자 피드백 — "붕 뜬다")
    # → 매끈한 샌드위치 패널 단색. WallBoard 노멀은 어두운 원판이라 미사용(기존 실측)
    bind_pbr(stage, office_xf, "OfficePanel", (0.90, 0.91, 0.93), rough=0.55)
    print(f"[1] 벽 {len(walls) - n_colbox - n_office} + 기둥 {n_colbox} + 사무실 벽 {n_office}(3m)")


def build_steel_envelope(stage):
    """[1c] 철골 외피 — 포털 프레임 6m 모듈의 윈드 컬럼(H형강) + 월 거트 (시각 전용)."""
    # 1c) 철골 외피 — 설계도(포털 프레임 6m 모듈)의 윈드 컬럼 + 월 거트.
    #     벽면에 밀착한 시각 전용 부재(collide=False — 벽 팽창역 안이라 플래너 무관).
    steel_xf = UsdGeom.Xform.Define(stage, "/World/steel")
    n_steel = 0
    for x in FRAME_XS:
        for y0 in (25.65, 88.8):                              # 윈드 컬럼 — H형강 (웨브 벽 직교)
            add_h_col(stage, add_box, f"/World/steel/c{n_steel}", x, y0 + 0.15, 8.8,
                      depth_axis="y", D=0.30, B=0.35)
            n_steel += 1
    for y in END_YS:
        for x0 in (14.45, 109.5):
            add_h_col(stage, add_box, f"/World/steel/c{n_steel}", x0 + 0.15, y, 8.8,
                      depth_axis="x", D=0.30, B=0.35)
            n_steel += 1
    for z in GIRT_Z:
        for y0 in (25.65, 88.95):                             # 남·북벽 전장 거트
            add_box(stage, f"/World/steel/g{n_steel}", 15.0, y0, 95.0, 0.15,
                    z, z + 0.12, collide=False)
            n_steel += 1
        for ya, yb in (DOOR_FREE_Y if z < 6.5 else ((26.0, 89.0),)):
            for x0 in (14.45, 109.55):
                add_box(stage, f"/World/steel/g{n_steel}", x0, ya, 0.15, yb - ya,
                        z, z + 0.12, collide=False)
                n_steel += 1
    bind_mdl(stage, steel_xf, "MI_FrameA_01", MAT_DIR + "/MI_FrameA_01.mdl")
    print(f"[1c] 철골 외피: 윈드 컬럼(H형강)·거트 {n_steel}개")


def build_gable_roof(stage):
    """[1d] 박공지붕 + 상부 철골 (설계 실측: 처마 +9.0 · i=15% · 릿지면 ~+14.5 · 모니터 4.5m)."""
    n_roof, n_pur = build_roof(stage, add_box=add_box, bind_pbr=bind_pbr, bind_mdl=bind_mdl,
                               mat_dir=MAT_DIR, frame_xs=FRAME_XS)
    print(f"[1d] 박공지붕: 트러스·모니터·브레이싱 부재 {n_roof} + 퍼린 {n_pur}")


def build_floor_markings(stage, grid, rack_units):
    """[3c] 바닥 마킹 — 렉 세그먼트 안전선(황) + 스테이션 마킹(청, 셀값 3). /World/markings 정의."""
    # 바닥 마킹 — 렉 세그먼트 안전선(황) + 스테이션 마킹(청): 시인성 + 레이아웃 데이터의 시각화
    UsdGeom.Xform.Define(stage, "/World/markings")
    n_mark = 0
    for bi, ru in enumerate(rack_units):
        xc, ys, nu = ru[0], ru[1], ru[2]
        half_w = RACK_W / 2 if (len(ru) <= 3 or int(ru[3]) == 2) else RACK_W / 4
        bx0, by0 = xc - half_w - 0.27, ys - 0.27
        bx1, by1 = xc + half_w + 0.27, ys + int(nu) * UNIT_L + 0.27
        for j, (qx0, qy0, qx1, qy1) in enumerate([
                (bx0, by0, bx1, by0 + LW), (bx0, by1 - LW, bx1, by1),
                (bx0, by0, bx0 + LW, by1), (bx1 - LW, by0, bx1, by1)]):
            q = add_quad(stage, f"/World/markings/rk{bi}_{j}", qx0, qy0, qx1, qy1, 0.01, 4.0)
            UsdGeom.Gprim(q.GetPrim()).CreateDisplayColorAttr([(1.0, 0.78, 0.05)])
            n_mark += 1
    for i, (r0, c0, hh, ww) in enumerate(greedy_rects(grid == 3)):
        q = add_quad(stage, f"/World/markings/st{i}", c0 * CELL, r0 * CELL,
                     (c0 + ww) * CELL, (r0 + hh) * CELL, 0.012, 4.0)
        UsdGeom.Gprim(q.GetPrim()).CreateDisplayColorAttr([(0.15, 0.45, 0.9)])
        n_mark += 1
    print(f"[3c] 바닥 마킹 {n_mark}개 (안전선·스테이션)")


def door_x0(face, dsign, depth):
    """실내면 face에서 실내 쪽으로 depth 돌출한 박스의 x0 (dsign: 실내 방향 부호)."""
    return face if dsign > 0 else face - depth


def build_doors(stage):
    """[3e] 도어 드레싱 — 서·동 박공벽 문 4곳: 폐쇄 셔터 패널+리브, 롤 하우징, 잼 포스트, 상부 메꿈, 경계 스트립."""
    # 3e) 도어 드레싱 — 서·동 박공벽 문 4곳(그리드 실측 y 38.3~44.6 / 70.3~76.7).
    #     그리드의 문 구간은 값4 스트립 양옆에 전고 벽 라미나 2겹(예: x 14.1~14.2,
    #     14.3~14.4)이 남아 물리적으로 봉인돼 있다(USD bbox 스캔 실측) — 그리드가 단일
    #     소스이므로 씬도 "셔터 내려진 상태"로 표현한다: 라미나 위에 안팎 셔터 패널+
    #     리브를 씌우고 하우징·잼 포스트·상부 메꿈을 붙인다. 시각물은 전부 collide=False
    #     (스캔 밴드 z 0.2~1.2와 겹치는 높이 — 콜라이더는 라미나 벽이 담당).
    door_xf = UsdGeom.Xform.Define(stage, "/World/doors")
    shut_xf = UsdGeom.Xform.Define(stage, "/World/shutters")
    UsdGeom.Xform.Define(stage, "/World/shutters_rib")
    n_door = 0
    for face, dsign, wx0 in ((14.40, 1.0, 14.10), (109.90, -1.0, 109.90)):
        for ya, yb in DOOR_SPANS:
            add_box_mesh(stage, f"/World/walls/door_top_{n_door}", wx0 + 0.005, ya, 0.29,
                         yb - ya, 6.2, WALL_H)                 # 개구 상부 메꿈 (5mm 인셋 — 라미나 면과 z-파이팅 방지)
            add_box(stage, f"/World/doors/box_{n_door}", door_x0(face, dsign, 0.45),
                    ya - 0.25, 0.45, yb - ya + 0.50, 5.55, 6.20, collide=False)  # 셔터 롤 하우징
            # 셔터 커튼(폐쇄 상태) — 벽 라미나 안팎 면을 덮는 패널 + 가로 리브
            in_x0 = face if dsign > 0 else face - 0.06              # 실내면 패널
            out_x0 = wx0 - 0.06 if dsign > 0 else wx0 + 0.30       # 실외면 패널
            for side, px0, z1 in (("in", in_x0, 5.55), ("out", out_x0, 6.20)):
                # 실외면은 헤더(6.2)까지 — 라미나가 안 덮는 문 가장자리 z5.55~6.2 슬롯 차단
                add_box(stage, f"/World/shutters/{side}_{n_door}", px0, ya + 0.05,
                        0.06, yb - ya - 0.10, 0.05, z1, collide=False)
                for rk in range(11):
                    r = add_box(stage, f"/World/shutters_rib/{side}_{n_door}_{rk}",
                                px0 - 0.01, ya + 0.05, 0.08, yb - ya - 0.10,
                                0.55 + 0.5 * rk, 0.59 + 0.5 * rk, collide=False)
                    UsdGeom.Gprim(r.GetPrim()).CreateDisplayColorAttr([(0.42, 0.44, 0.47)])
            for pj, yp in enumerate((ya - 0.27, yb + 0.02)):
                add_box(stage, f"/World/doors/post_{n_door}_{pj}", door_x0(face, dsign, 0.30),
                        yp, 0.30, 0.25, 0.0, 6.20, collide=False)   # 잼 포스트
            sx = face + (0.20 if dsign > 0 else -0.35)
            q = add_quad(stage, f"/World/markings/door_{n_door}", sx, ya, sx + 0.15, yb,
                         0.012, 2.0)                                # 실내 경계 황색 스트립
            UsdGeom.Gprim(q.GetPrim()).CreateDisplayColorAttr([(1.0, 0.78, 0.05)])
            n_door += 1
    bind_mdl(stage, door_xf, "MI_FrameA_01", MAT_DIR + "/MI_FrameA_01.mdl")
    bind_pbr(stage, shut_xf, "ShutterSteel", (0.58, 0.60, 0.63), rough=0.45, metal=0.5)
    print(f"[3e] 도어 드레싱 {n_door}곳 (폐쇄 셔터·하우징·잼 포스트·상부 메꿈)")
