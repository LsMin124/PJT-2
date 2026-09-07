"""랙 — 래핑 파렛트 더미·바닥 파렛트 블록(셀값 6), 프레임·데크 조립, 피킹 낱박스 적재 (pxr 필요)."""
from pxr import Gf, UsdGeom, UsdShade

from .config import (BOX_A, BOX_B, BOX_C, BOX_KIND, CELL, DECK_SCALE, DECK_Z, FORK_POS, FRAME_USD,
                     LOAD_Z, PALLET_D, PALLET_L, PALLET_USD, RACK_D, SHELF_USD, UNIT_L)
from .gridutil import greedy_rects, rnd
from .materials import define_stretch_wrap
from .prims import add_asset, add_box
from .stations import add_forklift


def add_pallet_stack(stage, path, x, y, rz, layers, wrap_mtl):
    """래핑 파렛트 더미(시각 전용): 파렛트 + 교차 적재 박스 layers층 + 반투명 랩.
    실제 파렛타이징처럼 층마다 90° 교차 — 2층 1.2m / 3층 1.7m."""
    w = UsdGeom.Xform.Define(stage, path)
    xf = UsdGeom.Xformable(w.GetPrim())
    xf.AddTranslateOp().Set(Gf.Vec3d(x, y, 0.0))
    xf.AddRotateZOp().Set(rz)

    def sub(name, usd, lx, ly, lz, lrz=None):
        s = UsdGeom.Xform.Define(stage, f"{path}/{name}")
        sx = UsdGeom.Xformable(s.GetPrim())
        sx.AddTranslateOp().Set(Gf.Vec3d(lx, ly, lz))
        if lrz is not None:
            sx.AddRotateZOp().Set(lrz)
        a = UsdGeom.Xform.Define(stage, f"{path}/{name}/a")
        a.GetPrim().GetReferences().AddReference(usd)
        a.GetPrim().SetInstanceable(True)

    sub("pal", PALLET_USD, 0, 0, 0)
    z = 0.21
    sub("l1a", BOX_A, 0, -0.25, z)
    sub("l1b", BOX_A, 0, 0.25, z)
    z += 0.50
    if layers >= 2:
        sub("l2a", BOX_A, -0.25, 0, z, lrz=90)
        sub("l2b", BOX_A, 0.25, 0, z, lrz=90)
        z += 0.50
    if layers >= 3:
        sub("l3a", BOX_B, 0, -0.25, z)
        sub("l3b", BOX_B, 0, 0.25, z)
        z += 0.50
    if layers >= 4:                            # 최상단 테이퍼 층 — 재고 만재
        sub("l4a", BOX_C, 0, -0.25, z)
        sub("l4b", BOX_C, 0, 0.25, z)
        z += 0.25
    wrap = add_box(stage, f"{path}/wrap", -0.56, -0.47, 1.12, 0.94, 0.16, z + 0.03,
                   collide=False)
    UsdShade.MaterialBindingAPI.Apply(wrap.GetPrim()).Bind(wrap_mtl)


def build_pallet_blocks(stage, grid):
    """[2b] 바닥 파렛트 블록 (셀값 6) — 투명 콜라이더 + 지게차 + 래핑 파렛트 더미 자동 채움."""
    # 2b) 바닥 파렛트 블록 (셀값 6) — 구형 창고 블록 스태킹. 콜라이더는 그리드 rect
    #     그대로(투명, 라이다·플래너 단일 소스), 비주얼은 rect 안에 래핑 파렛트
    #     더미를 자동 채움. 랩은 반투명 OmniPBR(기성 래핑 에셋 없음 — 합성).
    wrap_mtl = define_stretch_wrap(stage)
    UsdGeom.Xform.Define(stage, "/World/pallets")
    n_stack = 0
    add_forklift(stage)
    prects = greedy_rects(grid == 6)
    for i, (r0, c0, hh, ww) in enumerate(prects):
        b = add_box(stage, f"/World/pallets/col_{i}", c0 * CELL, r0 * CELL,
                    ww * CELL, hh * CELL, 0.0, 2.05)
        UsdGeom.Imageable(b.GetPrim()).MakeInvisible()
        x0, y0, w_m, h_m = c0 * CELL, r0 * CELL, ww * CELL, hh * CELL
        horiz = w_m >= h_m                        # 장변 방향으로 파렛트 장축 정렬
        along, deep = (w_m, h_m) if horiz else (h_m, w_m)
        nl, nd = int(along // PALLET_L), max(int(deep // PALLET_D), 1)
        oa = (along - nl * PALLET_L) / 2
        od = (deep - nd * PALLET_D) / 2
        for j in range(nl):
            for k in range(nd):
                if rnd(i, j, k) < 0.03:           # 빈 자리 최소 — 꽉 찬 창고
                    continue
                a = oa + PALLET_L * (j + 0.5) + (rnd(i, j, k, "a") - 0.5) * 0.06
                d = od + PALLET_D * (k + 0.5) + (rnd(i, j, k, "d") - 0.5) * 0.06
                px, py = (x0 + a, y0 + d) if horiz else (x0 + d, y0 + a)
                if abs(px - FORK_POS[0]) < 2.0 and abs(py - FORK_POS[1]) < 2.7:
                    continue                       # 지게차 주차 포켓
                rz = (0.0 if horiz else 90.0) + (rnd(i, j, k, "r") - 0.5) * 7
                r = rnd(i, j, k, "t")
                add_pallet_stack(stage, f"/World/pallets/s{i}_{j}_{k}", px, py, rz,
                                 layers=4 if r > 0.45 else (3 if r > 0.1 else 2), wrap_mtl=wrap_mtl)
                n_stack += 1
    print(f"[2b] 바닥 파렛트 블록 {len(prects)}rect → 래핑 더미 {n_stack}개", flush=True)


def build_racks(stage, rack_units):
    """[3] 렉 조립 — rack_units [x중심, y시작, 유닛수(, 열수)] 세로형 더블로우: 프레임 n+1 + 데크 2단."""
    # 3) 렉 — 부품 조립 (rack_units: [x중심, y시작, 유닛수] — 세로형 더블로우)
    #    v6.0 구역제: 유닛 3.0m(반쪽당 3랙×2열=구역 6랙). SM_RackShelf 베이는 4.0m라
    #    장축 DECK_SCALE(0.75) 스케일로 정합. 컴포즈된 데크는 가로 정렬이므로 세로형에선
    #    z축 90° 회전이 "필요"하다 (근거는 참조 컴포즈 bbox 실측).
    UsdGeom.Xform.Define(stage, "/World/racks")
    n_frame = n_shelf = 0
    for bi, ru in enumerate(rack_units):
        xc, ys, n_units = ru[0], ru[1], int(ru[2])
        n_rows = int(ru[3]) if len(ru) > 3 else 2          # v6.0.1: 양끝 열은 1열
        for line in range(n_rows):                         # 등맞대기 2줄 (x = xc ± 0.54) 또는 단열
            lx = xc if n_rows == 1 else xc + RACK_D * (line - 0.5)
            root = f"/World/racks/b{bi}_l{line}"
            UsdGeom.Xform.Define(stage, root)
            for k in range(n_units + 1):                   # 프레임 — 유닛 경계 공유 (n+1개)
                add_asset(stage, f"{root}/frame_{k}", FRAME_USD,
                          lx, ys + UNIT_L * k, rot_z=90.0)
                n_frame += 1
            for u in range(n_units):                       # 유닛당 데크 2단 (+바닥 = 3단)
                yc = ys + UNIT_L * u + UNIT_L / 2
                for dz in DECK_Z:
                    add_asset(stage, f"{root}/shelf_{u}_{int(dz*100)}", SHELF_USD,
                              lx, yc, z=dz, rot_z=90.0, scale=(DECK_SCALE, 1.0, 1.0))
                    n_shelf += 1
    print(f"[3] 렉 조립: 프레임 {n_frame} + 데크 {n_shelf}", flush=True)


def build_rack_cargo(stage, rack_units):
    """[3b] 렉 화물 (시각 전용) — 슬롯별 채움 편차·지터·소적재·뒷줄 박스 (결정적 rnd)."""
    # 3b) 렉 화물 (시각 전용, 랙 풋프린트 안 — 그리드 값2 그대로):
    #     피킹 스테이지 — 파렛트 재고에서 옮겨온 낱박스가 선반에 "어느 정도"
    #     차 있는 모습(슬롯별 채움 편차, 간격·지터·소적재). 단높이 1.35/2.7이라
    #     낱박스+소적재(≤0.75m)도 층간 여유 0.97m로 관통 없음.
    #     데크 상판은 배치 z +0.03 (컴포즈 bbox 실측: 데크 z -0.345~+0.025).
    n_cargo = 0
    for bi, ru in enumerate(rack_units):
        xc, ys, n_units = ru[0], ru[1], int(ru[2])
        n_rows = int(ru[3]) if len(ru) > 3 else 2
        for line in range(n_rows):
            lx = xc if n_rows == 1 else xc + RACK_D * (line - 0.5)
            for u in range(n_units):
                yc = ys + UNIT_L * u + UNIT_L / 2
                root = f"/World/racks/cargo_b{bi}_l{line}_u{u}"
                UsdGeom.Xform.Define(stage, root)
                for li, lz in enumerate(LOAD_Z):
                    dens = 0.45 + rnd(bi, line, u, li, "d") * 0.45  # 슬롯별 채움 편차
                    yy = yc - (UNIT_L / 2 - 0.15)                   # 베이 길이 파생 (v6.0 3m)
                    while True:
                        r = rnd(bi, line, u, li, round(yy, 2))
                        usd, wid, hgt = BOX_KIND[int(r * 3) % 3]
                        yy += wid / 2
                        if yy + wid / 2 > yc + (UNIT_L / 2 - 0.05):
                            break
                        key = f"{li}_{int((yy + 2) * 100)}"
                        bx = lx + (rnd(bi, li, round(yy, 2), "x") - 0.5) * 0.3
                        add_asset(stage, f"{root}/d{key}", usd, bx, yy, z=lz,
                                  rot_z=90.0 + (rnd(bi, li, round(yy, 2), "r") - 0.5) * 14,
                                  instance=True)
                        n_cargo += 1
                        if rnd(bi, li, round(yy, 2), "s") > 0.78:   # 가끔 2단 소적재
                            add_asset(stage, f"{root}/s{key}", BOX_C, bx, yy,
                                      z=lz + hgt, rot_z=90.0, instance=True)
                            n_cargo += 1
                        if rnd(bi, li, round(yy, 2), "b") > 0.7:    # 가끔 뒷줄 박스
                            add_asset(stage, f"{root}/r{key}", BOX_B, lx + 0.27, yy,
                                      z=lz, rot_z=90.0, instance=True)
                            n_cargo += 1
                        yy += wid / 2 + 0.12 + (1 - dens) * 0.9 * rnd(bi, li, round(yy, 2), "g")
    print(f"[3b] 렉 화물: 피킹 낱박스 {n_cargo}점", flush=True)
