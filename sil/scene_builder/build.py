"""오케스트레이터 — 원본 build_scene.py 의 실행 단계 0~6 을 순서대로 호출한다 (엔트리 main)."""
# T3 씬 빌더 — occupancy grid + rack_units를 Isaac USD 씬으로 세운다 (v6.0 맵).
#
# 원칙: 그리드가 곧 씬이다 (DES와 SIL의 단일 소스).
#   - 벽·기둥(셀값 1): 그리디 메싱으로 병합한 박스 (높이 8.0m)
#   - 컨베이어·작업대(셀값 5): 폭으로 구분 — 폭 0.9m 컨베이어는 투명 콜라이더
#     + ConveyorBelt_A08 비주얼, 폭 1.2m+ 작업대는 가시 박스 (높이 0.9m — 라이다 평면 위)
#   - 렉(rack_units.npy): 세로형 더블로우 — [x중심, y시작, 유닛수], v6.0 유닛 3.0m
#     (구역제: 통로 반쪽 3랙×2열 = 구역 6랙, A~V 22구역). SM_RackShelf_01 베이는
#     4.0m라 장축 0.75 스케일 정합, 부품을 z축 90° 회전해 y축 정렬 조립
#   - V&V: isaacsim.asset.gen.omap으로 씬→점유맵 재생성 후 원 그리드와 diff
#
# 실행:
#   cd ~/isaacsim && ./python.sh <repo>/sil/apps/build_scene.py   (옛 경로 t3_warehouse/build_scene.py 는 셤)
# 전제: sil/t3_warehouse_map/map/에서 warehouse_layout_v5_5_final.py 실행 완료(npy 존재).
#
# pxr·omni 는 SimulationApp 생성 "후"에만 import 가능(6.0.1: 사전 import 시 ModuleNotFoundError·경고)
# → pxr 의존 모듈은 main() 안에서 로드한다. 이 모듈 자체는 numpy 만으로 import 된다.
import os
import time

import numpy as np

from .config import MAP_DIR, OUT_DIR, USD_PATH


def main():
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True})

    from isaacsim.core.experimental.utils.app import enable_extension  # noqa: F401  (원본 import 시점 유지, vnv 사용)
    import omni.physx  # noqa: F401
    import omni.timeline  # noqa: F401
    import omni.usd
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics

    from . import building, offices, racks, shots, stations, vnv   # pxr 이후 import

    os.makedirs(OUT_DIR, exist_ok=True)
    grid = np.load(os.path.join(MAP_DIR, "occupancy_grid.npy"))
    rack_units = np.load(os.path.join(MAP_DIR, "rack_units.npy"))
    ROWS, COLS = grid.shape
    print(f"[in] grid {ROWS}x{COLS}, rack_units {len(rack_units)}세그(세로형)")

    t0 = time.time()
    # 저작은 렌더러와 분리된 순수 USD 스테이지에서 — 참조를 하나씩 붙이며 app.update()를
    # 돌리면 로딩 중 hydra 경로에서 간헐 세그폴트(3회 재현). 완성 파일을 한 번에 열면 안정.
    usd_path = USD_PATH
    if os.path.exists(usd_path):
        os.remove(usd_path)
    stage = Usd.Stage.CreateNew(usd_path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.Scene.Define(stage, Sdf.Path("/physicsScene"))
    UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))

    building.build_ground_and_lights(stage, ROWS, COLS)      # [0]  바닥 재질 + 태양 + 현수등
    building.build_walls(stage, grid)                        # [1]  벽·기둥·사무실 벽 + 릿지 H형강
    building.build_steel_envelope(stage)                     # [1c] 윈드 컬럼·거트
    building.build_gable_roof(stage)                         # [1d] 박공지붕 + 상부 철골
    offices.build_office_interior(stage)                     # [1b] 사무실 인테리어
    stations.build_conveyors_and_tables(stage, grid)         # [2]  컨베이어·패킹 테이블
    racks.build_pallet_blocks(stage, grid)                   # [2b] 바닥 파렛트 블록 + 지게차
    racks.build_racks(stage, rack_units)                     # [3]  렉 조립
    racks.build_rack_cargo(stage, rack_units)                # [3b] 렉 화물
    building.build_floor_markings(stage, grid, rack_units)   # [3c] 바닥 마킹
    stations_pts = stations.load_stations()
    stations.build_chargers(stage, stations_pts)             # [3d] 충전 스테이션
    building.build_doors(stage)                              # [3e] 도어 드레싱
    stations.build_anchors(stage, stations_pts)              # [3f] 스테이션 앵커

    # 4) 저작 저장 → 완성 파일을 컨텍스트로 오픈 (참조 일괄 로딩)
    stage.GetRootLayer().Save()
    del stage
    print(f"[4] 저장: {usd_path} ({os.path.getsize(usd_path)//1024}KB) · 저작 {time.time()-t0:.0f}s", flush=True)

    ctx = omni.usd.get_context()
    ctx.open_stage(usd_path)
    while ctx.get_stage_loading_status()[2] > 0:
        app.update()
    for _ in range(30):
        app.update()
    stage = ctx.get_stage()

    # 렉 메시 콜라이더 보장 (에셋에 이미 있으면 0건)
    n_col = 0
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Mesh) and str(prim.GetPath()).startswith("/World/racks"):
            if not prim.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI.Apply(prim)
                n_col += 1
    print(f"[4b] 오픈·로딩 완료, 렉 메시 콜라이더 적용 {n_col}건 · {time.time()-t0:.0f}s", flush=True)

    vnv.run_vnv(app, ctx, grid, rack_units)                  # [5]  omap V&V
    shots.take_shots(app, stage, ROWS, COLS)                 # [6]  검수 스크린샷

    app.close()
    print(f"[done] 총 {time.time()-t0:.0f}s")


if __name__ == "__main__":
    main()
