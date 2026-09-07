"""검수 스크린샷 — /World/cam 카메라로 탑뷰·조감·통로·사무실 등 12장 캡처 (omni 뷰포트는 함수 안에서 import)."""
import os

from pxr import Gf, UsdGeom

from .config import CELL, OUT_DIR


def shoot(app, vp, cam, pos, rot, fname, focal=18.0):
    """카메라 위치·회전·초점 설정 → 렌더 워밍업 → 뷰포트 캡처 (비동기 저장 폴링)."""
    from omni.kit.viewport.utility import capture_viewport_to_file

    cam.CreateFocalLengthAttr(focal)
    xf = UsdGeom.Xformable(cam.GetPrim())
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    xf.AddRotateXYZOp().Set(Gf.Vec3f(*rot))
    for _ in range(45):
        app.update()
    path = os.path.join(OUT_DIR, fname)
    capture_viewport_to_file(vp, path)
    for _ in range(200):                               # 캡처는 비동기 저장 — 파일 생성까지 폴링
        app.update()
        if os.path.exists(path):
            break
    print(f"[6] {fname} {'OK' if os.path.exists(path) else '캡처 실패'}")


def take_shots(app, stage, rows, cols):
    """[6] 스크린샷 — 탑뷰 + 퍼스펙티브 + 통로 뷰(세로형 → 북향) 등 12장."""
    from omni.kit.viewport.utility import get_active_viewport

    cam = UsdGeom.Camera.Define(stage, "/World/cam")
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.1, 2000))
    vp = get_active_viewport()
    vp.camera_path = "/World/cam"

    shoot(app, vp, cam, (cols * CELL / 2, rows * CELL / 2, 120), (0, 0, 0), "scene_top.png", focal=24.0)
    shoot(app, vp, cam, (34, 40, 4.5), (75, 0, -35), "scene_persp.png")     # 남서측 실내 조감 (작업 라인+렉)
    shoot(app, vp, cam, (59.1, 45.5, 1.2), (87, 0, 0), "scene_aisle.png", focal=14.0)   # 통로5 북향
    shoot(app, vp, cam, (28.3, 30.2, 1.7), (75, 0, 100), "scene_office.png", focal=16.0)  # 남서 사무실 내부
    shoot(app, vp, cam, (58, 34.6, 2.4), (76, 0, 180), "scene_pallets.png", focal=17.0)   # 남측 파렛트 블록
    shoot(app, vp, cam, (-8, 2, 17), (75, 0, -52), "scene_exterior.png", focal=16.0)      # 남서측 외부 조감
    shoot(app, vp, cam, (-26, 57.5, 8), (85, 0, -90), "scene_gable.png", focal=16.0)      # 서측 박공 정면
    shoot(app, vp, cam, (84, 39.8, 2.0), (78, 0, 180), "scene_station.png", focal=15.0)   # 패킹 스테이션 열 남향
    shoot(app, vp, cam, (17.5, 60.5, 1.6), (80, 0, 170), "scene_forklift.png", focal=16.0)  # 인바운드 밴드·지게차
    shoot(app, vp, cam, (59.1, 42, 1.5), (125, 0, 0), "scene_truss.png", focal=14.0)      # 실내 트러스 앙시(상향각)
    shoot(app, vp, cam, (104.6, 57.5, 1.6), (78, 0, -90), "scene_charger.png", focal=16.0)  # 충전 스테이션 열(동향)
    shoot(app, vp, cam, (22.0, 48.0, 2.6), (82, 0, 131), "scene_door.png", focal=15.0)      # 서벽 남측 도어(북동→사선)
    # ※ (21.5,35)는 남서 사무실 "내부" — 도어 앞은 사무실(y<37.5)·컨베이어(y38.9) 사이가 좁아
    #   북동쪽 개활지에서 잡아야 한다 (2차 빌드 실측)
