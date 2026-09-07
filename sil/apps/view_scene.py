"""T3 씬 뷰어 — warehouse_scene.usd 를 WebRTC + TCP MJPEG(8211)로 띄운다 (관전 전용).

실행: cd ~/isaacsim && ./python.sh <repo>/sil/apps/view_scene.py
전제: build_scene.py 실행 완료(warehouse_scene.usd 존재).
"""
import _bootstrap  # noqa: F401

from sil_isaac import app as sil_app
from sil_isaac.scene.warehouse import WAREHOUSE_USD

VIEW_CAM = "/World/view_cam"
VIEW_POS, VIEW_ROT, VIEW_FOCAL = (20, 20, 22), (62, 0, -38), 18.0   # 남서측 조감 — 검수 스크린샷과 동일 구도
SETTLE_FRAMES = 30


def main() -> None:
    # 720p — tailscale DERP 릴레이 경유(RTT 170~500 ms)에서 1080p 는 대역폭 부족으로 화면 멈춤
    app = sil_app.launch(headless=True, ros2=False)

    import omni.usd
    from omni.kit.viewport.utility import get_active_viewport
    from pxr import Gf, UsdGeom
    from sil_isaac.viewer.http_stream import HttpViewer

    ctx = omni.usd.get_context()
    ctx.open_stage(WAREHOUSE_USD)
    while ctx.get_stage_loading_status()[2] > 0:
        app.update()
    for _ in range(SETTLE_FRAMES):
        app.update()
    cam = UsdGeom.Camera.Define(ctx.get_stage(), VIEW_CAM)
    cam.CreateFocalLengthAttr(VIEW_FOCAL)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.1, 2000))
    xf = UsdGeom.Xformable(cam.GetPrim())
    xf.AddTranslateOp().Set(Gf.Vec3d(*VIEW_POS))
    xf.AddRotateXYZOp().Set(Gf.Vec3f(*VIEW_ROT))
    get_active_viewport().camera_path = VIEW_CAM
    viewer = HttpViewer(port=app.mjpeg_port)
    print("[view_scene] ready — WebRTC 접속 대기 (종료: 프로세스 킬)", flush=True)
    while app.is_running():
        app.update()
        viewer.tick()
    app.close()


if __name__ == "__main__":
    main()
