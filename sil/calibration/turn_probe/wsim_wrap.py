"""warehouse_sim(T3)을 무수정으로 띄우고, ready 직후 iw_hub 각 링크의 USD 기본 오프셋(루트 기준)과 차체 bbox 를 출력.
Isaac python 으로 실행:  cd ~/isaacsim && ./python.sh <이 파일>
목적: FMS 헤딩 모델 상수(앞축 종방향 위치 = 회전 중심 vs 차체 전후 돌출)를 USD 에서 읽는다.
구현: sil_isaac.viewer.http_stream.HttpViewer 를 감싸 ready 시점(뷰어 생성)에 훅을 건다."""
import os
import runpy
import sys

SIL = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
sys.path.insert(0, SIL)
sys.argv = [os.path.join(SIL, "apps", "warehouse_sim.py")]
os.environ.setdefault("WSIM_N", "1")

from sil_isaac.viewer import http_stream  # noqa: E402

_Orig = http_stream.HttpViewer
ROOT = "/World/iw_hub"
LINKS = ("chassis", "left_wheel", "right_wheel", "left_swivel", "right_swivel", "left_caster", "right_caster",
         "lift", "chassis/hull", "chassis/lidar", "chassis/glider_0", "chassis/glider_1")


def _print_offsets():
    import omni.usd
    from pxr import Usd, UsdGeom

    st = omni.usd.get_context().get_stage()
    inv = UsdGeom.Xformable(st.GetPrimAtPath(ROOT)).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).GetInverse()
    for rel in LINKS:
        p = st.GetPrimAtPath(f"{ROOT}/{rel}")
        if not p.IsValid():
            continue
        loc = (UsdGeom.Xformable(p).ComputeLocalToWorldTransform(Usd.TimeCode.Default()) * inv).ExtractTranslation()
        print(f"[wrap] offset {rel:16s} root-frame xyz=({loc[0]:+.4f},{loc[1]:+.4f},{loc[2]:+.4f})", flush=True)
    bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"], useExtentsHint=False)
    for rel in ("", "chassis", "lift", "left_wheel"):
        p = st.GetPrimAtPath(ROOT + ("/" + rel if rel else ""))
        if not p.IsValid():
            continue
        r = bb.ComputeWorldBound(p).ComputeAlignedRange()
        if r.IsEmpty():
            continue
        mn, mx = inv.Transform(r.GetMin()), inv.Transform(r.GetMax())  # 스폰 yaw 0 → 축정렬 유지
        print(f"[wrap] bbox {rel or 'root':10s} root-frame x=[{mn[0]:+.3f},{mx[0]:+.3f}] "
              f"y=[{mn[1]:+.3f},{mx[1]:+.3f}] z=[{mn[2]:+.3f},{mx[2]:+.3f}]", flush=True)


class _Viewer(_Orig):
    def __init__(self, port: int = 8211):
        super().__init__(port=port)
        try:
            _print_offsets()
        except Exception as e:  # noqa: BLE001 — 측정 보조 출력이 본 실행을 막으면 안 됨
            print(f"[wrap] offset error: {type(e).__name__}: {e}", flush=True)


http_stream.HttpViewer = _Viewer
runpy.run_path(os.path.join(SIL, "apps", "warehouse_sim.py"), run_name="__main__")
