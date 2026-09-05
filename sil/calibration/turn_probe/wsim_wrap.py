"""warehouse_sim.py(T3)를 무수정으로 띄우고, ready 직후 iw_hub 각 링크의 USD 기본 오프셋(루트 기준)과
차체 bbox를 출력하는 래퍼. Isaac python으로 실행:  cd ~/isaacsim && ./python.sh <이 파일>
목적: FMS 헤딩 모델 상수(앞축 종방향 위치 = 회전 중심 vs 차체 전후 돌출)를 USD에서 읽는다."""
import os
import runpy
import sys

T3 = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "t3_warehouse"))
sys.path.insert(0, T3)
sys.argv = [os.path.join(T3, "warehouse_sim.py")]

import http_stream  # noqa: E402  (warehouse_sim이 ready 직후 HttpViewer를 만든다 → 그 시점에 훅)

_Orig = http_stream.HttpViewer


def _print_offsets():
    import omni.usd
    from pxr import Usd, UsdGeom

    st = omni.usd.get_context().get_stage()
    root = st.GetPrimAtPath("/World/iw_hub")
    inv = UsdGeom.Xformable(root).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).GetInverse()
    for rel in ("chassis", "left_wheel", "right_wheel", "left_swivel", "right_swivel",
                "left_caster", "right_caster", "lift", "chassis/hull", "chassis/lidar",
                "chassis/glider_0", "chassis/glider_1"):
        p = st.GetPrimAtPath(f"/World/iw_hub/{rel}")
        if not p.IsValid():
            continue
        xw = UsdGeom.Xformable(p).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        loc = (xw * inv).ExtractTranslation()
        print(f"[wrap] offset {rel:16s} root-frame xyz=({loc[0]:+.4f},{loc[1]:+.4f},{loc[2]:+.4f})", flush=True)
    bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"], useExtentsHint=False)
    for rel in ("", "chassis", "lift", "left_wheel"):
        p = st.GetPrimAtPath("/World/iw_hub" + ("/" + rel if rel else ""))
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
        except Exception as e:  # 측정 보조 출력이 본 실행을 막으면 안 됨
            print(f"[wrap] offset error: {type(e).__name__}: {e}", flush=True)


http_stream.HttpViewer = _Viewer
runpy.run_path(os.path.join(T3, "warehouse_sim.py"), run_name="__main__")
