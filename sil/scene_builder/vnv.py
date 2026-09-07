"""V&V — omap 확장으로 씬→점유맵 역생성 후 원 그리드와 diff (omni 런타임은 함수 안에서 import)."""
import os

import numpy as np
from scipy.ndimage import binary_dilation

from .config import CELL, OUT_DIR
from .gridutil import office_footprint_mask, omap_to_occ, rack_footprint_mask


def run_vnv(app, ctx, grid, rack_units):
    """[5] omap 재생성 → 재현율·렉 풋프린트 점유·오검출 셀 출력, out/omap_occ.npy 저장."""
    # 5) V&V — omap으로 씬→점유맵 재생성, 원 그리드와 diff
    # ※ omap 확장은 반드시 스테이지 로딩 "후"에 활성화 — 선활성화 상태로 대형 씬을
    #   열면 로딩 중 간헐 세그폴트 (기본 비활성 확장)
    from isaacsim.core.experimental.utils.app import enable_extension
    import omni.physx
    import omni.timeline

    ROWS, COLS = grid.shape
    enable_extension("isaacsim.asset.gen.omap")
    for _ in range(5):
        app.update()
    from isaacsim.asset.gen.omap.bindings import _omap

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(10):
        app.update()
    gen = _omap.Generator(omni.physx.get_physx_interface(), ctx.get_stage_id())
    gen.update_settings(CELL, 4, 5, 6)                     # 셀 0.1 / 점유4 자유5 미지6
    gen.set_transform((0, 0, 0), (0, 0, 0.2), (COLS * CELL, ROWS * CELL, 1.2))
    gen.generate2d()
    buf = np.array(gen.get_buffer())
    timeline.stop()
    occ = omap_to_occ(buf, ROWS, COLS)
    if occ is None:
        print(f"[5] omap 버퍼 크기 불일치({buf.size}) — diff 생략")
    else:
        st = np.isin(grid, (1, 5, 6))                      # 벽·컨베이어·작업대·파렛트: 셀 단위 일치 기대
        tol = binary_dilation(occ, iterations=2)
        cover = tol[st].mean() * 100
        rackmask = rack_footprint_mask(rack_units, st.shape)
        rack_hit = occ[rackmask].mean() * 100
        office_mask = office_footprint_mask(st.shape)
        fp = occ & ~binary_dilation(st | rackmask, iterations=3) & ~office_mask
        print(f"[5] V&V — 벽·컨베이어·작업대 재현율 {cover:.1f}% · 렉 풋프린트 내 점유(다리·데크 두께) {rack_hit:.1f}%"
              f" · 풋프린트 밖 오검출(사무실 가구 제외) {fp.sum()}셀")
        np.save(os.path.join(OUT_DIR, "omap_occ.npy"), occ)
