"""호환 셔임 — 구현은 sil/apps/teleop_sim.py (sil_isaac 위에서 재작성). 이 경로로 실행해도 같은 동작."""
import os
import runpy

_APP = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "apps", "teleop_sim.py")
runpy.run_path(os.path.abspath(_APP), run_name="__main__")
