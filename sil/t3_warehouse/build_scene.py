"""호환 셤 — 본체는 sil/scene_builder 패키지, 엔트리는 sil/apps/build_scene.py (옛 경로 실행도 동작)."""
import os, sys  # noqa: E401
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from scene_builder.build import main  # noqa: E402
main()
