"""T3 씬 빌더 엔트리 — cd ~/isaacsim && ./python.sh <repo>/sil/apps/build_scene.py (본체: sil/scene_builder 패키지)."""
import _bootstrap  # noqa: F401

from scene_builder.build import main

if __name__ == "__main__":
    main()
