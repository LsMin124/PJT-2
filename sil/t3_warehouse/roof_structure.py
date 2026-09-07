"""호환 셤 — 본체는 sil/scene_builder/roof.py (치수 상수는 scene_builder/config.py)."""
import os, sys  # noqa: E401
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from scene_builder.roof import *  # noqa: E402,F401,F403
