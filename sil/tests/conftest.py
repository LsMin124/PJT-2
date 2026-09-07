"""pytest 6 호환 — sil/ 을 import 경로에 올린다 (pyproject 의 pythonpath 는 pytest 7+ 전용)."""
import os
import sys

SIL = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
for p in (SIL, os.path.join(SIL, "t4_agent", "src", "amr_agent")):
    if p not in sys.path:
        sys.path.insert(0, p)
