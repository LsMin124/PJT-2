"""호환 셔임 — 구현은 sil_isaac.viewer.http_stream 으로 이동."""
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")))
from sil_isaac.viewer.http_stream import *  # noqa: E402,F401,F403
from sil_isaac.viewer.http_stream import HttpViewer  # noqa: E402,F401
