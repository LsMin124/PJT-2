"""호환 셔ム — 구현은 sil/sil_ros/tools/export_replay.py 로 이동. 실행법 그대로: python export_replay.py → ../../isaac_export_t3/"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from sil_ros.tools.export_replay import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
