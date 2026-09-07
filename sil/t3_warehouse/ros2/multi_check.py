"""호환 셔ム — 구현은 sil/sil_ros/nodes/multi_check.py 로 이동. 실행법 그대로: python3 multi_check.py --n 3 --drive amr02"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
from sil_ros.nodes.multi_check import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
