"""호환 셔ム — 구현은 sil/sil_ros/nodes/patrol.py 로 이동. 실행법 그대로: source /opt/ros/humble/setup.bash && python3 patrol.py"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
from sil_ros.nodes.patrol import main  # noqa: E402

if __name__ == "__main__":
    main()
