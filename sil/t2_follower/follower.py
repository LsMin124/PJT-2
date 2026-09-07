"""호환 셔ム — 구현은 sil/sil_ros/nodes/follower.py 로 이동. 실행법 그대로: python3 follower.py --route route.yaml --laps 4"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from sil_ros.nodes.follower import main  # noqa: E402

if __name__ == "__main__":
    main()
