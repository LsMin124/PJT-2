"""호환 셔ム — 구현은 sil/sil_ros/nodes/measure_accel.py 로 이동. 실행법 동일: python3 measure_accel.py --duration 12 --label step08"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from sil_ros.nodes.measure_accel import main  # noqa: E402

if __name__ == "__main__":
    main()
