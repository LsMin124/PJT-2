"""호환 셔ム — 구현은 sil/sil_ros/nodes/spin_probe.py 로 이동. 실행법 그대로: python3 spin_probe.py --label run1 (run_spin.sh 동일)"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
from sil_ros.nodes.spin_probe import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
