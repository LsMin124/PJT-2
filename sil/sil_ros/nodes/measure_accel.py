"""/odom 기록기 — T1의 DES 환류(최고 속도·가감속 곡선) 실측용.

시스템 Humble python으로 실행한다 (Isaac 아님):
  source /opt/ros/humble/setup.bash
  python3 measure_accel.py --duration 12 --label step08      # sil/t1_teleop/ 의 호환 셔ム
  python3 -m sil_ros.nodes.measure_accel --duration 12        # sil/ 에서 직접

기록 중 별도 셸에서 스텝 명령을 준다:
  ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.8}}'   # N초 후 Ctrl+C
  ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist '{}'                       # 정지

출력: sil/t1_teleop/out/accel_<label>.csv (t, x, y, speed) + 요약(최고 속도, 0→95% 도달 시간, 평균 가속도).
"""

import argparse
import csv
import math
import os
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

from sil_ros import SIL_ROOT
from sil_ros.geometry import stamp_to_sec

DEFAULT_DURATION_S = 12.0
DEFAULT_OUT_DIR = os.path.join(SIL_ROOT, "t1_teleop", "out")   # 원본 스크립트 옆 out/ 과 같은 위치
ODOM_QOS_DEPTH = 50
SPIN_TIMEOUT_S = 0.2
MIN_SAMPLES = 10            # 요약에 필요한 최소 샘플 수
V_START = 0.02              # m/s — 출발 판정 속도
RISE_FRAC = 0.95            # 최고 속도 대비 도달 비율


class OdomLogger(Node):
    def __init__(self):
        super().__init__("t1_odom_logger")
        self.rows = []
        self.create_subscription(Odometry, "/odom", self.cb, ODOM_QOS_DEPTH)

    def cb(self, msg: Odometry):
        t = stamp_to_sec(msg.header.stamp)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        v = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.rows.append((t, x, y, v))


def summarize(rows):
    if len(rows) < MIN_SAMPLES:
        return "샘플 부족 — /odom 수신 확인 필요"
    vmax = max(r[3] for r in rows)
    t0 = next((r[0] for r in rows if r[3] > V_START), rows[0][0])
    t95 = next((r[0] for r in rows if r[3] >= RISE_FRAC * vmax), None)
    lines = [f"샘플 {len(rows)}건 · 최고 속도 {vmax:.3f} m/s"]
    if t95 is not None and t95 > t0:
        rise = t95 - t0
        lines.append(f"0→95% 도달 {rise:.2f} s · 평균 가속도 {RISE_FRAC * vmax / rise:.3f} m/s²")
    return " · ".join(lines)


def write_csv(path, rows):
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["t", "x", "y", "speed"])
        for r in rows:
            w.writerow([f"{r[0]:.4f}", f"{r[1]:.4f}", f"{r[2]:.4f}", f"{r[3]:.4f}"])


def parse_args(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=DEFAULT_DURATION_S, help="기록 시간(벽시계 초)")
    ap.add_argument("--label", default="run", help="출력 파일 라벨")
    ap.add_argument("--out", default=DEFAULT_OUT_DIR, help="출력 디렉토리")
    return ap.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)

    rclpy.init()
    node = OdomLogger()
    deadline = time.monotonic() + args.duration
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=SPIN_TIMEOUT_S)

    os.makedirs(args.out, exist_ok=True)
    path = os.path.join(args.out, f"accel_{args.label}.csv")
    write_csv(path, node.rows)

    print(f"[measure] {path}")
    print("[measure]", summarize(node.rows))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
