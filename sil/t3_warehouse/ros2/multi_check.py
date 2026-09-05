#!/usr/bin/env python3
"""다중 로봇 개통 검증 — warehouse_sim(WSIM_N=N)이 ready 상태일 때 Humble python3로 실행.

  source /opt/ros/humble/setup.bash
  python3 multi_check.py --n 3 --drive amr02        # 네임스페이스 모드: /amr01~03/{odom,scan,cmd_vel}
  python3 multi_check.py --n 1 --flat               # 단일 평면 토픽(/odom /scan /cmd_vel) 회귀 확인

1) 3 s 동안 로봇별 odom·scan, 공용 /clock 발행률과 /tf 프레임 쌍을 센다.
2) --drive 로봇에만 v=0.3 m/s를 3 s 주고 1 s 정지 → 그 로봇만 움직이고(≥ 0.5 m) 나머지는 정지(≤ 2 cm)인지 본다.
결과는 JSON 한 줄, 통과 시 exit 0."""
import argparse
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

RATE_MIN_HZ = 30.0
DRIVE_V, DRIVE_S, REST_S = 0.3, 3.0, 1.0
MOVE_MIN_M, STILL_MAX_M = 0.5, 0.02


class Check(Node):
    def __init__(self, names):
        super().__init__("multi_check")
        self.names = names
        self.n_odom = {n: 0 for n in names}
        self.n_scan = {n: 0 for n in names}
        self.finite = {n: 0 for n in names}
        self.pose = {n: None for n in names}
        self.frames = set()
        self.n_clock = 0
        self.pubs = {}
        for n in names:
            pre = f"/{n}/" if n else "/"
            self.create_subscription(Odometry, pre + "odom", lambda m, n=n: self._odom(n, m), 50)
            self.create_subscription(LaserScan, pre + "scan", lambda m, n=n: self._scan(n, m), 50)
            self.pubs[n] = self.create_publisher(Twist, pre + "cmd_vel", 10)
        self.create_subscription(Clock, "/clock", lambda m: setattr(self, "n_clock", self.n_clock + 1), 50)
        self.create_subscription(TFMessage, "/tf", self._tf, 100)

    def _odom(self, n, m):
        self.n_odom[n] += 1
        self.pose[n] = (m.pose.pose.position.x, m.pose.pose.position.y)

    def _scan(self, n, m):
        self.n_scan[n] += 1
        self.finite[n] = sum(1 for r in m.ranges if math.isfinite(r) and m.range_min <= r <= m.range_max)

    def _tf(self, m):
        for t in m.transforms:
            self.frames.add(f"{t.header.frame_id}->{t.child_frame_id}")

    def spin_for(self, s):
        end = time.monotonic() + s
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def drive(self, n, v, s):
        end = time.monotonic() + s
        msg = Twist()
        msg.linear.x = float(v)
        while time.monotonic() < end:
            self.pubs[n].publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=1)
    ap.add_argument("--flat", action="store_true", help="단일 로봇 평면 토픽(/odom …)")
    ap.add_argument("--drive", default="", help="구동할 로봇 이름(네임스페이스 모드). 기본 마지막 로봇")
    ap.add_argument("--rate_s", type=float, default=3.0)
    args = ap.parse_args()
    names = [""] if args.flat else [f"amr{i + 1:02d}" for i in range(args.n)]
    target = "" if args.flat else (args.drive or names[-1])

    rclpy.init()
    node = Check(names)
    node.spin_for(args.rate_s)
    hz = {n or "flat": dict(odom=round(node.n_odom[n] / args.rate_s, 1), scan=round(node.n_scan[n] / args.rate_s, 1),
                            scan_finite=node.finite[n]) for n in names}
    clock_hz = round(node.n_clock / args.rate_s, 1)
    before = dict(node.pose)
    missing = [n for n in names if before[n] is None]
    moved = {}
    if not missing:
        node.drive(target, DRIVE_V, DRIVE_S)
        node.drive(target, 0.0, REST_S)
        moved = {n or "flat": round(math.hypot(node.pose[n][0] - before[n][0], node.pose[n][1] - before[n][1]), 3)
                 for n in names}
    ok_rates = all(h["odom"] >= RATE_MIN_HZ and h["scan"] >= RATE_MIN_HZ and h["scan_finite"] > 100 for h in hz.values()) \
        and clock_hz >= RATE_MIN_HZ
    ok_move = (not missing) and moved.get(target or "flat", 0.0) >= MOVE_MIN_M \
        and all(d <= STILL_MAX_M for k, d in moved.items() if k != (target or "flat"))
    exp_frames = {f"{n + '/' if n else ''}odom->{n + '/' if n else ''}base_link" for n in names} | \
                 {f"{n + '/' if n else ''}base_link->{n + '/' if n else ''}laser" for n in names}
    ok_tf = exp_frames <= node.frames
    res = dict(names=[n or "flat" for n in names], target=target or "flat", clock_hz=clock_hz, hz=hz, moved_m=moved,
               tf_ok=ok_tf, tf_missing=sorted(exp_frames - node.frames), missing_odom=missing,
               PASS=bool(ok_rates and ok_move and ok_tf))
    print("MULTI_CHECK " + json.dumps(res, ensure_ascii=False), flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if res["PASS"] else 1


if __name__ == "__main__":
    sys.exit(main())
