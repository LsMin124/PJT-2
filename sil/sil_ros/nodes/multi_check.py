"""다중 로봇 개통 검증 — warehouse_sim(WSIM_N=N)이 ready 상태일 때 Humble python3로 실행.

  source /opt/ros/humble/setup.bash
  python3 multi_check.py --n 3 --drive amr02        # 네임스페이스 모드: /amr01~03/{odom,scan,cmd_vel} (ros2/ 셔ム)
  python3 multi_check.py --n 1 --flat               # 단일 평면 토픽(/odom /scan /cmd_vel) 회귀 확인
  python3 -m sil_ros.nodes.multi_check --n 3        # sil/ 에서 직접

1) 3 s 동안 로봇별 odom·scan, 공용 /clock 발행률과 /tf 프레임 쌍을 센다.
2) --drive 로봇에만 v=0.3 m/s를 3 s 주고 1 s 정지 → 그 로봇만 움직이고(≥ 0.5 m) 나머지는 정지(≤ 2 cm)인지 본다.
결과는 JSON 한 줄("MULTI_CHECK {json}"), 통과 시 exit 0."""
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
SCAN_FINITE_MIN = 100           # 유효(finite · range 내) 빔 수 하한 (초과 조건)
DEFAULT_RATE_S = 3.0            # 발행률 측정 시간
SPIN_TIMEOUT_S = 0.05
FLAT = "flat"                   # 평면 토픽 모드의 표시 이름
QOS_SENSOR = 50
QOS_TF = 100
QOS_CMD = 10


# ── 이름 · 토픽 · 프레임 규칙 ──
def robot_names(n, flat):
    return [""] if flat else [f"amr{i + 1:02d}" for i in range(n)]


def label(n):
    return n or FLAT


def topic_prefix(n):
    return f"/{n}/" if n else "/"


def frame_prefix(n):
    return f"{n}/" if n else ""


def expected_tf_frames(names):
    return {f"{frame_prefix(n)}odom->{frame_prefix(n)}base_link" for n in names} | \
           {f"{frame_prefix(n)}base_link->{frame_prefix(n)}laser" for n in names}


# ── 판정 ──
def rates_ok(hz, clock_hz):
    return all(h["odom"] >= RATE_MIN_HZ and h["scan"] >= RATE_MIN_HZ and h["scan_finite"] > SCAN_FINITE_MIN
               for h in hz.values()) and clock_hz >= RATE_MIN_HZ


def move_ok(moved, target_key, missing):
    return (not missing) and moved.get(target_key, 0.0) >= MOVE_MIN_M \
        and all(d <= STILL_MAX_M for k, d in moved.items() if k != target_key)


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
            pre = topic_prefix(n)
            self.create_subscription(Odometry, pre + "odom", lambda m, n=n: self._odom(n, m), QOS_SENSOR)
            self.create_subscription(LaserScan, pre + "scan", lambda m, n=n: self._scan(n, m), QOS_SENSOR)
            self.pubs[n] = self.create_publisher(Twist, pre + "cmd_vel", QOS_CMD)
        self.create_subscription(Clock, "/clock", self._clock, QOS_SENSOR)
        self.create_subscription(TFMessage, "/tf", self._tf, QOS_TF)

    def _odom(self, n, m):
        self.n_odom[n] += 1
        self.pose[n] = (m.pose.pose.position.x, m.pose.pose.position.y)

    def _scan(self, n, m):
        self.n_scan[n] += 1
        self.finite[n] = sum(1 for r in m.ranges if math.isfinite(r) and m.range_min <= r <= m.range_max)

    def _clock(self, _m):
        self.n_clock += 1

    def _tf(self, m):
        for t in m.transforms:
            self.frames.add(f"{t.header.frame_id}->{t.child_frame_id}")

    def spin_for(self, s):
        end = time.monotonic() + s
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=SPIN_TIMEOUT_S)

    def drive(self, n, v, s):
        end = time.monotonic() + s
        msg = Twist()
        msg.linear.x = float(v)
        while time.monotonic() < end:
            self.pubs[n].publish(msg)
            rclpy.spin_once(self, timeout_sec=SPIN_TIMEOUT_S)


def measure_rates(node, names, rate_s):
    """rate_s 동안 수신 → 로봇별 {odom, scan, scan_finite} 와 /clock Hz."""
    node.spin_for(rate_s)
    hz = {label(n): dict(odom=round(node.n_odom[n] / rate_s, 1), scan=round(node.n_scan[n] / rate_s, 1),
                         scan_finite=node.finite[n]) for n in names}
    return hz, round(node.n_clock / rate_s, 1)


def drive_and_measure(node, names, target, before):
    """target 만 DRIVE_S 구동 + REST_S 정지 → 로봇별 이동 거리(m)."""
    node.drive(target, DRIVE_V, DRIVE_S)
    node.drive(target, 0.0, REST_S)
    return {label(n): round(math.hypot(node.pose[n][0] - before[n][0], node.pose[n][1] - before[n][1]), 3)
            for n in names}


def build_result(node, names, target, clock_hz, hz, moved, missing):
    exp_frames = expected_tf_frames(names)
    ok_tf = exp_frames <= node.frames
    return dict(names=[label(n) for n in names], target=label(target), clock_hz=clock_hz, hz=hz, moved_m=moved,
                tf_ok=ok_tf, tf_missing=sorted(exp_frames - node.frames), missing_odom=missing,
                PASS=bool(rates_ok(hz, clock_hz) and move_ok(moved, label(target), missing) and ok_tf))


def parse_args(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=1)
    ap.add_argument("--flat", action="store_true", help="단일 로봇 평면 토픽(/odom …)")
    ap.add_argument("--drive", default="", help="구동할 로봇 이름(네임스페이스 모드). 기본 마지막 로봇")
    ap.add_argument("--rate_s", type=float, default=DEFAULT_RATE_S)
    return ap.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    names = robot_names(args.n, args.flat)
    target = "" if args.flat else (args.drive or names[-1])

    rclpy.init()
    node = Check(names)
    hz, clock_hz = measure_rates(node, names, args.rate_s)
    before = dict(node.pose)
    missing = [n for n in names if before[n] is None]
    moved = {} if missing else drive_and_measure(node, names, target, before)
    res = build_result(node, names, target, clock_hz, hz, moved, missing)
    print("MULTI_CHECK " + json.dumps(res, ensure_ascii=False), flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if res["PASS"] else 1


if __name__ == "__main__":
    sys.exit(main())
