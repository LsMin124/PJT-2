"""T2 추종기 — 노드 시퀀스 순회: Pure Pursuit + 가속 램프 + 안전 필드 정지.

확정 아키텍처의 로봇 레벨 실행기(자체 추종기, ROS2 노드). 경로망(route.yaml)의
투어를 순회하며 /cmd_vel을 내고, 엣지별 통과시간(시뮬 시간)을 CSV로 남긴다(DES 환류).

- 가속 램프: T1 실측(물리 즉답형)에 따라 명령 수준에서 가감속 제한 (sil_ros.twist_ramp.ramp)
- 안전 필드: 사각형 보호 필드(전방 진행 대역 |측면|<0.45m) — 감속(1.5m)→정지(1.2m)→해제(1.4m, 히스테리시스)
  ※ 라이다 range_min이 1.0m라 그 미만은 보이지 않음 — 정지 임계는 그 위여야 한다

실행(시스템 Humble python):
  source /opt/ros/humble/setup.bash
  python3 follower.py --route route.yaml --laps 4       # sil/t2_follower/ 의 호환 셔ム
  python3 -m sil_ros.nodes.follower --laps 4            # sil/ 에서 직접 (route 기본 sil/t2_follower/route.yaml)
전제: mapping_sim.py --manual 이 떠서 /odom·/scan을 내고 /cmd_vel을 받는 상태.
"""

import argparse
import csv
import math
import os

import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from sil_ros import SIL_ROOT
from sil_ros.geometry import ang_diff, dist, heading_to, pose2d_of, stamp_to_sec
from sil_ros.twist_ramp import SafetyField, clamp, front_clearance, ramp

CRUISE = 0.6        # m/s
V_TURN = 0.25       # 정렬 중 상한
W_MAX = 0.9         # rad/s
A_LIN = 0.5         # m/s^2 — 명령 램프 (T1 환류)
A_ANG = 1.2         # rad/s^2
K_HEAD = 2.0        # 방위 P 이득
ARRIVE = 0.35       # m — 노드 도착 반경
ALIGN_ERR = math.radians(60)
LAT_HALF = 0.45          # 안전 필드 측면 반폭 (로봇 반폭 ~0.33 + 여유)
SLOW_DIST, STOP_DIST, CLEAR_DIST = 1.5, 1.2, 1.4
FOV_HALF = 1.4           # rad — 스캔에서 전방 판정에 쓰는 시야 반각
GOAL_NEAR_DIST = 1.2     # m — 목표 근접: 감속 + 보호 필드 축소
NEAR_STOP_DIST = 1.05    # m — 축소 필드 정지 거리 (라이다 range_min 1.0m 바로 위)
CLEAR_MARGIN = 0.2       # m — 해제 거리 = 정지 거리 + 마진 (히스테리시스)
BLOCKED_WARN_S = 20.0    # s — 정지가 이보다 길면 막힘 경고 (반복 간격)
STOP_TURN_W = 0.5        # rad/s — 안전 정지 중 목표 방향 제자리 회전 속도
STOP_TURN_ERR = 0.15     # rad — 그 회전을 시작하는 방위 오차
ALIGN_W = 0.6            # rad/s — 정렬 회전 속도 (W_MAX 로 포화)
CRUISE_ERR = 0.3         # rad — 이 안이면 순항, 밖이면 V_TURN
V_GOAL_NEAR = 0.2        # m/s — 목표 근접 상한
DT_MAX = 0.2             # s — 시뮬 dt 상한 (일시정지 뒤 램프 폭주 방지)
TICK_S = 0.05            # 20Hz(벽시계)
DEFAULT_ROUTE = os.path.join(SIL_ROOT, "t2_follower", "route.yaml")
DEFAULT_OUT_DIR = os.path.join(SIL_ROOT, "t2_follower", "out")   # 원본 스크립트 옆 out/ 과 같은 위치
QOS_CMD, QOS_ODOM, QOS_SCAN = 10, 50, 10


class Follower(Node):
    def __init__(self, route_path, laps, label="run", out_dir=DEFAULT_OUT_DIR):
        super().__init__("t2_follower")
        with open(route_path) as fh:
            cfg = yaml.safe_load(fh)
        self.nodes = {k: tuple(v) for k, v in cfg["nodes"].items()}
        self.tour = cfg["tour"]
        self.laps = laps
        self.label = label
        self.out_dir = out_dir
        self.plan = None                              # 첫 pose에서 최근접 노드 기준으로 확정
        self.idx = 0
        self.pose = None                              # (x, y, yaw, t_sim)
        self.front = float("inf")
        self.field = SafetyField(BLOCKED_WARN_S)      # 정지/해제 히스테리시스 + 정지 횟수
        self.v_cmd = 0.0
        self.w_cmd = 0.0
        self.t_prev = None
        self.arrivals = []                            # (node_id, t_sim)
        self.done = False

        self.pub = self.create_publisher(Twist, "/cmd_vel", QOS_CMD)
        self.create_subscription(Odometry, "/odom", self.cb_odom, QOS_ODOM)
        self.create_subscription(LaserScan, "/scan", self.cb_scan, QOS_SCAN)
        self.create_timer(TICK_S, self.control)
        self.get_logger().info(f"투어 {len(self.tour)}노드/바퀴 · {laps}바퀴 — 시작 대기")

    def cb_odom(self, m):
        p = pose2d_of(m.pose.pose)
        self.pose = (p.x, p.y, p.yaw, stamp_to_sec(m.header.stamp))

    def cb_scan(self, m):
        # 사각형 보호 필드 — 원뿔 섹터는 측면 물체(통로 옆 박스)에 오탐. 산업 필드처럼
        # 로봇 전방 진행 대역(|측면| < LAT_HALF)에 드는 점만 본다.
        self.front = front_clearance(m.ranges, m.angle_min, m.angle_increment,
                                     m.range_min, m.range_max, LAT_HALF, FOV_HALF)

    def _start_plan(self, x, y):
        # 접근 구간이 장애물 모서리를 스치지 않도록, 현재 위치 최근접 노드부터 투어 시작
        near = min(range(len(self.tour)), key=lambda i: dist(self.nodes[self.tour[i]], (x, y)))
        rotated = self.tour[near:] + self.tour[:near]
        self.plan = rotated * self.laps + [rotated[0]]
        self.get_logger().info(f"투어 시작 노드 = {rotated[0]} (최근접)")

    def _step_dt(self, t):
        """시뮬 시간 dt — 첫 호출·시간 역행이면 None(이번 틱 제어 생략), 상한 DT_MAX."""
        if self.t_prev is None:
            self.t_prev = t
            return None
        dt = t - self.t_prev
        if dt <= 0:
            return None
        self.t_prev = t
        return min(dt, DT_MAX)

    def _target(self, x, y, t):
        """현재 목표 노드 좌표 — 도착 반경 안이면 기록 후 다음 노드로(완주 시 finish → SystemExit)."""
        tgt = self.nodes[self.plan[self.idx]]
        if dist((x, y), tgt) < ARRIVE:
            self.arrivals.append((self.plan[self.idx], t))
            self.get_logger().info(f"도착 {self.plan[self.idx]} (t={t:.1f}s)")
            self.idx += 1
            if self.idx >= len(self.plan):
                self.finish()
                return None
            tgt = self.nodes[self.plan[self.idx]]
        return tgt

    def _update_safety(self, stop_d, clear_d, t):
        ev = self.field.step(self.front, stop_d, clear_d, t)
        if ev == SafetyField.RESUME:
            self.get_logger().info(f"재개 (전방 {self.front:.2f}m)")
        elif ev == SafetyField.BLOCKED:
            self.get_logger().warn("경로 막힘 20s+ — 아키텍처상 재라우팅은 중앙(FMS) 몫: 보고 대상")
        elif ev == SafetyField.STOP:
            self.get_logger().info(f"안전 정지 (전방 {self.front:.2f}m)")

    def _desired(self, alpha, goal_near):
        """방위 오차 · 근접 · 전방 거리 · 정지 상태 → 원하는 (v, w)."""
        if self.field.stopped:
            # 산업 관행: 안전 정지는 선속만 차단 — 목표 방향 제자리 회전은 허용(데드락 방지)
            return 0.0, (math.copysign(STOP_TURN_W, alpha) if abs(alpha) > STOP_TURN_ERR else 0.0)
        if abs(alpha) > ALIGN_ERR:
            return 0.0, math.copysign(min(W_MAX, ALIGN_W), alpha)
        v_des = CRUISE if abs(alpha) < CRUISE_ERR else V_TURN
        if goal_near:
            v_des = min(v_des, V_GOAL_NEAR)
        if self.front < SLOW_DIST:
            v_des = min(v_des, V_TURN)
        return v_des, clamp(K_HEAD * alpha, -W_MAX, W_MAX)

    def control(self):
        if self.done or self.pose is None:
            return
        x, y, yaw, t = self.pose
        if self.plan is None:
            self._start_plan(x, y)
        dt = self._step_dt(t)
        if dt is None:
            return
        tgt = self._target(x, y, t)
        if tgt is None:
            return
        alpha = ang_diff(heading_to((x, y), tgt), yaw)

        # 목표 근접 시 감속 + 보호 필드 축소 (속도 연동 필드 — 코너 노드가 벽에
        # 가까울 때 필드가 노드 도달을 막는 기하 충돌 방지. 하한 1.05m는 라이다
        # range_min 1.0m 바로 위)
        goal_near = dist((x, y), tgt) < GOAL_NEAR_DIST
        aligning = abs(alpha) > ALIGN_ERR
        # 근접·회전 중에는 축소 필드(전진하지 않으므로 안전) — 하한은 라이다 range_min 위
        stop_d = NEAR_STOP_DIST if (goal_near or aligning) else STOP_DIST
        self._update_safety(stop_d, stop_d + CLEAR_MARGIN, t)

        v_des, w_des = self._desired(alpha, goal_near)
        # 가속 램프 (시뮬 시간 기준)
        self.v_cmd = ramp(self.v_cmd, v_des, A_LIN * dt)
        self.w_cmd = ramp(self.w_cmd, w_des, A_ANG * dt)

        msg = Twist()
        msg.linear.x = self.v_cmd
        msg.angular.z = self.w_cmd
        self.pub.publish(msg)

    def _write_edge_times(self):
        os.makedirs(self.out_dir, exist_ok=True)
        path = os.path.join(self.out_dir, f"edge_times_{self.label}.csv")
        with open(path, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["seq", "edge", "t_start", "t_end", "duration"])
            for i in range(1, len(self.arrivals)):
                a, b = self.arrivals[i - 1], self.arrivals[i]
                w.writerow([i, f"{a[0]}→{b[0]}", f"{a[1]:.3f}", f"{b[1]:.3f}", f"{b[1] - a[1]:.3f}"])
        return path

    def finish(self):
        self.done = True
        self.pub.publish(Twist())
        path = self._write_edge_times()
        durs = [self.arrivals[i][1] - self.arrivals[i - 1][1] for i in range(1, len(self.arrivals))]
        self.get_logger().info(
            f"완주 — 엣지 {len(durs)}건, 평균 {sum(durs)/len(durs):.2f}s, 안전 정지 {self.field.stops}회 → {path}"
        )
        raise SystemExit(0)


def parse_args(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--route", default=DEFAULT_ROUTE)
    ap.add_argument("--laps", type=int, default=4)
    ap.add_argument("--label", default="run", help="출력 CSV 라벨")
    ap.add_argument("--out", default=DEFAULT_OUT_DIR, help="출력 디렉토리")
    return ap.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    rclpy.init()
    node = Follower(args.route, args.laps, args.label, args.out)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
