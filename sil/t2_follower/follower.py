"""T2 추종기 — 노드 시퀀스 순회: Pure Pursuit + 가속 램프 + 안전 필드 정지.

확정 아키텍처의 로봇 레벨 실행기(자체 추종기, ROS2 노드). 경로망(route.yaml)의
투어를 순회하며 /cmd_vel을 내고, 엣지별 통과시간(시뮬 시간)을 CSV로 남긴다(DES 환류).

- 가속 램프: T1 실측(물리 즉답형)에 따라 명령 수준에서 가감속 제한
- 안전 필드: 사각형 보호 필드(전방 진행 대역 |측면|<0.45m) — 감속(1.5m)→정지(1.2m)→해제(1.4m, 히스테리시스)
  ※ 라이다 range_min이 1.0m라 그 미만은 보이지 않음 — 정지 임계는 그 위여야 한다

실행(시스템 Humble python):
  source /opt/ros/humble/setup.bash
  python3 follower.py --route route.yaml --laps 4
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


def ang_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


class Follower(Node):
    def __init__(self, route_path, laps, label="run"):
        super().__init__("t2_follower")
        cfg = yaml.safe_load(open(route_path))
        self.nodes = {k: tuple(v) for k, v in cfg["nodes"].items()}
        self.tour = cfg["tour"]
        self.laps = laps
        self.plan = None                              # 첫 pose에서 최근접 노드 기준으로 확정
        self.idx = 0
        self.pose = None                              # (x, y, yaw, t_sim)
        self.front = float("inf")
        self.stopped = False
        self.v_cmd = 0.0
        self.w_cmd = 0.0
        self.t_prev = None
        self.arrivals = []                            # (node_id, t_sim)
        self.slow_stops = 0
        self.stop_t = None
        self.done = False
        self.label = label

        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self.cb_odom, 50)
        self.create_subscription(LaserScan, "/scan", self.cb_scan, 10)
        self.create_timer(0.05, self.control)         # 20Hz(벽시계)
        self.get_logger().info(f"투어 {len(self.tour)}노드/바퀴 · {laps}바퀴 — 시작 대기")

    def cb_odom(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y, yaw, t)

    def cb_scan(self, m):
        # 사각형 보호 필드 — 원뿔 섹터는 측면 물체(통로 옆 박스)에 오탐. 산업 필드처럼
        # 로봇 전방 진행 대역(|측면| < LAT_HALF)에 드는 점만 본다.
        best = float("inf")
        ang = m.angle_min
        for r in m.ranges:
            if m.range_min < r < m.range_max and abs(ang) < 1.4:
                fx = r * math.cos(ang)
                if 0.0 < fx < best and abs(r * math.sin(ang)) < LAT_HALF:
                    best = fx
            ang += m.angle_increment
        self.front = best

    def control(self):
        if self.done or self.pose is None:
            return
        x, y, yaw, t = self.pose
        if self.plan is None:
            # 접근 구간이 장애물 모서리를 스치지 않도록, 현재 위치 최근접 노드부터 투어 시작
            near = min(range(len(self.tour)), key=lambda i: math.hypot(self.nodes[self.tour[i]][0] - x, self.nodes[self.tour[i]][1] - y))
            rotated = self.tour[near:] + self.tour[:near]
            self.plan = rotated * self.laps + [rotated[0]]
            self.get_logger().info(f"투어 시작 노드 = {rotated[0]} (최근접)")
        if self.t_prev is None:
            self.t_prev = t
            return
        dt = t - self.t_prev
        if dt <= 0:
            return
        dt = min(dt, 0.2)
        self.t_prev = t

        tgt = self.nodes[self.plan[self.idx]]
        dist = math.hypot(tgt[0] - x, tgt[1] - y)
        if dist < ARRIVE:
            self.arrivals.append((self.plan[self.idx], t))
            self.get_logger().info(f"도착 {self.plan[self.idx]} (t={t:.1f}s)")
            self.idx += 1
            if self.idx >= len(self.plan):
                self.finish()
                return
            tgt = self.nodes[self.plan[self.idx]]
            dist = math.hypot(tgt[0] - x, tgt[1] - y)

        alpha = ang_diff(math.atan2(tgt[1] - y, tgt[0] - x), yaw)

        # 목표 근접 시 감속 + 보호 필드 축소 (속도 연동 필드 — 코너 노드가 벽에
        # 가까울 때 필드가 노드 도달을 막는 기하 충돌 방지. 하한 1.05m는 라이다
        # range_min 1.0m 바로 위)
        goal_near = dist < 1.2
        aligning = abs(alpha) > ALIGN_ERR
        # 근접·회전 중에는 축소 필드(전진하지 않으므로 안전) — 하한은 라이다 range_min 위
        stop_d = 1.05 if (goal_near or aligning) else STOP_DIST
        clear_d = stop_d + 0.2

        # 안전 필드 (히스테리시스)
        if self.stopped:
            if self.front > clear_d:
                self.stopped = False
                self.get_logger().info(f"재개 (전방 {self.front:.2f}m)")
            elif self.stop_t is not None and t - self.stop_t > 20.0:
                self.get_logger().warn("경로 막힘 20s+ — 아키텍처상 재라우팅은 중앙(FMS) 몫: 보고 대상")
                self.stop_t = t  # 반복 경고 간격 유지
        elif self.front < stop_d:
            self.stopped = True
            self.stop_t = t
            self.slow_stops += 1
            self.get_logger().info(f"안전 정지 (전방 {self.front:.2f}m)")

        if self.stopped:
            # 산업 관행: 안전 정지는 선속만 차단 — 목표 방향 제자리 회전은 허용(데드락 방지)
            v_des = 0.0
            w_des = math.copysign(0.5, alpha) if abs(alpha) > 0.15 else 0.0
        elif abs(alpha) > ALIGN_ERR:
            v_des = 0.0
            w_des = math.copysign(min(W_MAX, 0.6), alpha)
        else:
            v_des = CRUISE if abs(alpha) < 0.3 else V_TURN
            if goal_near:
                v_des = min(v_des, 0.2)
            if self.front < SLOW_DIST:
                v_des = min(v_des, V_TURN)
            w_des = max(-W_MAX, min(W_MAX, K_HEAD * alpha))

        # 가속 램프 (시뮬 시간 기준)
        self.v_cmd += max(-A_LIN * dt, min(A_LIN * dt, v_des - self.v_cmd))
        self.w_cmd += max(-A_ANG * dt, min(A_ANG * dt, w_des - self.w_cmd))

        msg = Twist()
        msg.linear.x = self.v_cmd
        msg.angular.z = self.w_cmd
        self.pub.publish(msg)

    def finish(self):
        self.done = True
        self.pub.publish(Twist())
        out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")
        os.makedirs(out_dir, exist_ok=True)
        path = os.path.join(out_dir, f"edge_times_{self.label}.csv")
        with open(path, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["seq", "edge", "t_start", "t_end", "duration"])
            for i in range(1, len(self.arrivals)):
                a, b = self.arrivals[i - 1], self.arrivals[i]
                w.writerow([i, f"{a[0]}→{b[0]}", f"{a[1]:.3f}", f"{b[1]:.3f}", f"{b[1] - a[1]:.3f}"])
        durs = [self.arrivals[i][1] - self.arrivals[i - 1][1] for i in range(1, len(self.arrivals))]
        self.get_logger().info(
            f"완주 — 엣지 {len(durs)}건, 평균 {sum(durs)/len(durs):.2f}s, 안전 정지 {self.slow_stops}회 → {path}"
        )
        raise SystemExit(0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--route", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "route.yaml"))
    ap.add_argument("--laps", type=int, default=4)
    ap.add_argument("--label", default="run", help="출력 CSV 라벨")
    args = ap.parse_args()
    rclpy.init()
    node = Follower(args.route, args.laps, args.label)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
