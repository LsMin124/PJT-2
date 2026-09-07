"""패트롤 드라이버 — 스테이션 그랜드 투어를 무한 순회하며 AMCL 추적을 시연.

warehouse_sim + loc.launch 가 떠 있는 상태에서:
  source /opt/ros/humble/setup.bash && python3 patrol.py          # sil/t3_warehouse/ros2/ 의 호환 셔ム
  python3 -m sil_ros.nodes.patrol [--map-dir DIR]                   # sil/ 에서 직접 (환경변수 SIL_MAP_DIR 도 가능)
경로는 obstacle_mask(0.8m 팽창) 기준 A* — export_replay.py와 동일 플래너(sil_ros.gridmap.GridMap).
제어는 정답 오돔 기준 순수 추종(P) — localization은 "관측 대상"이므로 제어에
쓰지 않는다. 5초마다 odom vs amcl 오차를 출력.
"""

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from sil_ros.geometry import dist, heading_to, pose2d_of, wrap
from sil_ros.gridmap import DEFAULT_MAP_DIR, ENV_MAP_DIR, GridMap
from sil_ros.twist_ramp import clamp

# 그랜드 투어 — 남측 코리도→패킹 라인→충전 구역→북측 코리도→반품→서측 (무한 반복)
TOUR = [("handoff", 0), ("induction", 0), ("packing", 2), ("handoff", 2),
        ("charger", 0), ("handoff", 3), ("returns", 0), ("handoff", 1)]

V_MAX = 0.6
W_MAX = 0.9
WP_TOL = 0.45               # m — 경유점 도달 반경
K_HEAD = 1.8                # 방위 P 이득
TURN_IN_PLACE_ERR = 0.7     # rad — 방위 오차가 이보다 크면 제자리 회전(선속 0)
V_MIN_FRAC = 0.25           # 방위 오차에 따른 선속 감쇠 하한 (V_MAX 비율)
TICK_S = 0.05               # 제어 주기 (벽시계)
LOG_PERIOD_S = 5.0          # odom vs amcl 오차 출력 주기
AMCL_QOS_DEPTH = 5
ODOM_QOS_DEPTH = 10
CMD_QOS_DEPTH = 10


def heading_cmd(herr):
    """방위 오차(rad) → (v, w): P 조향, 오차가 크면 제자리 회전, 아니면 cos 감쇠 전진."""
    w = clamp(K_HEAD * herr, -W_MAX, W_MAX)
    v = 0.0 if abs(herr) > TURN_IN_PLACE_ERR else V_MAX * max(V_MIN_FRAC, math.cos(herr))
    return v, w


class Patrol(Node):
    def __init__(self, gridmap):
        super().__init__("t3_patrol")
        self.gm = gridmap
        self.odom = None
        self.amcl = None
        qos_tl = QoSProfile(depth=AMCL_QOS_DEPTH)
        qos_tl.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(Odometry, "odom", self.on_odom, ODOM_QOS_DEPTH)
        self.create_subscription(PoseWithCovarianceStamped, "amcl_pose",
                                 self.on_amcl, qos_tl)
        self.cmd = self.create_publisher(Twist, "cmd_vel", CMD_QOS_DEPTH)
        self.path = []
        self.idx = 0
        self.tour_i = 0
        self.last_log = 0.0
        self.create_timer(TICK_S, self.tick)

    def on_odom(self, m):
        self.odom = m

    def on_amcl(self, m):
        self.amcl = m

    def pose(self):
        return pose2d_of(self.odom.pose.pose)

    def plan_next(self):
        x, y, _ = self.pose()
        typ, si = TOUR[self.tour_i % len(TOUR)]
        goal = self.gm.station(typ, si)
        raw = self.gm.astar((x, y), goal)
        if raw is None:
            self.get_logger().warn(f"경로 실패 → {typ}_{si} 건너뜀")
            self.tour_i += 1
            return
        self.path = self.gm.densify(self.gm.simplify(raw))
        self.idx = 1 if len(self.path) > 1 else 0
        print(f"[patrol] → {typ}_{si} {goal} (경유 {len(self.path)}점)", flush=True)
        self.tour_i += 1

    def tick(self):
        if self.odom is None:
            return
        if not self.path or self.idx >= len(self.path):
            self.plan_next()
            return
        x, y, yaw = self.pose()
        tx, ty = self.path[self.idx]
        if dist((x, y), (tx, ty)) < WP_TOL:
            self.idx += 1
            return
        herr = wrap(heading_to((x, y), (tx, ty)) - yaw)
        v, w = heading_cmd(herr)
        tw = Twist()
        tw.angular.z = w
        tw.linear.x = v
        self.cmd.publish(tw)
        self._log_error(x, y)

    def _log_error(self, x, y):
        """LOG_PERIOD_S 마다 odom vs amcl 오차 출력 (amcl 수신 전에는 생략)."""
        now = time.monotonic()
        if now - self.last_log > LOG_PERIOD_S and self.amcl is not None:
            a = self.amcl.pose.pose.position
            err = math.hypot(a.x - x, a.y - y)
            print(f"[patrol] odom=({x:.2f},{y:.2f}) amcl=({a.x:.2f},{a.y:.2f}) "
                  f"err={err:.3f} m wp {self.idx}/{len(self.path)}", flush=True)
            self.last_log = now


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description="T3 패트롤 — 스테이션 그랜드 투어 + AMCL 오차 로그")
    ap.add_argument("--map-dir", default=None,
                    help=f"맵 디렉토리 (기본 {DEFAULT_MAP_DIR}, 환경변수 {ENV_MAP_DIR})")
    args, _ = ap.parse_known_args(argv)      # 원본은 인자를 받지 않았으므로 낯선 인자는 무시
    return args


def main(argv=None):
    args = parse_args(argv)
    gm = GridMap.load(args.map_dir)
    rclpy.init()
    n = Patrol(gm)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.cmd.publish(Twist())


if __name__ == "__main__":
    main()
