"""패트롤 드라이버 — 스테이션 그랜드 투어를 무한 순회하며 AMCL 추적을 시연.

warehouse_sim + loc.launch 가 떠 있는 상태에서:
  source /opt/ros/humble/setup.bash && python3 patrol.py
경로는 obstacle_mask(0.8m 팽창) 기준 A* — export_replay.py와 동일 플래너.
제어는 정답 오돔 기준 순수 추종(P) — localization은 "관측 대상"이므로 제어에
쓰지 않는다. 5초마다 odom vs amcl 오차를 출력.
"""

import heapq
import json
import math
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

HERE = os.path.dirname(os.path.abspath(__file__))
MAP_DIR = os.path.join(HERE, "..", "..", "t3_warehouse_map", "map")
CELL = 0.1

obst = np.load(os.path.join(MAP_DIR, "obstacle_mask.npy")).astype(bool)
stations = json.load(open(os.path.join(MAP_DIR, "stations.json"), encoding="utf-8"))
free = ~obst
ROWS, COLS = obst.shape

# 그랜드 투어 — 남측 코리도→패킹 라인→충전 구역→북측 코리도→반품→서측 (무한 반복)
TOUR = [("handoff", 0), ("induction", 0), ("packing", 2), ("handoff", 2),
        ("charger", 0), ("handoff", 3), ("returns", 0), ("handoff", 1)]

V_MAX = 0.6
W_MAX = 0.9
WP_TOL = 0.45


def cell(p):
    return int(p[1] / CELL), int(p[0] / CELL)


def astar(a, b):
    """팽창 자유 그리드 8방 A* (코너 컷 금지) — export_replay.py와 동일."""
    (r0, c0), (r1, c1) = cell(a), cell(b)
    if not (free[r0, c0] and free[r1, c1]):
        return None
    D = [(-1, 0, 1), (1, 0, 1), (0, -1, 1), (0, 1, 1),
         (-1, -1, 2**0.5), (-1, 1, 2**0.5), (1, -1, 2**0.5), (1, 1, 2**0.5)]
    g = {(r0, c0): 0.0}
    came = {}
    pq = [(0.0, (r0, c0))]
    while pq:
        _, cur = heapq.heappop(pq)
        if cur == (r1, c1):
            path = [cur]
            while cur in came:
                cur = came[cur]
                path.append(cur)
            return [((c + 0.5) * CELL, (r + 0.5) * CELL) for r, c in reversed(path)]
        r, c = cur
        for dr, dc, w in D:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < ROWS and 0 <= nc < COLS and free[nr, nc]):
                continue
            if dr and dc and not (free[r + dr, c] and free[r, c + dc]):
                continue
            ng = g[cur] + w
            if ng < g.get((nr, nc), 1e18):
                g[(nr, nc)] = ng
                came[(nr, nc)] = cur
                heapq.heappush(pq, (ng + math.hypot(nr - r1, nc - c1), (nr, nc)))
    return None


def seg_clear(p, q):
    d = math.hypot(q[0] - p[0], q[1] - p[1])
    for k in range(int(d / 0.05) + 2):
        a = min(k * 0.05 / d, 1.0) if d else 0.0
        if obst[int((p[1] + (q[1] - p[1]) * a) / CELL),
                int((p[0] + (q[0] - p[0]) * a) / CELL)]:
            return False
    return True


def simplify(path):
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not seg_clear(path[i], path[j]):
            j -= 1
        out.append(path[j])
        i = j
    return out


def densify(pts, step=2.5):
    """긴 직선 레그를 step 간격으로 분할 — 크로스트랙 이탈 억제."""
    out = [pts[0]]
    for a, b in zip(pts, pts[1:]):
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        for k in range(1, int(d / step) + 1):
            t = k * step / d
            out.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
        out.append(b)
    return out


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Patrol(Node):
    def __init__(self):
        super().__init__("t3_patrol")
        self.odom = None
        self.amcl = None
        qos_tl = QoSProfile(depth=5)
        qos_tl.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(Odometry, "odom", self.on_odom, 10)
        self.create_subscription(PoseWithCovarianceStamped, "amcl_pose",
                                 self.on_amcl, qos_tl)
        self.cmd = self.create_publisher(Twist, "cmd_vel", 10)
        self.path = []
        self.idx = 0
        self.tour_i = 0
        self.last_log = 0.0
        self.create_timer(0.05, self.tick)

    def on_odom(self, m):
        self.odom = m

    def on_amcl(self, m):
        self.amcl = m

    def pose(self):
        p = self.odom.pose.pose
        yaw = math.atan2(2 * (p.orientation.w * p.orientation.z
                              + p.orientation.x * p.orientation.y),
                         1 - 2 * (p.orientation.y ** 2 + p.orientation.z ** 2))
        return p.position.x, p.position.y, yaw

    def plan_next(self):
        x, y, _ = self.pose()
        typ, si = TOUR[self.tour_i % len(TOUR)]
        goal = tuple(stations[typ][si])
        raw = astar((x, y), goal)
        if raw is None:
            self.get_logger().warn(f"경로 실패 → {typ}_{si} 건너뜀")
            self.tour_i += 1
            return
        self.path = densify(simplify(raw))
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
        d = math.hypot(tx - x, ty - y)
        if d < WP_TOL:
            self.idx += 1
            return
        herr = wrap(math.atan2(ty - y, tx - x) - yaw)
        tw = Twist()
        tw.angular.z = max(-W_MAX, min(W_MAX, 1.8 * herr))
        tw.linear.x = 0.0 if abs(herr) > 0.7 else V_MAX * max(0.25, math.cos(herr))
        self.cmd.publish(tw)

        now = time.monotonic()
        if now - self.last_log > 5.0 and self.amcl is not None:
            a = self.amcl.pose.pose.position
            err = math.hypot(a.x - x, a.y - y)
            print(f"[patrol] odom=({x:.2f},{y:.2f}) amcl=({a.x:.2f},{a.y:.2f}) "
                  f"err={err:.3f} m wp {self.idx}/{len(self.path)}", flush=True)
            self.last_log = now


def main():
    rclpy.init()
    n = Patrol()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.cmd.publish(Twist())


if __name__ == "__main__":
    main()
