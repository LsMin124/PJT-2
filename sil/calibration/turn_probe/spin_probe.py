#!/usr/bin/env python3
"""스팟턴 실측 프로브 — warehouse_sim(iw.hub) 상대로 /cmd_vel 각속도 스텝을 주고 /odom(정답 포즈)으로
회전 응답(정상 각속도·상승 시간), 회전 중심(오돔 원점 대비 원 궤적 반경과 차체 기준 종·횡 오프셋),
정지 관성(지령 0 이후 관성 각·정지 시간)을 잰다. 마지막에 T4 TurnController로 90° 회전 4회를
폐루프 실행해 틱 시간을 잰다.

Humble python3로 실행(Isaac 아님). warehouse_sim이 ready 상태일 때:
  source /opt/ros/humble/setup.bash && python3 spin_probe.py --label run1
출력: out/spin_<label>.csv (t x y yaw wz cmd_w phase) · out/spin_<label>.json (요약)
"""
import argparse
import csv
import json
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "t4_agent", "src", "amr_agent"))
from amr_agent.control.primitives import Limits, TurnController, wrap  # noqa: E402

STEPS = (0.3, 0.6, 0.9, 1.2, 1.5)   # rad/s — warehouse_sim MAX_ANG 1.5, T4 w_max 1.2
HOLD_S, REST_S = 6.0, 4.0
RATE_HZ = 30.0
N_TURNS = 4
SETTLE_S = 0.5


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Probe(Node):
    def __init__(self):
        super().__init__("spin_probe")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 50)
        self.rows = []
        self.last = None
        self.cmd_w = 0.0
        self.phase = "idle"

    def on_odom(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p = m.pose.pose.position
        self.last = (t, p.x, p.y, yaw_of(m.pose.pose.orientation), m.twist.twist.angular.z)
        self.rows.append(self.last + (self.cmd_w, self.phase))

    def send(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_w = float(w)
        self.pub.publish(msg)

    def run_for(self, v, w, dur, phase):
        self.phase = phase
        t_end = time.monotonic() + dur
        while time.monotonic() < t_end:
            self.send(v, w)
            rclpy.spin_once(self, timeout_sec=1.0 / RATE_HZ)

    def wait_odom(self, timeout=30.0):
        t_end = time.monotonic() + timeout
        while self.last is None and time.monotonic() < t_end:
            rclpy.spin_once(self, timeout_sec=0.2)
        return self.last is not None

    def turn90(self, k):
        """T4 primitive_controller와 같은 TurnController(기본 Limits)로 +90° 폐루프 회전."""
        t0, x0, y0, yaw0, _ = self.last
        target = wrap(yaw0 + math.pi / 2.0)
        tc = TurnController(target, tol=0.03, limits=Limits())
        self.phase = f"turn90_{k}"
        t_prev = time.monotonic()
        w_peak = 0.0
        n = 0
        while not tc.done and n < 600:
            now = time.monotonic()
            dt = min(0.2, max(1e-3, now - t_prev))
            t_prev = now
            yaw = self.last[3]
            _, w, _, _ = tc.step(yaw, dt)
            w_peak = max(w_peak, abs(w))
            self.send(0.0, w)
            n += 1
            rclpy.spin_once(self, timeout_sec=0.05)
        t1 = self.last[0]
        self.run_for(0.0, 0.0, SETTLE_S, f"turn90_{k}_settle")
        t2, x2, y2, yaw2, _ = self.last
        return dict(k=k, done=tc.done, sim_s=round(t1 - t0, 3), settled_sim_s=round(t2 - t0, 3),
                    final_err_deg=round(math.degrees(wrap(target - yaw2)), 2),
                    drift_m=round(math.hypot(x2 - x0, y2 - y0), 4), w_peak=round(w_peak, 3))


def circle_fit(xs, ys):
    """Kasa 대수 원 맞춤 → (R, cx, cy). 점이 거의 안 움직이면 평균점 기준 RMS 반경."""
    import numpy as np
    xs, ys = np.asarray(xs), np.asarray(ys)
    if xs.std() < 2e-3 and ys.std() < 2e-3:
        cx, cy = xs.mean(), ys.mean()
        return float(np.sqrt(((xs - cx) ** 2 + (ys - cy) ** 2).mean())), float(cx), float(cy)
    A = np.column_stack([xs, ys, np.ones_like(xs)])
    b = -(xs ** 2 + ys ** 2)
    (a, bb, c), *_ = np.linalg.lstsq(A, b, rcond=None)
    cx, cy = -a / 2.0, -bb / 2.0
    R = math.sqrt(max(0.0, cx * cx + cy * cy - c))
    return float(R), float(cx), float(cy)


def analyze(rows):
    import numpy as np
    t = np.array([r[0] for r in rows])
    x = np.array([r[1] for r in rows])
    y = np.array([r[2] for r in rows])
    yaw = np.array([r[3] for r in rows])
    wz = np.array([r[4] for r in rows])
    ph = [r[6] for r in rows]
    yawu = np.unwrap(yaw)
    res = {}
    for w in STEPS:
        name = f"step_{w:.1f}"
        idx = [i for i, p in enumerate(ph) if p == name]
        rest = [i for i, p in enumerate(ph) if p == name + "_rest"]
        if len(idx) < 10:
            continue
        ts, te = t[idx[0]], t[idx[-1]]
        ss = [i for i in idx if t[i] >= te - 3.0]           # 정상 구간 = 스텝 마지막 3 s(시뮬 시간)
        w_ss = float(np.median(wz[ss]))
        slope = float(np.polyfit(t[ss], yawu[ss], 1)[0])    # yaw 미분 교차검증
        rise = next((t[i] - ts for i in idx if abs(wz[i]) >= 0.9 * abs(w_ss)), None)
        R, cx, cy = circle_fit(x[ss], y[ss])
        x0, y0, p0 = x[idx[0]], y[idx[0]], yaw[idx[0]]
        dx, dy = cx - x0, cy - y0
        lon = dx * math.cos(p0) + dy * math.sin(p0)         # 시작 헤딩 기준 종방향(+앞)
        lat = -dx * math.sin(p0) + dy * math.cos(p0)        # 횡방향(+좌)
        coast = stop_t = None
        if rest:
            coast = float(math.degrees(yawu[rest[-1]] - yawu[rest[0]]))
            stop_t = next((t[i] - t[rest[0]] for i in rest if abs(wz[i]) < 0.02), None)
        res[name] = dict(
            cmd_rad_s=w, w_ss_rad_s=round(w_ss, 4), ratio=round(w_ss / w, 3), yaw_slope_rad_s=round(slope, 4),
            rise90_s=None if rise is None else round(float(rise), 3),
            turned_deg=round(math.degrees(yawu[idx[-1]] - yawu[idx[0]]), 1),
            circle_R_m=round(R, 4), center_lon_m=round(lon, 4), center_lat_m=round(lat, 4),
            xy_drift_m=round(float(math.hypot(x[idx[-1]] - x0, y[idx[-1]] - y0)), 4),
            coast_deg=None if coast is None else round(coast, 2),
            stop_s=None if stop_t is None else round(float(stop_t), 3),
            sim_dt_ms=round(float(np.median(np.diff(t[idx])) * 1000.0), 2), n=len(idx))
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", default="run")
    ap.add_argument("--out", default=os.path.join(HERE, "out"))
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    rclpy.init()
    node = Probe()
    if not node.wait_odom():
        print("[probe] /odom 수신 없음 — warehouse_sim ready 확인", flush=True)
        return 1
    wall0 = time.monotonic()
    node.run_for(0.0, 0.0, 2.0, "idle")
    for w in STEPS:
        node.run_for(0.0, w, HOLD_S, f"step_{w:.1f}")
        node.run_for(0.0, 0.0, REST_S, f"step_{w:.1f}_rest")
        print(f"[probe] step {w:.1f} rad/s 완료 (yaw {math.degrees(node.last[3]):+.1f}°)", flush=True)
    turns = [node.turn90(k) for k in range(N_TURNS)]
    node.run_for(0.0, 0.0, 1.0, "end")
    wall = time.monotonic() - wall0
    sim = node.rows[-1][0] - node.rows[0][0]

    csv_path = os.path.join(args.out, f"spin_{args.label}.csv")
    with open(csv_path, "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["t", "x", "y", "yaw", "wz", "cmd_w", "phase"])
        wr.writerows(node.rows)
    summary = dict(label=args.label, wall_s=round(wall, 1), sim_s=round(sim, 1), rtf=round(sim / wall, 3),
                   odom_hz=round(len(node.rows) / wall, 1), steps=analyze(node.rows), turn90=turns,
                   limits=Limits().__dict__)
    with open(os.path.join(args.out, f"spin_{args.label}.json"), "w") as f:
        json.dump(summary, f, indent=1, ensure_ascii=False)

    print(f"[probe] wall {wall:.1f}s · sim {sim:.1f}s · RTF {sim / wall:.2f} · odom {len(node.rows) / wall:.0f} Hz")
    print("[probe] cmd  w_ss   ratio rise90  R_m    lon     lat     coast°  stop_s")
    for k, s in summary["steps"].items():
        print(f"[probe] {s['cmd_rad_s']:.1f}  {s['w_ss_rad_s']:.3f}  {s['ratio']:.3f} {s['rise90_s']}  "
              f"{s['circle_R_m']:.4f} {s['center_lon_m']:+.4f} {s['center_lat_m']:+.4f} "
              f"{s['coast_deg']} {s['stop_s']}")
    for tr in turns:
        print(f"[probe] turn90 #{tr['k']} sim {tr['sim_s']}s (settled {tr['settled_sim_s']}s) "
              f"err {tr['final_err_deg']}° drift {tr['drift_m']} m w_peak {tr['w_peak']} done={tr['done']}")
    node.send(0.0, 0.0)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
