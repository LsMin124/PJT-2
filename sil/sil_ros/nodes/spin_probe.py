"""스팟턴 실측 프로브 — warehouse_sim(iw.hub) 상대로 /cmd_vel 각속도 스텝을 주고 /odom(정답 포즈)으로
회전 응답(정상 각속도·상승 시간), 회전 중심(오돔 원점 대비 원 궤적 반경과 차체 기준 종·횡 오프셋),
정지 관성(지령 0 이후 관성 각·정지 시간)을 잰다. 마지막에 T4 TurnController로 90° 회전 4회를
폐루프 실행해 틱 시간을 잰다.

Humble python3로 실행(Isaac 아님). warehouse_sim이 ready 상태일 때:
  source /opt/ros/humble/setup.bash && python3 spin_probe.py --label run1     # sil/calibration/turn_probe/ 셔ム
  python3 -m sil_ros.nodes.spin_probe --label run1                             # sil/ 에서 직접
출력: sil/calibration/turn_probe/out/spin_<label>.csv (t x y yaw wz cmd_w phase) · spin_<label>.json (요약)
T4 의 amr_agent.control.primitives(TurnController · Limits) 는 sil/t4_agent/src 를 sys.path 에 넣어 그대로 쓴다.
"""
import argparse
import csv
import json
import math
import os
import sys
import time
from typing import NamedTuple

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from sil_ros import SIL_ROOT
from sil_ros.geometry import stamp_to_sec, wrap, yaw_of

AMR_AGENT_SRC = os.path.join(SIL_ROOT, "t4_agent", "src", "amr_agent")
sys.path.insert(0, AMR_AGENT_SRC)
from amr_agent.control.primitives import Limits, TurnController  # noqa: E402

STEPS = (0.3, 0.6, 0.9, 1.2, 1.5)   # rad/s — warehouse_sim MAX_ANG 1.5, T4 w_max 1.2
HOLD_S, REST_S = 6.0, 4.0
RATE_HZ = 30.0
N_TURNS = 4
SETTLE_S = 0.5
IDLE_S, END_S = 2.0, 1.0            # 시작 전 · 종료 전 정지 유지
ODOM_WAIT_S = 30.0                  # 첫 /odom 대기 한계
ODOM_QOS_DEPTH, CMD_QOS_DEPTH = 50, 10
TURN_TOL = 0.03                     # rad — 90° 회전 완료 허용 오차
TURN_MAX_ITERS = 600                # 폐루프 회전 최대 반복 (≈ 30 s)
TURN_SPIN_S = 0.05
DT_MIN, DT_MAX = 1e-3, 0.2          # 폐루프 dt 클램프
SS_WINDOW_S = 3.0                   # 정상 구간 = 스텝 마지막 3 s(시뮬 시간)
RISE_FRAC = 0.9                     # 상승 시간 판정 비율 (정상 각속도 대비)
STOP_W = 0.02                       # rad/s — 정지 판정 각속도
MIN_STEP_SAMPLES = 10
STILL_STD_M = 2e-3                  # 원 맞춤 대신 평균점 RMS 를 쓰는 정지 판정 표준편차
DEFAULT_OUT_DIR = os.path.join(SIL_ROOT, "calibration", "turn_probe", "out")


class Probe(Node):
    def __init__(self):
        super().__init__("spin_probe")
        self.pub = self.create_publisher(Twist, "/cmd_vel", CMD_QOS_DEPTH)
        self.create_subscription(Odometry, "/odom", self.on_odom, ODOM_QOS_DEPTH)
        self.rows = []
        self.last = None
        self.cmd_w = 0.0
        self.phase = "idle"

    def on_odom(self, m):
        t = stamp_to_sec(m.header.stamp)
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

    def wait_odom(self, timeout=ODOM_WAIT_S):
        t_end = time.monotonic() + timeout
        while self.last is None and time.monotonic() < t_end:
            rclpy.spin_once(self, timeout_sec=0.2)
        return self.last is not None

    def turn90(self, k):
        """T4 primitive_controller와 같은 TurnController(기본 Limits)로 +90° 폐루프 회전."""
        t0, x0, y0, yaw0, _ = self.last
        target = wrap(yaw0 + math.pi / 2.0)
        tc = TurnController(target, tol=TURN_TOL, limits=Limits())
        self.phase = f"turn90_{k}"
        t_prev = time.monotonic()
        w_peak = 0.0
        n = 0
        while not tc.done and n < TURN_MAX_ITERS:
            now = time.monotonic()
            dt = min(DT_MAX, max(DT_MIN, now - t_prev))
            t_prev = now
            _, w, _, _ = tc.step(self.last[3], dt)
            w_peak = max(w_peak, abs(w))
            self.send(0.0, w)
            n += 1
            rclpy.spin_once(self, timeout_sec=TURN_SPIN_S)
        t1 = self.last[0]
        self.run_for(0.0, 0.0, SETTLE_S, f"turn90_{k}_settle")
        t2, x2, y2, yaw2, _ = self.last
        return dict(k=k, done=tc.done, sim_s=round(t1 - t0, 3), settled_sim_s=round(t2 - t0, 3),
                    final_err_deg=round(math.degrees(wrap(target - yaw2)), 2),
                    drift_m=round(math.hypot(x2 - x0, y2 - y0), 4), w_peak=round(w_peak, 3))


def circle_fit(xs, ys):
    """Kasa 대수 원 맞춤 → (R, cx, cy). 점이 거의 안 움직이면 평균점 기준 RMS 반경."""
    xs, ys = np.asarray(xs), np.asarray(ys)
    if xs.std() < STILL_STD_M and ys.std() < STILL_STD_M:
        cx, cy = xs.mean(), ys.mean()
        return float(np.sqrt(((xs - cx) ** 2 + (ys - cy) ** 2).mean())), float(cx), float(cy)
    A = np.column_stack([xs, ys, np.ones_like(xs)])
    b = -(xs ** 2 + ys ** 2)
    (a, bb, c), *_ = np.linalg.lstsq(A, b, rcond=None)
    cx, cy = -a / 2.0, -bb / 2.0
    R = math.sqrt(max(0.0, cx * cx + cy * cy - c))
    return float(R), float(cx), float(cy)


class Series(NamedTuple):
    """rows 열 분해 — t · x · y · yaw · yawu(unwrap) · wz."""
    t: np.ndarray
    x: np.ndarray
    y: np.ndarray
    yaw: np.ndarray
    yawu: np.ndarray
    wz: np.ndarray


def _series(rows):
    yaw = np.array([r[3] for r in rows])
    return Series(np.array([r[0] for r in rows]), np.array([r[1] for r in rows]),
                  np.array([r[2] for r in rows]), yaw, np.unwrap(yaw), np.array([r[4] for r in rows]))


def _rotation_center(s, ss, i0):
    """정상 구간 원 맞춤 → (R, 시작 헤딩 기준 종방향(+앞), 횡방향(+좌)) 오프셋."""
    R, cx, cy = circle_fit(s.x[ss], s.y[ss])
    x0, y0, p0 = s.x[i0], s.y[i0], s.yaw[i0]
    dx, dy = cx - x0, cy - y0
    lon = dx * math.cos(p0) + dy * math.sin(p0)
    lat = -dx * math.sin(p0) + dy * math.cos(p0)
    return R, lon, lat


def _coast(s, rest):
    """정지 지령 뒤 관성 각(deg)과 정지까지 시간(s) — rest 구간 없으면 (None, None)."""
    if not rest:
        return None, None
    coast = float(math.degrees(s.yawu[rest[-1]] - s.yawu[rest[0]]))
    stop_t = next((s.t[i] - s.t[rest[0]] for i in rest if abs(s.wz[i]) < STOP_W), None)
    return coast, stop_t


def _step_summary(w, idx, rest, s):
    ts, te = s.t[idx[0]], s.t[idx[-1]]
    ss = [i for i in idx if s.t[i] >= te - SS_WINDOW_S]
    w_ss = float(np.median(s.wz[ss]))
    slope = float(np.polyfit(s.t[ss], s.yawu[ss], 1)[0])    # yaw 미분 교차검증
    rise = next((s.t[i] - ts for i in idx if abs(s.wz[i]) >= RISE_FRAC * abs(w_ss)), None)
    R, lon, lat = _rotation_center(s, ss, idx[0])
    coast, stop_t = _coast(s, rest)
    return dict(
        cmd_rad_s=w, w_ss_rad_s=round(w_ss, 4), ratio=round(w_ss / w, 3), yaw_slope_rad_s=round(slope, 4),
        rise90_s=None if rise is None else round(float(rise), 3),
        turned_deg=round(math.degrees(s.yawu[idx[-1]] - s.yawu[idx[0]]), 1),
        circle_R_m=round(R, 4), center_lon_m=round(lon, 4), center_lat_m=round(lat, 4),
        xy_drift_m=round(float(math.hypot(s.x[idx[-1]] - s.x[idx[0]], s.y[idx[-1]] - s.y[idx[0]])), 4),
        coast_deg=None if coast is None else round(coast, 2),
        stop_s=None if stop_t is None else round(float(stop_t), 3),
        sim_dt_ms=round(float(np.median(np.diff(s.t[idx])) * 1000.0), 2), n=len(idx))


def analyze(rows):
    """스텝별 요약 {step_w: {...}} — 샘플이 MIN_STEP_SAMPLES 미만인 스텝은 생략."""
    s = _series(rows)
    ph = [r[6] for r in rows]
    res = {}
    for w in STEPS:
        name = f"step_{w:.1f}"
        idx = [i for i, p in enumerate(ph) if p == name]
        rest = [i for i, p in enumerate(ph) if p == name + "_rest"]
        if len(idx) < MIN_STEP_SAMPLES:
            continue
        res[name] = _step_summary(w, idx, rest, s)
    return res


def run_sequence(node):
    """idle → 각속도 스텝(유지 · 정지) × STEPS → 90° 폐루프 회전 × N_TURNS → end. turn 요약 리스트 반환."""
    node.run_for(0.0, 0.0, IDLE_S, "idle")
    for w in STEPS:
        node.run_for(0.0, w, HOLD_S, f"step_{w:.1f}")
        node.run_for(0.0, 0.0, REST_S, f"step_{w:.1f}_rest")
        print(f"[probe] step {w:.1f} rad/s 완료 (yaw {math.degrees(node.last[3]):+.1f}°)", flush=True)
    turns = [node.turn90(k) for k in range(N_TURNS)]
    node.run_for(0.0, 0.0, END_S, "end")
    return turns


def write_outputs(out_dir, label, rows, summary):
    with open(os.path.join(out_dir, f"spin_{label}.csv"), "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["t", "x", "y", "yaw", "wz", "cmd_w", "phase"])
        wr.writerows(rows)
    with open(os.path.join(out_dir, f"spin_{label}.json"), "w") as f:
        json.dump(summary, f, indent=1, ensure_ascii=False)


def print_summary(summary, wall, sim, n_rows):
    print(f"[probe] wall {wall:.1f}s · sim {sim:.1f}s · RTF {sim / wall:.2f} · odom {n_rows / wall:.0f} Hz")
    print("[probe] cmd  w_ss   ratio rise90  R_m    lon     lat     coast°  stop_s")
    for k, s in summary["steps"].items():
        print(f"[probe] {s['cmd_rad_s']:.1f}  {s['w_ss_rad_s']:.3f}  {s['ratio']:.3f} {s['rise90_s']}  "
              f"{s['circle_R_m']:.4f} {s['center_lon_m']:+.4f} {s['center_lat_m']:+.4f} "
              f"{s['coast_deg']} {s['stop_s']}")
    for tr in summary["turn90"]:
        print(f"[probe] turn90 #{tr['k']} sim {tr['sim_s']}s (settled {tr['settled_sim_s']}s) "
              f"err {tr['final_err_deg']}° drift {tr['drift_m']} m w_peak {tr['w_peak']} done={tr['done']}")


def parse_args(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", default="run")
    ap.add_argument("--out", default=DEFAULT_OUT_DIR)
    return ap.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    os.makedirs(args.out, exist_ok=True)

    rclpy.init()
    node = Probe()
    if not node.wait_odom():
        print("[probe] /odom 수신 없음 — warehouse_sim ready 확인", flush=True)
        return 1
    wall0 = time.monotonic()
    turns = run_sequence(node)
    wall = time.monotonic() - wall0
    sim = node.rows[-1][0] - node.rows[0][0]

    summary = dict(label=args.label, wall_s=round(wall, 1), sim_s=round(sim, 1), rtf=round(sim / wall, 3),
                   odom_hz=round(len(node.rows) / wall, 1), steps=analyze(node.rows), turn90=turns,
                   limits=Limits().__dict__)
    write_outputs(args.out, args.label, node.rows, summary)
    print_summary(summary, wall, sim, len(node.rows))
    node.send(0.0, 0.0)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
