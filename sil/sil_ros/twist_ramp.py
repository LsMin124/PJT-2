"""ROS 와 무관한 순수 제어 계산 — follower 의 가속 램프 · 사각형 안전 필드 · 정지/해제 히스테리시스를 분리.

  clamp · ramp            명령 포화 · 가감속 제한(명령 수준, T1 실측 환류)
  front_clearance         LaserScan 배열 → 전방 진행 대역 안 최근접 전방 거리(사각형 보호 필드)
  SafetyField             정지(front < stop_d) → 해제(front > clear_d) 히스테리시스 + 막힘 경고 타이머
표준 math 만 쓴다(rclpy 불필요).
"""
import math


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def ramp(prev, target, max_delta):
    """prev 에서 target 으로 최대 max_delta 만큼만 이동한 값 (가속 램프 1스텝)."""
    return prev + max(-max_delta, min(max_delta, target - prev))


def front_clearance(ranges, angle_min, angle_increment, range_min, range_max, lat_half, fov_half):
    """사각형 보호 필드 — 시야 |각| < fov_half 이고 |측면| < lat_half 인 점 중 가장 가까운 전방 거리.

    원뿔 섹터는 측면 물체(통로 옆 박스)에 오탐하므로 산업 필드처럼 로봇 전방 진행 대역만 본다.
    유효 점이 없으면 inf.
    """
    best = float("inf")
    ang = angle_min
    for r in ranges:
        if range_min < r < range_max and abs(ang) < fov_half:
            fx = r * math.cos(ang)
            if 0.0 < fx < best and abs(r * math.sin(ang)) < lat_half:
                best = fx
        ang += angle_increment
    return best


class SafetyField:
    """안전 정지 히스테리시스. step() 이 이벤트(STOP · RESUME · BLOCKED · None)를 돌려주고 stopped 를 갱신한다."""

    STOP, RESUME, BLOCKED = "stop", "resume", "blocked"

    def __init__(self, blocked_warn_s):
        self.blocked_warn_s = blocked_warn_s   # 정지가 이보다 길면 BLOCKED (반복 경고 간격)
        self.stopped = False
        self.stop_t = None
        self.stops = 0

    def step(self, front, stop_d, clear_d, t):
        if self.stopped:
            if front > clear_d:
                self.stopped = False
                return self.RESUME
            if self.stop_t is not None and t - self.stop_t > self.blocked_warn_s:
                self.stop_t = t
                return self.BLOCKED
            return None
        if front < stop_d:
            self.stopped = True
            self.stop_t = t
            self.stops += 1
            return self.STOP
        return None
