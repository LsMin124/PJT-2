"""Grid motion primitive controllers (pure Python, unit-testable).

TurnController  : spot turn to a target heading.
DriveController : straight line from start to goal while holding a heading;
                  reverse=True drives backward (v < 0) with the same heading held.

Both produce (v, w) with acceleration ramps because the simulated iw.hub responds
almost instantly (T1 measurement: ~6 m/s^2), so real-vehicle limits live here.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def ramp(prev: float, target: float, max_delta: float) -> float:
    return prev + clamp(target - prev, -max_delta, max_delta)


@dataclass
class Limits:
    v_max: float = 0.8        # m/s  (T1: asset tops out at 0.835)
    w_max: float = 1.2        # rad/s
    a_max: float = 1.0        # m/s^2 command ramp (real iw.hub class)
    alpha_max: float = 2.5    # rad/s^2
    v_min: float = 0.05       # creep speed near the goal
    w_min: float = 0.12       # minimum turn rate while not settled
    decel_ratio: float = 0.5  # braking profile uses decel_ratio*a_max so the ramp can follow it
    stop_lead: float = 0.01   # [m] start the final stop this far before the goal


@dataclass
class Gains:
    k_turn: float = 2.5
    k_heading: float = 2.5
    k_lateral: float = 1.5
    lateral_cap: float = 0.5  # rad, max travel-direction correction from cross-track error
    v_heading_gate: float = 0.35  # rad, no forward speed while heading error exceeds this


class TurnController:
    def __init__(self, target_theta: float, tol: float, limits: Limits = Limits(),
                 gains: Gains = Gains(), settle_cycles: int = 3) -> None:
        self.target = target_theta
        self.tol = tol
        self.lim = limits
        self.g = gains
        self.settle_cycles = settle_cycles
        self.w_prev = 0.0
        self._settled = 0
        self.done = False

    def step(self, yaw: float, dt: float):
        """Returns (v, w, done, heading_error)."""
        err = wrap(self.target - yaw)
        if abs(err) < self.tol:
            self._settled += 1
        else:
            self._settled = 0
        if self._settled >= self.settle_cycles:
            self.done = True
            self.w_prev = 0.0
            return 0.0, 0.0, True, err
        w_des = clamp(self.g.k_turn * err, -self.lim.w_max, self.lim.w_max)
        if abs(err) >= self.tol and abs(w_des) < self.lim.w_min:
            w_des = math.copysign(self.lim.w_min, err)
        # do not exceed the rate that can still stop within the remaining angle
        w_stop = math.sqrt(2.0 * self.lim.decel_ratio * self.lim.alpha_max * abs(err)) + self.lim.w_min
        w_des = clamp(w_des, -w_stop, w_stop)
        self.w_prev = ramp(self.w_prev, w_des, self.lim.alpha_max * dt)
        return 0.0, self.w_prev, False, err


class DriveController:
    def __init__(self, start_xy, goal_xy, reverse: bool, tol_xy: float,
                 limits: Limits = Limits(), gains: Gains = Gains(),
                 max_speed: float = 0.0) -> None:
        sx, sy = start_xy
        gx, gy = goal_xy
        L = math.hypot(gx - sx, gy - sy)
        self.sx, self.sy, self.gx, self.gy = sx, sy, gx, gy
        self.dx, self.dy = ((gx - sx) / L, (gy - sy) / L) if L > 1e-9 else (1.0, 0.0)
        self.travel_heading = math.atan2(self.dy, self.dx)   # direction of motion
        self.reverse = reverse
        self.tol = tol_xy
        self.lim = limits
        self.g = gains
        self.v_cap = min(limits.v_max, max_speed) if max_speed > 0 else limits.v_max
        self.v_prev = 0.0
        self.w_prev = 0.0
        self.reached = False
        self.done = False

    def errors(self, x: float, y: float):
        along = (self.gx - x) * self.dx + (self.gy - y) * self.dy       # remaining along path
        cross = self.dx * (y - self.sy) - self.dy * (x - self.sx)        # +: robot left of path
        dist = math.hypot(self.gx - x, self.gy - y)
        return along, cross, dist

    def step(self, x: float, y: float, yaw: float, dt: float):
        """Returns (v, w, done, distance_remaining, heading_error)."""
        along, cross, dist = self.errors(x, y)
        phi = wrap(yaw + (math.pi if self.reverse else 0.0))            # actual travel direction
        e_h = wrap(self.travel_heading - phi)
        if not self.reached and along <= self.lim.stop_lead:
            self.reached = True
        if self.reached:
            self.v_prev = ramp(self.v_prev, 0.0, self.lim.a_max * dt)
            self.w_prev = ramp(self.w_prev, 0.0, self.lim.alpha_max * dt)
            if abs(self.v_prev) < 0.02 and abs(self.w_prev) < 0.05:
                self.done = True
                self.v_prev = self.w_prev = 0.0
            return self.v_prev, self.w_prev, self.done, dist, e_h
        # travel-direction correction from cross-track error (same sign rule fwd/rev)
        corr = clamp(-self.g.k_lateral * cross, -self.g.lateral_cap, self.g.lateral_cap)
        w_des = clamp(self.g.k_heading * wrap(e_h + corr), -self.lim.w_max, self.lim.w_max)
        v_stop = math.sqrt(2.0 * self.lim.decel_ratio * self.lim.a_max * max(along - self.lim.stop_lead, 0.0)) + self.lim.v_min
        v_des = min(self.v_cap, v_stop)
        if abs(e_h) > self.g.v_heading_gate:
            v_des = 0.0
        if self.reverse:
            v_des = -v_des
        self.v_prev = ramp(self.v_prev, v_des, self.lim.a_max * dt)
        self.w_prev = ramp(self.w_prev, w_des, self.lim.alpha_max * dt)
        return self.v_prev, self.w_prev, False, dist, e_h
