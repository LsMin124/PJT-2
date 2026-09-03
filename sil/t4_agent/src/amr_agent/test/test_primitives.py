"""Closed-loop checks of the primitive controllers on a kinematic unicycle."""
import math

from amr_agent.control.primitives import DriveController, Limits, TurnController, wrap

DT = 0.02


def simulate(ctrl, x, y, yaw, kind, t_max=20.0):
    t = 0.0
    while t < t_max:
        if kind == "turn":
            v, w, done, _ = ctrl.step(yaw, DT)
        else:
            v, w, done, _, _ = ctrl.step(x, y, yaw, DT)
        if done:
            return x, y, yaw, t
        x += v * math.cos(yaw) * DT
        y += v * math.sin(yaw) * DT
        yaw = wrap(yaw + w * DT)
        t += DT
    raise AssertionError("controller did not finish")


def test_turn_left_90():
    x, y, yaw, t = simulate(TurnController(math.pi / 2, 0.03), 0, 0, 0, "turn")
    assert abs(wrap(yaw - math.pi / 2)) < 0.03 and t < 5


def test_drive_forward_one_cell():
    x, y, yaw, t = simulate(DriveController((0, 0), (1, 0), False, 0.05), 0, 0, 0, "drive")
    assert math.hypot(x - 1, y) < 0.03 and abs(yaw) < 0.05 and t < 6


def test_drive_reverse_one_cell_keeps_heading():
    x, y, yaw, t = simulate(DriveController((0, 0), (-1, 0), True, 0.05), 0, 0, 0, "drive")
    assert math.hypot(x + 1, y) < 0.03 and abs(yaw) < 0.05


def test_drive_recovers_lateral_offset():
    # start 0.15 m left of the path with a 10° heading error, 3 m leg
    x, y, yaw, t = simulate(DriveController((0, 0), (3, 0), False, 0.05), 0, 0.15, math.radians(10), "drive")
    assert math.hypot(x - 3, y) < 0.03 and abs(yaw) < 0.06


def test_speed_cap_respected():
    lim = Limits()
    c = DriveController((0, 0), (5, 0), False, 0.05, lim, max_speed=0.3)
    x = y = yaw = 0.0
    vmax = 0.0
    for _ in range(int(20 / DT)):
        v, w, done, _, _ = c.step(x, y, yaw, DT)
        vmax = max(vmax, abs(v))
        if done:
            break
        x += v * math.cos(yaw) * DT
        y += v * math.sin(yaw) * DT
        yaw = wrap(yaw + w * DT)
    assert done and vmax <= 0.3 + 1e-9
