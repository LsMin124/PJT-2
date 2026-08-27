"""T1 Teleop — iw.hub 1대, /cmd_vel 구독 구동 + /clock·/odom 발행.

SIL 로드맵 T1: 기존 데모(iwhub_warehouse_demo.py)의 스크립트 내장 주행을
ROS2 토픽 제어로 교체한다. 브리지 파이프라인 개통이 목적.

실행(서버):
  source /opt/ros/humble/setup.bash
  cd ~/isaacsim && ./python.sh ~/workspace/PJT-2/sil/t1_teleop/teleop_sim.py
조종(별도 셸):
  source /opt/ros/humble/setup.bash
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
관전: Isaac Sim WebRTC Streaming Client -> 서버 IP (tailscale 100.89.12.112)
"""

from isaacsim import SimulationApp

CONFIG = {
    "width": 1280,
    "height": 720,
    "window_width": 1920,
    "window_height": 1080,
    "headless": True,
    "hide_ui": False,
}

simulation_app = SimulationApp(launch_config=CONFIG)
simulation_app.set_setting("/app/window/drawMouse", True)

import faulthandler

faulthandler.enable()

from isaacsim.core.experimental.utils.app import enable_extension

enable_extension("omni.kit.livestream.app")
enable_extension("isaacsim.ros2.bridge")

import math

import isaacsim.core.experimental.utils.app as app_utils
import numpy as np
import omni.graph.core as og
import omni.timeline
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.robot.experimental.wheeled_robots.controllers import DifferentialController
from isaacsim.robot.experimental.wheeled_robots.robots import WheeledRobot

ASSETS = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0"
ENV_USD = ASSETS + "/Isaac/Environments/Grid/default_environment.usd"
IWHUB_USD = ASSETS + "/Isaac/Robots/Idealworks/iwhub/iw_hub.usd"

# iw.hub 구동 기하 — 08-24 실구동 확인값
WHEEL_RADIUS = 0.115
WHEEL_BASE = 0.413

DT = 1.0 / 60.0
# 안전 클램프 (스펙 상한 근사, 추정)
MAX_LIN = 1.2  # m/s
MAX_ANG = 1.5  # rad/s

GRAPH = "/World/ros2_graph"

stage_utils.set_stage_up_axis("Z")
stage_utils.set_stage_units(meters_per_unit=1.0)
stage_utils.add_reference_to_stage(usd_path=ENV_USD, path="/World/env")

robot = WheeledRobot(
    paths="/World/iw_hub",
    wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
    usd_path=IWHUB_USD,
    positions=[0.0, 0.0, 0.05],
)
controller = DifferentialController(wheel_radius=WHEEL_RADIUS, wheel_base=WHEEL_BASE)

SimulationManager.setup_simulation(dt=DT, device="cpu")
physics_scene = SimulationManager.get_physics_scenes()[0]
physics_scene.set_enabled_gpu_dynamics(False)

try:
    from isaacsim.core.utils.viewports import set_camera_view

    set_camera_view(eye=[5.0, 5.0, 3.5], target=[0.0, 0.0, 0.3])
except Exception as exc:
    print(f"[t1_teleop] camera setup skipped: {exc}")

# ── ROS2 I/O 그래프: cmd_vel 구독, clock·odom 발행 ──
og.Controller.edit(
    {"graph_path": GRAPH, "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("tick", "omni.graph.action.OnPlaybackTick"),
            ("ctx", "isaacsim.ros2.bridge.ROS2Context"),
            ("sub_twist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
            ("pub_clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ("pub_odom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("sub_twist.inputs:topicName", "cmd_vel"),
            ("pub_clock.inputs:topicName", "clock"),
            ("pub_odom.inputs:topicName", "odom"),
            ("pub_odom.inputs:odomFrameId", "odom"),
            ("pub_odom.inputs:chassisFrameId", "base_link"),
            ("pub_odom.inputs:publishRawVelocities", True),
        ],
        og.Controller.Keys.CONNECT: [
            ("tick.outputs:tick", "sub_twist.inputs:execIn"),
            ("tick.outputs:tick", "pub_clock.inputs:execIn"),
            ("tick.outputs:tick", "pub_odom.inputs:execIn"),
            ("ctx.outputs:context", "sub_twist.inputs:context"),
            ("ctx.outputs:context", "pub_clock.inputs:context"),
            ("ctx.outputs:context", "pub_odom.inputs:context"),
        ],
    },
)

attr = lambda p: og.Controller.attribute(f"{GRAPH}/{p}")
twist_lin = attr("sub_twist.outputs:linearVelocity")
twist_ang = attr("sub_twist.outputs:angularVelocity")
clock_ts = attr("pub_clock.inputs:timeStamp")
odom_ts = attr("pub_odom.inputs:timeStamp")
odom_pos = attr("pub_odom.inputs:position")
odom_ori = attr("pub_odom.inputs:orientation")
odom_lin = attr("pub_odom.inputs:linearVelocity")
odom_ang = attr("pub_odom.inputs:angularVelocity")

timeline = omni.timeline.get_timeline_interface()

app_utils.play()
app_utils.update_app(steps=10)
print("[t1_teleop] ready — /cmd_vel 대기, /clock·/odom 발행 중", flush=True)

_prev = {"t": None, "p": None, "yaw": None}


def pose_and_vel():
    """월드 포즈 + 속도. get_velocities 부재 시 유한차분 폴백."""
    pos_w, ori_w = robot.get_world_poses()
    p = np.asarray(pos_w.numpy()).reshape(-1)[:3]
    q = np.asarray(ori_w.numpy()).reshape(-1)[:4]  # 실험 API: (w, x, y, z)
    t = timeline.get_current_time()
    try:
        lv_w, av_w = robot.get_velocities()
        lv = np.asarray(lv_w.numpy()).reshape(-1)[:3]
        av = np.asarray(av_w.numpy()).reshape(-1)[:3]
    except Exception:
        yaw = math.atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] ** 2 + q[3] ** 2))
        if _prev["t"] is None or t <= _prev["t"]:
            lv = np.zeros(3)
            av = np.zeros(3)
        else:
            dt = t - _prev["t"]
            lv = (p - _prev["p"]) / dt
            dyaw = math.atan2(math.sin(yaw - _prev["yaw"]), math.cos(yaw - _prev["yaw"]))
            av = np.array([0.0, 0.0, dyaw / dt])
        _prev.update(t=t, p=p.copy(), yaw=yaw)
    return t, p, q, lv, av


reset_needed = False
frame = 0
while simulation_app.is_running():
    simulation_app.update()
    frame += 1
    if frame == 1 or frame % 600 == 0:
        print(f"[t1_teleop] frame {frame} alive", flush=True)
    if not app_utils.is_playing() and not reset_needed:
        reset_needed = True
    if app_utils.is_playing():
        if reset_needed:
            app_utils.stop()
            app_utils.update_app(steps=5)
            app_utils.play()
            app_utils.update_app(steps=5)
            reset_needed = False
        lin = np.asarray(og.Controller.get(twist_lin)).reshape(-1)
        ang = np.asarray(og.Controller.get(twist_ang)).reshape(-1)
        v = float(np.clip(lin[0], -MAX_LIN, MAX_LIN))
        w = float(np.clip(ang[2], -MAX_ANG, MAX_ANG))
        robot.apply_wheel_actions(controller.forward([v, w]))

        t, p, q, lv, av = pose_and_vel()
        og.Controller.set(clock_ts, float(t))
        og.Controller.set(odom_ts, float(t))
        og.Controller.set(odom_pos, [float(p[0]), float(p[1]), float(p[2])])
        # 실험 API (w,x,y,z) → ROS (x,y,z,w)
        og.Controller.set(odom_ori, [float(q[1]), float(q[2]), float(q[3]), float(q[0])])
        og.Controller.set(odom_lin, [float(lv[0]), float(lv[1]), float(lv[2])])
        og.Controller.set(odom_ang, [float(av[0]), float(av[1]), float(av[2])])

app_utils.stop()
simulation_app.close()
