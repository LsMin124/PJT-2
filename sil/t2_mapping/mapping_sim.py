"""T2 매핑 런 — iw.hub + 2D RTX 라이다, /scan·/odom·/tf·/clock 발행 + 패트롤 주행.

실제 AMR 커미셔닝 절차(설치 시 SLAM 1회 → 지도 고정)의 재현.
지도는 외부 slam_toolbox가 생성한다. 방(벽 4·박스 3)은 좌표를 아는 GT라서
SLAM 지도와의 오차 비교(V&V 자료)가 가능하다.

실행(서버):
  source /opt/ros/humble/setup.bash
  cd ~/isaacsim && ./python.sh <repo>/sil/t2_mapping/mapping_sim.py            # 패트롤(기본)
  cd ~/isaacsim && ./python.sh <repo>/sil/t2_mapping/mapping_sim.py --manual   # 텔레옵 조종

SLAM(별도 셸):
  ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true \
      slam_params_file:=<repo>/sil/t2_mapping/slam_params.yaml
지도 저장:
  ros2 run nav2_map_server map_saver_cli -f <repo>/sil/t2_mapping/out/map --ros-args -p use_sim_time:=true
"""

import argparse

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--manual", action="store_true", help="패트롤 대신 /cmd_vel 텔레옵")
parser.add_argument("--laps", type=int, default=2, help="패트롤 바퀴 수")
parser.add_argument("--spawn", type=float, nargs=2, default=[-1.8, -1.8], help="로봇 시작 위치 x y")
args, _ = parser.parse_known_args()

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
simulation_app.update()

import math

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.prim as prim_utils
import numpy as np
import omni.graph.core as og
import omni.timeline
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.robot.experimental.wheeled_robots.controllers import DifferentialController
from isaacsim.robot.experimental.wheeled_robots.robots import WheeledRobot
from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor
from pxr import Gf, UsdGeom, UsdPhysics

ASSETS = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0"
ENV_USD = ASSETS + "/Isaac/Environments/Grid/default_environment.usd"
IWHUB_USD = ASSETS + "/Isaac/Robots/Idealworks/iwhub/iw_hub.usd"

WHEEL_RADIUS = 0.115
WHEEL_BASE = 0.413
DT = 1.0 / 60.0
MAX_LIN = 1.2
MAX_ANG = 1.5

LIDAR_OFFSET = [0.0, 0.0, 0.60]  # 로봇 몸체 위 — 자기 몸 반사 방지

# ── GT 방: (중심 x, y, z) / (전체 치수 dx, dy, dz) — 12 x 9 m, 벽 두께 0.2 ──
ROOM = [
    ("wall_e", (6.0, 0.0, 0.5), (0.2, 9.4, 1.0)),
    ("wall_w", (-6.0, 0.0, 0.5), (0.2, 9.4, 1.0)),
    ("wall_n", (0.0, 4.5, 0.5), (12.4, 0.2, 1.0)),
    ("wall_s", (0.0, -4.5, 0.5), (12.4, 0.2, 1.0)),
    ("box_a", (-2.5, 2.5, 0.4), (1.0, 1.0, 0.8)),
    ("box_b", (-3.0, -2.0, 0.4), (0.8, 1.6, 0.8)),
    ("box_c", (2.0, -2.6, 0.4), (1.2, 0.6, 0.8)),  # ROOM v2: 남측 회랑 1.5m 확보(1.1m는 통행 불가 실측 판정)
]

# 패트롤: 0.6 m/s 전진 360스텝(3.6 m) + π/8 rad/s 좌회전 240스텝(90°) — 회전 중 스캔 왜곡 완화
FWD_STEPS, TURN_STEPS = 360, 240
CYCLE = FWD_STEPS + TURN_STEPS
PATROL_FRAMES = args.laps * 4 * CYCLE

GRAPH = "/World/ros2_graph"

stage_utils.set_stage_up_axis("Z")
stage_utils.set_stage_units(meters_per_unit=1.0)
stage_utils.add_reference_to_stage(usd_path=ENV_USD, path="/World/env")

stage = stage_utils.get_current_stage()
for name, pos, dims in ROOM:
    cube = UsdGeom.Cube.Define(stage, f"/World/room/{name}")
    cube.GetSizeAttr().Set(1.0)
    xf = UsdGeom.Xformable(cube.GetPrim())
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    xf.AddScaleOp().Set(Gf.Vec3f(*dims))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

# 토픽 제어 장애물 — /obstacle_cmd (linear.z>0.5일 때 linear.x,y로 순간이동)
obs = UsdGeom.Cube.Define(stage, "/World/obstacle")
obs.GetSizeAttr().Set(1.0)
_obs_xf = UsdGeom.Xformable(obs.GetPrim())
obs_translate = _obs_xf.AddTranslateOp()
obs_translate.Set(Gf.Vec3d(20.0, 20.0, 0.7))
_obs_xf.AddScaleOp().Set(Gf.Vec3f(0.6, 0.6, 1.4))  # 라이다 평면(~0.7m)보다 높게 — 0.6m 큐브는 빔이 위로 지나감(실측)
UsdPhysics.CollisionAPI.Apply(obs.GetPrim())

robot = WheeledRobot(
    paths="/World/iw_hub",
    wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
    usd_path=IWHUB_USD,
    positions=[args.spawn[0], args.spawn[1], 0.05],  # 기본: 사각 패트롤이 방 중앙에 오도록
)
controller = DifferentialController(wheel_radius=WHEEL_RADIUS, wheel_base=WHEEL_BASE)

# 2D 회전 라이다 — 반드시 "움직이는 링크(체시스 강체)" 하위에 부착해야 한다.
# articulation 루트 Xform 밑에 두면 물리는 링크만 움직여 라이다가 스폰 위치에
# 고정된다(08-27 실증: 지도 전체가 회전·오프셋된 원인).
chassis_path = None
for prim in stage.Traverse():
    if prim.GetName() == "left_wheel_joint" and str(prim.GetPath()).startswith("/World/iw_hub"):
        joint = UsdPhysics.RevoluteJoint(prim)
        cands = [str(t) for rel in (joint.GetBody0Rel(), joint.GetBody1Rel()) for t in rel.GetTargets()]
        chassis_path = next((c for c in cands if "wheel" not in c.lower()), None)
        break
if chassis_path is None:
    raise RuntimeError("iw_hub 체시스 링크를 찾지 못함 (left_wheel_joint body 관계 확인)")
print(f"[t2_mapping] lidar mount: {chassis_path}/lidar", flush=True)

lidar = Lidar.create(
    path=chassis_path + "/lidar",
    config="Example_Rotary_2D",
    tick_rate=10.0,
    translations=[LIDAR_OFFSET],
)

SimulationManager.setup_simulation(dt=DT, device="cpu")
physics_scene = SimulationManager.get_physics_scenes()[0]
physics_scene.set_enabled_gpu_dynamics(False)
simulation_app.update()


def _laser_scan_meta(prim):
    rotation_rate = float(prim.GetAttribute("omni:sensor:Core:scanRateBaseHz").Get() or 0)
    near_range = float(prim.GetAttribute("omni:sensor:Core:nearRangeM").Get() or 0)
    far_range = float(prim.GetAttribute("omni:sensor:Core:farRangeM").Get() or 0)
    firing_rate = int(prim.GetAttribute("omni:sensor:Core:patternFiringRateHz").Get() or 0)
    if rotation_rate <= 0 or firing_rate <= 0:
        raise RuntimeError("lidar prim scan-rate metadata missing")
    return {
        "horizontalFov": 360.0,
        "horizontalResolution": 360.0 * rotation_rate / firing_rate,
        "depthRange": [near_range, far_range],
        "rotationRate": rotation_rate,
        "azimuthRange": [-180.0, 180.0],
    }


sensor_2d = LidarSensor(lidar, annotators=[])
sensor_2d.attach_writer(
    "RtxLidarROS2PublishLaserScan",
    topicName="scan",
    frameId="lidar_link",
    **_laser_scan_meta(prim_utils.get_prim_at_path(lidar.paths[0])),
)

try:
    from isaacsim.core.utils.viewports import set_camera_view

    set_camera_view(eye=[5.0, 5.0, 3.5], target=[0.0, 0.0, 0.3])
except Exception as exc:
    set_camera_view = None
    print(f"[t2_mapping] camera setup skipped: {exc}")

CAM_OFFSET = np.array([-4.5, -4.5, 3.0])
CAM_ALPHA = 0.06
_cam = {"eye": None, "target": None}

# ── ROS2 그래프: cmd_vel 구독 + clock·odom·TF(odom→base_link, base_link→lidar_link) ──
og.Controller.edit(
    {"graph_path": GRAPH, "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("tick", "omni.graph.action.OnPlaybackTick"),
            ("ctx", "isaacsim.ros2.bridge.ROS2Context"),
            ("sub_twist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
            ("pub_clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ("pub_odom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
            ("tf_odom", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
            ("tf_lidar", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
            ("sub_obs", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("sub_twist.inputs:topicName", "cmd_vel"),
            ("sub_obs.inputs:topicName", "obstacle_cmd"),
            ("pub_clock.inputs:topicName", "clock"),
            ("pub_odom.inputs:topicName", "odom"),
            ("pub_odom.inputs:odomFrameId", "odom"),
            ("pub_odom.inputs:chassisFrameId", "base_link"),
            ("pub_odom.inputs:publishRawVelocities", True),
            ("tf_odom.inputs:parentFrameId", "odom"),
            ("tf_odom.inputs:childFrameId", "base_link"),
            ("tf_lidar.inputs:parentFrameId", "base_link"),
            ("tf_lidar.inputs:childFrameId", "lidar_link"),
            ("tf_lidar.inputs:translation", LIDAR_OFFSET),
            ("tf_lidar.inputs:rotation", [0.0, 0.0, 0.0, 1.0]),
        ],
        og.Controller.Keys.CONNECT: [
            ("tick.outputs:tick", "sub_twist.inputs:execIn"),
            ("tick.outputs:tick", "pub_clock.inputs:execIn"),
            ("tick.outputs:tick", "pub_odom.inputs:execIn"),
            ("tick.outputs:tick", "tf_odom.inputs:execIn"),
            ("tick.outputs:tick", "tf_lidar.inputs:execIn"),
            ("tick.outputs:tick", "sub_obs.inputs:execIn"),
            ("ctx.outputs:context", "sub_twist.inputs:context"),
            ("ctx.outputs:context", "pub_clock.inputs:context"),
            ("ctx.outputs:context", "pub_odom.inputs:context"),
            ("ctx.outputs:context", "tf_odom.inputs:context"),
            ("ctx.outputs:context", "tf_lidar.inputs:context"),
            ("ctx.outputs:context", "sub_obs.inputs:context"),
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
tfo_ts = attr("tf_odom.inputs:timeStamp")
tfo_tr = attr("tf_odom.inputs:translation")
tfo_rot = attr("tf_odom.inputs:rotation")
tfl_ts = attr("tf_lidar.inputs:timeStamp")
obs_lin = attr("sub_obs.outputs:linearVelocity")

timeline = omni.timeline.get_timeline_interface()

app_utils.play()
app_utils.update_app(steps=10)
mode = "manual" if args.manual else f"patrol({args.laps}바퀴)"
print(f"[t2_mapping] ready — 모드 {mode}, /scan·/odom·/tf·/clock 발행 중", flush=True)

reset_needed = False
frame = 0
patrol_done = False
while simulation_app.is_running():
    simulation_app.update()
    frame += 1
    if frame == 1 or frame % 1200 == 0:
        print(f"[t2_mapping] frame {frame} alive", flush=True)
    if not app_utils.is_playing() and not reset_needed:
        reset_needed = True
    if app_utils.is_playing():
        if reset_needed:
            app_utils.stop()
            app_utils.update_app(steps=5)
            app_utils.play()
            app_utils.update_app(steps=5)
            reset_needed = False

        if args.manual:
            lin = np.asarray(og.Controller.get(twist_lin)).reshape(-1)
            ang = np.asarray(og.Controller.get(twist_ang)).reshape(-1)
            command = [
                float(np.clip(lin[0], -MAX_LIN, MAX_LIN)),
                float(np.clip(ang[2], -MAX_ANG, MAX_ANG)),
            ]
        elif frame < PATROL_FRAMES:
            command = [0.6, 0.0] if (frame % CYCLE) < FWD_STEPS else [0.0, math.pi / 8]  # 저속 회전 — 스캔 스미어 절반
        else:
            command = [0.0, 0.0]
            if not patrol_done:
                patrol_done = True
                print("[t2_mapping] patrol done — 지도 저장 가능", flush=True)
        robot.apply_wheel_actions(controller.forward(command))

        pos_w, ori_w = robot.get_world_poses()
        p = np.asarray(pos_w.numpy()).reshape(-1)[:3]
        q = np.asarray(ori_w.numpy()).reshape(-1)[:4]  # (w, x, y, z)
        lv_w, av_w = robot.get_velocities()
        lv = np.asarray(lv_w.numpy()).reshape(-1)[:3]
        av = np.asarray(av_w.numpy()).reshape(-1)[:3]
        t = timeline.get_current_time()
        q_ros = [float(q[1]), float(q[2]), float(q[3]), float(q[0])]
        p_ros = [float(p[0]), float(p[1]), float(p[2])]

        og.Controller.set(clock_ts, float(t))
        og.Controller.set(odom_ts, float(t))
        og.Controller.set(odom_pos, p_ros)
        og.Controller.set(odom_ori, q_ros)
        og.Controller.set(odom_lin, [float(lv[0]), float(lv[1]), float(lv[2])])
        og.Controller.set(odom_ang, [float(av[0]), float(av[1]), float(av[2])])
        og.Controller.set(tfo_ts, float(t))
        og.Controller.set(tfo_tr, p_ros)
        og.Controller.set(tfo_rot, q_ros)
        og.Controller.set(tfl_ts, float(t))

        ocmd = np.asarray(og.Controller.get(obs_lin)).reshape(-1)
        if ocmd[2] > 0.5:
            obs_translate.Set(Gf.Vec3d(float(ocmd[0]), float(ocmd[1]), 0.7))

        if set_camera_view is not None:
            want_eye = p + CAM_OFFSET
            want_tgt = np.array([p[0], p[1], 0.3])
            if _cam["eye"] is None:
                _cam["eye"], _cam["target"] = want_eye.copy(), want_tgt.copy()
            else:
                _cam["eye"] += CAM_ALPHA * (want_eye - _cam["eye"])
                _cam["target"] += CAM_ALPHA * (want_tgt - _cam["target"])
            set_camera_view(eye=_cam["eye"].tolist(), target=_cam["target"].tolist())

app_utils.stop()
simulation_app.close()
