"""T3 warehouse_sim — 창고 씬 + iw.hub + 물리 라이다, ROS2 개통.

AMR 내부 = ROS2 원칙(프로젝트 계획)에 따라 센서·구동을 전부 토픽으로 낸다:
  발행  /scan(LaserScan 720빔 0.5° 0.4~20m) /odom /tf(odom→base_link, base_link→laser)
        /clock (use_sim_time 소스)
  구독  /cmd_vel (Twist — teleop·플래너 공용)
라이다는 물리 레이캐스트(콜라이더 기준) — 콜라이더는 전부 그리드 rect에서 나오므로
map.pgm(동일 그리드)과 원천이 일치, AMCL 정합에 유리. RTX 라이다는 렌더 기하 기준이라
투명 콜라이더 설계와 상충(부적합).

실행(서버): source /opt/ros/humble/setup.bash 후
  cd <isaacsim> && ./python.sh <repo>/sil/t3_warehouse/warehouse_sim.py
localization(별도 셸): ros2 launch <repo>/sil/t3_warehouse/ros2/loc.launch.py
관전: WebRTC(49100) 또는 http://<서버IP>:8211/ (TCP MJPEG — UDP 차단망)
전제: warehouse_scene.usd 존재, Isaac 동시 1인스턴스.
"""

import math
import os

from isaacsim import SimulationApp

app = SimulationApp({"headless": True, "hide_ui": False,
                     "window_width": 1280, "window_height": 720})
app.set_setting("/app/window/drawMouse", True)

from isaacsim.core.experimental.utils.app import enable_extension

enable_extension("omni.kit.livestream.app")
enable_extension("isaacsim.ros2.bridge")

import isaacsim.core.experimental.utils.app as app_utils
import numpy as np
import omni.graph.core as og
import omni.timeline
import usdrt
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.robot.experimental.wheeled_robots.controllers import DifferentialController
from isaacsim.robot.experimental.wheeled_robots.robots import WheeledRobot
from isaacsim.sensors.experimental.physics import Raycast

HERE = os.path.dirname(os.path.abspath(__file__))
SCENE_USD = os.path.join(HERE, "warehouse_scene.usd")
ASSETS = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0"
IWHUB_USD = ASSETS + "/Isaac/Robots/Idealworks/iwhub/iw_hub.usd"

# 실측 운동학 (에셋 콜리전·비주얼 bbox 실측 — T1의 0.115/0.413은 오류였음:
# 바퀴 실린더·메시 반경 0.081, 휠 트랙 tr y=±0.29 → 0.58)
WHEEL_RADIUS = 0.08
WHEEL_BASE = 0.58
MAX_LIN, MAX_ANG = 1.2, 1.5     # 안전 클램프 (T1과 동일)
SPAWN = (34.0, 40.8)            # 남측 코리도, handoff 인근 — 그리드 0·팽창 밖 실측

# 라이다 스펙 — 2D 360°, 0.5° 해상도(720빔), 0.4~20m
LIDAR_Z = 0.45                  # 차체(0.35) 위, omap 스캔 밴드(0.2~1.2) 안
H_RES = 0.5
N_RAYS = int(360.0 / H_RES)
R_MIN, R_MAX = 0.4, 20.0

DT = 1.0 / 60.0
GRAPH = "/World/ros2_graph"

stage_utils.set_stage_up_axis("Z")
stage_utils.set_stage_units(meters_per_unit=1.0)
stage_utils.add_reference_to_stage(usd_path=SCENE_USD, path="/World/warehouse")

robot = WheeledRobot(
    paths="/World/iw_hub",
    wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
    usd_path=IWHUB_USD,
    positions=[SPAWN[0], SPAWN[1], 0.08],   # 축 평형 0.07(지면 -0.01 + r0.08) 위 1cm
)
controller = DifferentialController(wheel_radius=WHEEL_RADIUS, wheel_base=WHEEL_BASE)

# 무구동 lift_joint(프리즘틱) 잠금 — 중력에 서서히 침하해 리프트 콜리전이 바닥에
# 닿으면 주행이 점진 감속 후 고착(시간 기반 ~40s 동결 실측)되는 것을 차단
import omni.usd as _ousd
from pxr import Gf, UsdGeom
from pxr import UsdPhysics as _UsdPhysics

_stage_now = _ousd.get_context().get_stage()
_lift_j = _stage_now.GetPrimAtPath("/World/iw_hub/lift_joint")
if _lift_j.IsValid():
    _drv = _UsdPhysics.DriveAPI.Apply(_lift_j, "linear")
    _drv.CreateTargetPositionAttr(0.0)
    _drv.CreateStiffnessAttr(1.0e5)
    _drv.CreateDampingAttr(1.0e4)
    print("[wsim] lift_joint 위치 드라이브 잠금", flush=True)

# 캐스터 무력화 — iw_hub 캐스터는 스월 축과 바퀴가 동축(트레일 0)이라 자기 정렬이
# 없고, 주행 중 스월이 감기다 관절 한계(±2π)에 걸리면 바퀴가 옆으로 꺾인 채 앵커가
# 되어 구동륜이 공회전(3회 주행 실측: 고착 시 스월 +5.905/-6.054 rad). 스월·롤을
# 드라이브로 잠그고 캐스터 재질을 저마찰(0.05)로 — 뒤축은 "미끄럼 글라이더"가 된다.
for _jn in ("left_swivel_joint", "right_swivel_joint",
            "left_caster_joint", "right_caster_joint"):
    _jp = _stage_now.GetPrimAtPath(f"/World/iw_hub/{_jn}")
    if _jp.IsValid():
        _d = _UsdPhysics.DriveAPI.Apply(_jp, "angular")
        _d.CreateTargetPositionAttr(0.0)
        _d.CreateStiffnessAttr(1.0e4)
        _d.CreateDampingAttr(1.0e3)
_cm = _stage_now.GetPrimAtPath("/World/iw_hub/caster_material")
if _cm.IsValid():
    _m = _UsdPhysics.MaterialAPI.Apply(_cm)
    _m.CreateStaticFrictionAttr(0.05)
    _m.CreateDynamicFrictionAttr(0.05)
    _m.CreateRestitutionAttr(0.0)
print("[wsim] 캐스터 잠금 + 저마찰 글라이더화", flush=True)

# 하부 콜리전 수술 — 에셋의 섀시/리프트/스월/캐스터 Collision은 스케일이 ×100
# 깨져 있음(실측: 섀시 박스 142x10m, 캐스터 구체 r14m) → 전부 비활성하고 바퀴
# 실린더(정상 r0.08)만 남긴 뒤, 자체 섀시 박스 + 저마찰 글라이더 구체 4개로 재구성.
BROKEN_COLS = ("chassis/Collision", "chassis/Collision_01", "lift/Collision",
               "left_swivel/Collision", "right_swivel/Collision",
               "left_caster/Collision", "right_caster/Collision")
for _rel in BROKEN_COLS:
    _p = _stage_now.GetPrimAtPath(f"/World/iw_hub/{_rel}")
    if _p.IsValid():
        _UsdPhysics.CollisionAPI.Apply(_p).CreateCollisionEnabledAttr(False)

from pxr import UsdShade as _UsdShade

_gmat = _UsdShade.Material.Define(_stage_now, "/World/iw_hub/glider_mat")
_gm = _UsdPhysics.MaterialAPI.Apply(_gmat.GetPrim())
_gm.CreateStaticFrictionAttr(0.03)
_gm.CreateDynamicFrictionAttr(0.03)
_gm.CreateRestitutionAttr(0.0)
# 뒤 글라이더 2개만, 바퀴 바닥보다 3mm 높게(-0.077 vs -0.08) — 4점 코플레이너로
# 두면 하중이 저마찰 글라이더에만 실려 구동륜 수직력이 0이 된다(실측: 완전 무이동).
# CoM(헐 중심 x-0.32)이 축 뒤라 앞 글라이더 불필요 — 바퀴·뒤글라이더가 하중 분담.
for _gi, (_gx, _gy) in enumerate(((-0.677, 0.093), (-0.677, -0.093))):
    _sp = UsdGeom.Sphere.Define(_stage_now, f"/World/iw_hub/chassis/glider_{_gi}")
    _sp.CreateRadiusAttr(0.055)
    UsdGeom.Xformable(_sp.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(_gx, _gy, -0.022))
    _UsdPhysics.CollisionAPI.Apply(_sp.GetPrim())
    _UsdShade.MaterialBindingAPI.Apply(_sp.GetPrim()).Bind(
        _gmat, materialPurpose="physics")
    UsdGeom.Imageable(_sp.GetPrim()).MakeInvisible()
# 바퀴 실린더 → 해석적 구체 접촉 (r 0.08, 바퀴 바디 원점=축심). PhysX가 실린더를
# 컨벡스 헐(다면체)로 근사하면 다각형 바퀴가 되어 비포장도로처럼 덜컹거림(사용자
# 관전 실측) — 구체는 해석적이라 완전 매끄러운 굴림. 마찰은 에셋 wheel_material 상속.
_wmat = _UsdShade.Material.Get(_stage_now, "/World/iw_hub/wheel_material")
for _side in ("left_wheel", "right_wheel"):
    _cyl = _stage_now.GetPrimAtPath(f"/World/iw_hub/{_side}/Cylinder")
    if _cyl.IsValid():
        _UsdPhysics.CollisionAPI.Apply(_cyl).CreateCollisionEnabledAttr(False)
    _ws = UsdGeom.Sphere.Define(_stage_now, f"/World/iw_hub/{_side}/contact_sphere")
    _ws.CreateRadiusAttr(0.08)
    _UsdPhysics.CollisionAPI.Apply(_ws.GetPrim())
    if _wmat:
        _UsdShade.MaterialBindingAPI.Apply(_ws.GetPrim()).Bind(
            _wmat, materialPurpose="physics")
    UsdGeom.Imageable(_ws.GetPrim()).MakeInvisible()

# 섀시 외곽 박스(벽·장애물 접촉용, 바닥과 여유 0.03) — 시각 메시 bbox 실측 근사
_cb = UsdGeom.Cube.Define(_stage_now, "/World/iw_hub/chassis/hull")
_cb.GetSizeAttr().Set(1.0)
_cxf = UsdGeom.Xformable(_cb.GetPrim())
_cxf.AddTranslateOp().Set(Gf.Vec3d(-0.32, 0.0, 0.13))
_cxf.AddScaleOp().Set(Gf.Vec3f(1.40, 0.62, 0.22))
_UsdPhysics.CollisionAPI.Apply(_cb.GetPrim())
UsdGeom.Imageable(_cb.GetPrim()).MakeInvisible()
print("[wsim] 하부 콜리전 수술: 깨진 콜리전 7개 비활성 + 글라이더 4 + 헐 박스", flush=True)

# 물리 레이캐스트 라이다 — 반드시 "리지드바디 링크"(chassis) 밑에 부착.
# 관절 루트 Xform에 붙이면 USD 트랜스폼이 시뮬 중 갱신되지 않아(fabric) 센서가
# 스폰 위치에 동결 + 로봇 자체도 점진 정지 (실측 — 서쪽 레이 19.60 고정으로 판정)
origins, directions = [], []
for hi in range(N_RAYS):
    a = math.radians(-180.0 + 360.0 * hi / N_RAYS)
    origins.append([0.0, 0.0, 0.0])
    directions.append([math.cos(a), math.sin(a), 0.0])
lidar = Raycast.create(
    "/World/iw_hub/chassis/lidar",
    min_range=R_MIN,
    max_range=R_MAX,
    ray_origins=origins,
    ray_directions=directions,
    translations=[[0.0, 0.0, LIDAR_Z]],
)
LIDAR_PATH = lidar.paths[0]

SimulationManager.setup_simulation(dt=DT, device="cpu")
SimulationManager.get_physics_scenes()[0].set_enabled_gpu_dynamics(False)

# ── ROS2 I/O 그래프 ──
og.Controller.edit(
    {"graph_path": GRAPH, "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("tick", "omni.graph.action.OnPlaybackTick"),
            ("ctx", "isaacsim.ros2.bridge.ROS2Context"),
            ("sub_twist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
            ("pub_clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ("pub_odom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
            ("read_lidar", "isaacsim.sensors.physics.IsaacReadRaycastSensor"),
            ("pub_scan", "isaacsim.ros2.bridge.ROS2PublishLaserScan"),
            ("tf_odom", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
            ("tf_laser", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("sub_twist.inputs:topicName", "cmd_vel"),
            ("pub_clock.inputs:topicName", "clock"),
            ("pub_odom.inputs:topicName", "odom"),
            ("pub_odom.inputs:odomFrameId", "odom"),
            ("pub_odom.inputs:chassisFrameId", "base_link"),
            ("pub_odom.inputs:publishRawVelocities", True),
            ("read_lidar.inputs:raycastSensorPrim", [usdrt.Sdf.Path(LIDAR_PATH)]),
            ("pub_scan.inputs:topicName", "scan"),
            ("pub_scan.inputs:frameId", "laser"),
            ("pub_scan.inputs:horizontalFov", 360.0),
            ("pub_scan.inputs:horizontalResolution", H_RES),
            ("pub_scan.inputs:numCols", N_RAYS),
            ("pub_scan.inputs:numRows", 1),
            ("pub_scan.inputs:depthRange", [R_MIN, R_MAX]),
            ("pub_scan.inputs:rotationRate", 0.0),
            ("pub_scan.inputs:azimuthRange", [-180.0, 180.0]),
            ("tf_odom.inputs:parentFrameId", "odom"),
            ("tf_odom.inputs:childFrameId", "base_link"),
            ("tf_laser.inputs:parentFrameId", "base_link"),
            ("tf_laser.inputs:childFrameId", "laser"),
            # staticPublisher=True는 스택 재시작 시 구독자에 전달 안 됨(실측 —
            # tf2에 laser 프레임 부재로 AMCL 스캔 전량 드롭) → 동적 60Hz 발행
            ("tf_laser.inputs:translation", [0.0, 0.0, LIDAR_Z]),
            ("tf_laser.inputs:rotation", [0.0, 0.0, 0.0, 1.0]),
        ],
        og.Controller.Keys.CONNECT: [
            ("tick.outputs:tick", "sub_twist.inputs:execIn"),
            ("tick.outputs:tick", "pub_clock.inputs:execIn"),
            ("tick.outputs:tick", "pub_odom.inputs:execIn"),
            ("tick.outputs:tick", "read_lidar.inputs:execIn"),
            ("tick.outputs:tick", "tf_odom.inputs:execIn"),
            ("tick.outputs:tick", "tf_laser.inputs:execIn"),
            ("read_lidar.outputs:execOut", "pub_scan.inputs:execIn"),
            ("read_lidar.outputs:depths", "pub_scan.inputs:linearDepthData"),
            ("ctx.outputs:context", "sub_twist.inputs:context"),
            ("ctx.outputs:context", "pub_clock.inputs:context"),
            ("ctx.outputs:context", "pub_odom.inputs:context"),
            ("ctx.outputs:context", "pub_scan.inputs:context"),
            ("ctx.outputs:context", "tf_odom.inputs:context"),
            ("ctx.outputs:context", "tf_laser.inputs:context"),
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
scan_ts = attr("pub_scan.inputs:timeStamp")
tfo_ts = attr("tf_odom.inputs:timeStamp")
tfo_tr = attr("tf_odom.inputs:translation")
tfo_rot = attr("tf_odom.inputs:rotation")
tfl_ts = attr("tf_laser.inputs:timeStamp")

timeline = omni.timeline.get_timeline_interface()
app_utils.play()
app_utils.update_app(steps=10)

from http_stream import HttpViewer

viewer = HttpViewer(port=8211)
print("[wsim] ready — /scan·/odom·/tf·/clock 발행, /cmd_vel 대기", flush=True)

CAM_OFFSET = np.array([-4.0, -4.0, 7.0])   # 고각 — 스폰 인근 사무실 벽(3m)에 안 가리게
CAM_ALPHA = 0.06
_cam = {"eye": None, "target": None}
_prev = {"t": None, "p": None, "yaw": None}
_dbg = {}


def pose_and_vel():
    """월드 포즈 + 속도 (T1 검증 패턴 — get_velocities 부재 시 유한차분)."""
    pos_w, ori_w = robot.get_world_poses()
    p = np.asarray(pos_w.numpy()).reshape(-1)[:3]
    q = np.asarray(ori_w.numpy()).reshape(-1)[:4]  # (w, x, y, z)
    t = timeline.get_current_time()
    try:
        lv_w, av_w = robot.get_velocities()
        lv = np.asarray(lv_w.numpy()).reshape(-1)[:3]
        av = np.asarray(av_w.numpy()).reshape(-1)[:3]
    except Exception:
        yaw = math.atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] ** 2 + q[3] ** 2))
        if _prev["t"] is None or t <= _prev["t"]:
            lv, av = np.zeros(3), np.zeros(3)
        else:
            dt = t - _prev["t"]
            lv = (p - _prev["p"]) / dt
            dyaw = math.atan2(math.sin(yaw - _prev["yaw"]), math.cos(yaw - _prev["yaw"]))
            av = np.array([0.0, 0.0, dyaw / dt])
        _prev.update(t=t, p=p.copy(), yaw=yaw)
    return t, p, q, lv, av


reset_needed = False
frame = 0
while app.is_running():
    app.update()
    viewer.tick()
    frame += 1
    if frame == 1 or frame % 300 == 0:
        _p = _dbg.get("p", (0.0, 0.0))
        try:
            _d = np.asarray(robot.get_dof_positions().numpy()).reshape(-1)
            _ds = " ".join(f"{x:+.3f}" for x in _d)
        except Exception as e:
            _ds = f"err:{type(e).__name__}"
        _z = _dbg.get("z", 0.0)
        _pi = _dbg.get("pitch", 0.0)
        print(f"[wsim] f{frame} v={_dbg.get('v', 0.0):+.2f} w={_dbg.get('w', 0.0):+.2f} "
              f"p=({_p[0]:.2f},{_p[1]:.2f}) z={_z:+.4f} pitch={_pi:+.3f} dof=[{_ds}]",
              flush=True)
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
        _pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q[0] * q[2] - q[3] * q[1]))))
        _dbg.update(v=v, w=w, p=(p[0], p[1]), z=float(p[2]), pitch=_pitch)
        og.Controller.set(clock_ts, float(t))
        og.Controller.set(odom_ts, float(t))
        og.Controller.set(scan_ts, float(t))
        og.Controller.set(tfo_ts, float(t))
        og.Controller.set(tfl_ts, float(t))
        og.Controller.set(odom_pos, [float(p[0]), float(p[1]), float(p[2])])
        ros_q = [float(q[1]), float(q[2]), float(q[3]), float(q[0])]  # (w,x,y,z)→(x,y,z,w)
        og.Controller.set(odom_ori, ros_q)
        og.Controller.set(odom_lin, [float(lv[0]), float(lv[1]), float(lv[2])])
        og.Controller.set(odom_ang, [float(av[0]), float(av[1]), float(av[2])])
        og.Controller.set(tfo_tr, [float(p[0]), float(p[1]), float(p[2])])
        og.Controller.set(tfo_rot, ros_q)

        want_eye = p + CAM_OFFSET
        want_tgt = np.array([p[0], p[1], 0.3])
        if _cam["eye"] is None:
            _cam["eye"], _cam["target"] = want_eye.copy(), want_tgt.copy()
        else:
            _cam["eye"] += CAM_ALPHA * (want_eye - _cam["eye"])
            _cam["target"] += CAM_ALPHA * (want_tgt - _cam["target"])
        set_camera_view(eye=_cam["eye"].tolist(), target=_cam["target"].tolist())

app_utils.stop()
app.close()
