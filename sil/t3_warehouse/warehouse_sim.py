"""T3 warehouse_sim — 창고 씬 + iw.hub ×N + 물리 라이다, ROS2 개통.

AMR 내부 = ROS2 원칙(프로젝트 계획)에 따라 센서·구동을 전부 토픽으로 낸다:
  발행  /scan(LaserScan 720빔 0.5° 0.4~20m) /odom /tf(odom→base_link, base_link→laser)
        /clock (use_sim_time 소스, 인스턴스당 1회)
  구독  /cmd_vel (Twist — teleop·플래너·T4 에이전트 공용)
라이다는 물리 레이캐스트(콜라이더 기준) — 콜라이더는 전부 그리드 rect에서 나오므로
map.pgm(동일 그리드)과 원천이 일치, AMCL 정합에 유리. RTX 라이다는 렌더 기하 기준이라
투명 콜라이더 설계와 상충(부적합).

다중 로봇(환경변수):
  WSIM_N=3          로봇 수(기본 1). 2 이상이면 자동으로 네임스페이스 모드
  WSIM_NS=1|0       N=1일 때 네임스페이스 강제/해제(기본 auto: N>1만 켬)
  WSIM_SPACING=3.0  스폰 간격 [m] (남측 코리도 y=40.8, x=36.0부터 +x)
  WSIM_SPAWN="x,y;x,y;…"  스폰 좌표 직접 지정(개수 = N)
  네임스페이스 모드: 프림 /World/amr01…, 토픽 /amr01/{cmd_vel,odom,scan}, 프레임 amr01/{odom,base_link,laser}
  (T4 에이전트 `agent.launch.py serial:=amr01`의 기본 토픽과 일치. flat_topics 불필요)
  단일·비네임스페이스(기본): 프림 /World/iw_hub, 토픽 /cmd_vel /odom /scan — T3 loc.launch·patrol 그대로

실행(서버): source /opt/ros/humble/setup.bash 후
  cd <isaacsim> && ./python.sh <repo>/sil/t3_warehouse/warehouse_sim.py
  cd <isaacsim> && WSIM_N=3 ./python.sh <repo>/sil/t3_warehouse/warehouse_sim.py   # 3대, /amr01~03
localization(별도 셸, 단일 모드): ros2 launch <repo>/sil/t3_warehouse/ros2/loc.launch.py
검증: python3 <repo>/sil/t3_warehouse/ros2/multi_check.py --n 3 --drive amr02
관전: WebRTC(49100) 또는 http://<서버IP>:8211/ (TCP MJPEG — UDP 차단망). 카메라는 1번 로봇 추종
전제: warehouse_scene.usd 존재. 동시 실행 한도는 pages/working/isaac_parallel_measure.html.
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
import omni.usd
import usdrt
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.robot.experimental.wheeled_robots.controllers import DifferentialController
from isaacsim.robot.experimental.wheeled_robots.robots import WheeledRobot
from isaacsim.sensors.experimental.physics import Raycast
from pxr import Gf, UsdGeom, UsdPhysics, UsdShade

HERE = os.path.dirname(os.path.abspath(__file__))
SCENE_USD = os.path.join(HERE, "warehouse_scene.usd")
ASSETS = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0"
IWHUB_USD = ASSETS + "/Isaac/Robots/Idealworks/iwhub/iw_hub.usd"

# 실측 운동학 (에셋 콜리전·비주얼 bbox 실측 — T1의 0.115/0.413은 오류였음:
# 바퀴 실린더·메시 반경 0.081, 휠 트랙 tr y=±0.29 → 0.58). 회전 중심 = 구동축 = 루트 원점,
# 차체는 축 기준 앞 0.40 m·뒤 1.03 m (calibration/turn_probe 실측 2026-09-06)
WHEEL_RADIUS = 0.08
WHEEL_BASE = 0.58
MAX_LIN, MAX_ANG = 1.2, 1.5     # 안전 클램프 (T1과 동일)
SPAWN = (34.0, 40.8)            # 남측 코리도, handoff 인근 — 그리드 0·팽창 밖 실측 (단일 모드)
MULTI_SPAWN0 = (36.0, 40.8)     # 다중 모드 첫 로봇 — x 36~60 구간 장애물 이격 ≥ 3.9 m (occupancy_grid 실측)

# 라이다 스펙 — 2D 360°, 0.5° 해상도(720빔), 0.4~20m
LIDAR_Z = 0.45                  # 차체(0.35) 위, omap 스캔 밴드(0.2~1.2) 안
H_RES = 0.5
N_RAYS = int(360.0 / H_RES)
R_MIN, R_MAX = 0.4, 20.0

DT = 1.0 / 60.0

# ── 다중 로봇 구성 ──
N_ROBOTS = max(1, int(os.environ.get("WSIM_N", "1")))
_ns = os.environ.get("WSIM_NS", "auto").strip().lower()
NAMESPACED = (N_ROBOTS > 1) if _ns == "auto" else (_ns in ("1", "true", "yes")) or N_ROBOTS > 1
SPACING = float(os.environ.get("WSIM_SPACING", "3.0"))


def spawn_points():
    raw = os.environ.get("WSIM_SPAWN", "").strip()
    if raw:
        pts = [tuple(float(v) for v in s.split(",")) for s in raw.split(";") if s.strip()]
        if len(pts) != N_ROBOTS:
            raise SystemExit(f"[wsim] WSIM_SPAWN 개수 {len(pts)} ≠ WSIM_N {N_ROBOTS}")
        return pts
    if N_ROBOTS == 1:
        return [SPAWN]
    return [(MULTI_SPAWN0[0] + SPACING * i, MULTI_SPAWN0[1]) for i in range(N_ROBOTS)]


ROBOT_NAMES = [f"amr{i + 1:02d}" for i in range(N_ROBOTS)] if NAMESPACED else ["iw_hub"]

stage_utils.set_stage_up_axis("Z")
stage_utils.set_stage_units(meters_per_unit=1.0)
stage_utils.add_reference_to_stage(usd_path=SCENE_USD, path="/World/warehouse")


def surgery(stage, root):
    """iw_hub 에셋 런타임 수술 — 리프트 잠금, 캐스터 무력화, 깨진 콜리전 교체. root = 로봇 프림 경로."""
    # 무구동 lift_joint(프리즘틱) 잠금 — 중력에 서서히 침하해 리프트 콜리전이 바닥에
    # 닿으면 주행이 점진 감속 후 고착(시간 기반 ~40s 동결 실측)되는 것을 차단
    lift_j = stage.GetPrimAtPath(f"{root}/lift_joint")
    if lift_j.IsValid():
        drv = UsdPhysics.DriveAPI.Apply(lift_j, "linear")
        drv.CreateTargetPositionAttr(0.0)
        drv.CreateStiffnessAttr(1.0e5)
        drv.CreateDampingAttr(1.0e4)

    # 캐스터 무력화 — iw_hub 캐스터는 스월 축과 바퀴가 동축(트레일 0)이라 자기 정렬이
    # 없고, 주행 중 스월이 감기다 관절 한계(±2π)에 걸리면 바퀴가 옆으로 꺾인 채 앵커가
    # 되어 구동륜이 공회전(3회 주행 실측: 고착 시 스월 +5.905/-6.054 rad). 스월·롤을
    # 드라이브로 잠그고 캐스터 재질을 저마찰(0.05)로 — 뒤축은 "미끄럼 글라이더"가 된다.
    for jn in ("left_swivel_joint", "right_swivel_joint", "left_caster_joint", "right_caster_joint"):
        jp = stage.GetPrimAtPath(f"{root}/{jn}")
        if jp.IsValid():
            d = UsdPhysics.DriveAPI.Apply(jp, "angular")
            d.CreateTargetPositionAttr(0.0)
            d.CreateStiffnessAttr(1.0e4)
            d.CreateDampingAttr(1.0e3)
    cm = stage.GetPrimAtPath(f"{root}/caster_material")
    if cm.IsValid():
        m = UsdPhysics.MaterialAPI.Apply(cm)
        m.CreateStaticFrictionAttr(0.05)
        m.CreateDynamicFrictionAttr(0.05)
        m.CreateRestitutionAttr(0.0)

    # 하부 콜리전 수술 — 에셋의 섀시/리프트/스월/캐스터 Collision은 스케일이 ×100
    # 깨져 있음(실측: 섀시 박스 142x10m, 캐스터 구체 r14m) → 전부 비활성하고 바퀴
    # 실린더(정상 r0.08)만 남긴 뒤, 자체 섀시 박스 + 저마찰 글라이더 구체로 재구성.
    for rel in ("chassis/Collision", "chassis/Collision_01", "lift/Collision",
                "left_swivel/Collision", "right_swivel/Collision",
                "left_caster/Collision", "right_caster/Collision"):
        p = stage.GetPrimAtPath(f"{root}/{rel}")
        if p.IsValid():
            UsdPhysics.CollisionAPI.Apply(p).CreateCollisionEnabledAttr(False)

    gmat = UsdShade.Material.Define(stage, f"{root}/glider_mat")
    gm = UsdPhysics.MaterialAPI.Apply(gmat.GetPrim())
    gm.CreateStaticFrictionAttr(0.03)
    gm.CreateDynamicFrictionAttr(0.03)
    gm.CreateRestitutionAttr(0.0)
    # 뒤 글라이더 2개만, 바퀴 바닥보다 3mm 높게(-0.077 vs -0.08) — 4점 코플레이너로
    # 두면 하중이 저마찰 글라이더에만 실려 구동륜 수직력이 0이 된다(실측: 완전 무이동).
    # CoM(헐 중심 x-0.32)이 축 뒤라 앞 글라이더 불필요 — 바퀴·뒤글라이더가 하중 분담.
    for gi, (gx, gy) in enumerate(((-0.677, 0.093), (-0.677, -0.093))):
        sp = UsdGeom.Sphere.Define(stage, f"{root}/chassis/glider_{gi}")
        sp.CreateRadiusAttr(0.055)
        UsdGeom.Xformable(sp.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(gx, gy, -0.022))
        UsdPhysics.CollisionAPI.Apply(sp.GetPrim())
        UsdShade.MaterialBindingAPI.Apply(sp.GetPrim()).Bind(gmat, materialPurpose="physics")
        UsdGeom.Imageable(sp.GetPrim()).MakeInvisible()
    # 바퀴 실린더 → 해석적 구체 접촉 (r 0.08, 바퀴 바디 원점=축심). PhysX가 실린더를
    # 컨벡스 헐(다면체)로 근사하면 다각형 바퀴가 되어 비포장도로처럼 덜컹거림(사용자
    # 관전 실측) — 구체는 해석적이라 완전 매끄러운 굴림. 마찰은 에셋 wheel_material 상속.
    wmat = UsdShade.Material.Get(stage, f"{root}/wheel_material")
    for side in ("left_wheel", "right_wheel"):
        cyl = stage.GetPrimAtPath(f"{root}/{side}/Cylinder")
        if cyl.IsValid():
            UsdPhysics.CollisionAPI.Apply(cyl).CreateCollisionEnabledAttr(False)
        ws = UsdGeom.Sphere.Define(stage, f"{root}/{side}/contact_sphere")
        ws.CreateRadiusAttr(0.08)
        UsdPhysics.CollisionAPI.Apply(ws.GetPrim())
        if wmat:
            UsdShade.MaterialBindingAPI.Apply(ws.GetPrim()).Bind(wmat, materialPurpose="physics")
        UsdGeom.Imageable(ws.GetPrim()).MakeInvisible()

    # 섀시 외곽 박스(벽·장애물 접촉용, 바닥과 여유 0.03) — 시각 메시 bbox 실측 근사
    cb = UsdGeom.Cube.Define(stage, f"{root}/chassis/hull")
    cb.GetSizeAttr().Set(1.0)
    cxf = UsdGeom.Xformable(cb.GetPrim())
    cxf.AddTranslateOp().Set(Gf.Vec3d(-0.32, 0.0, 0.13))
    cxf.AddScaleOp().Set(Gf.Vec3f(1.40, 0.62, 0.22))
    UsdPhysics.CollisionAPI.Apply(cb.GetPrim())
    UsdGeom.Imageable(cb.GetPrim()).MakeInvisible()


def make_lidar(root):
    """물리 레이캐스트 라이다 — 반드시 "리지드바디 링크"(chassis) 밑에 부착.
    관절 루트 Xform에 붙이면 USD 트랜스폼이 시뮬 중 갱신되지 않아(fabric) 센서가
    스폰 위치에 동결 + 로봇 자체도 점진 정지 (실측 — 서쪽 레이 19.60 고정으로 판정)"""
    origins, directions = [], []
    for hi in range(N_RAYS):
        a = math.radians(-180.0 + 360.0 * hi / N_RAYS)
        origins.append([0.0, 0.0, 0.0])
        directions.append([math.cos(a), math.sin(a), 0.0])
    lidar = Raycast.create(f"{root}/chassis/lidar", min_range=R_MIN, max_range=R_MAX,
                           ray_origins=origins, ray_directions=directions,
                           translations=[[0.0, 0.0, LIDAR_Z]])
    return lidar.paths[0]


_stage_now = omni.usd.get_context().get_stage()
robots = []
for name, (sx, sy) in zip(ROBOT_NAMES, spawn_points()):
    root = f"/World/{name}"
    wr = WheeledRobot(paths=root, wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
                      usd_path=IWHUB_USD,
                      positions=[sx, sy, 0.08])   # 축 평형 0.07(지면 -0.01 + r0.08) 위 1cm
    surgery(_stage_now, root)
    robots.append(dict(name=name, root=root, robot=wr,
                       ctrl=DifferentialController(wheel_radius=WHEEL_RADIUS, wheel_base=WHEEL_BASE),
                       lidar=make_lidar(root), prev=dict(t=None, p=None, yaw=None), dbg={}))
print(f"[wsim] 로봇 {N_ROBOTS}대 스폰 — {'네임스페이스 ' + ', '.join(ROBOT_NAMES) if NAMESPACED else '단일(/iw_hub, 평면 토픽)'}; "
      f"하부 콜리전 수술(깨진 콜리전 7개 비활성 + 글라이더 2 + 바퀴 구체 2 + 헐 박스) 로봇별 적용", flush=True)

SimulationManager.setup_simulation(dt=DT, device="cpu")
SimulationManager.get_physics_scenes()[0].set_enabled_gpu_dynamics(False)


def build_graph(r, publish_clock):
    """로봇 1대분 ROS2 I/O 그래프. 네임스페이스 모드면 토픽 amr01/odom → /amr01/odom, 프레임 amr01/base_link."""
    name = r["name"]
    ns = f"{name}/" if NAMESPACED else ""
    graph = f"/World/ros2_graph_{name}" if NAMESPACED else "/World/ros2_graph"
    f_odom, f_base, f_laser = f"{ns}odom", f"{ns}base_link", f"{ns}laser"
    nodes = [
        ("tick", "omni.graph.action.OnPlaybackTick"),
        ("ctx", "isaacsim.ros2.bridge.ROS2Context"),
        ("sub_twist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
        ("pub_odom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
        ("read_lidar", "isaacsim.sensors.physics.IsaacReadRaycastSensor"),
        ("pub_scan", "isaacsim.ros2.bridge.ROS2PublishLaserScan"),
        ("tf_odom", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
        ("tf_laser", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
    ]
    values = [
        ("sub_twist.inputs:topicName", f"{ns}cmd_vel"),
        ("pub_odom.inputs:topicName", f"{ns}odom"),
        ("pub_odom.inputs:odomFrameId", f_odom),
        ("pub_odom.inputs:chassisFrameId", f_base),
        ("pub_odom.inputs:publishRawVelocities", True),
        ("read_lidar.inputs:raycastSensorPrim", [usdrt.Sdf.Path(r["lidar"])]),
        ("pub_scan.inputs:topicName", f"{ns}scan"),
        ("pub_scan.inputs:frameId", f_laser),
        ("pub_scan.inputs:horizontalFov", 360.0),
        ("pub_scan.inputs:horizontalResolution", H_RES),
        ("pub_scan.inputs:numCols", N_RAYS),
        ("pub_scan.inputs:numRows", 1),
        ("pub_scan.inputs:depthRange", [R_MIN, R_MAX]),
        ("pub_scan.inputs:rotationRate", 0.0),
        ("pub_scan.inputs:azimuthRange", [-180.0, 180.0]),
        ("tf_odom.inputs:parentFrameId", f_odom),
        ("tf_odom.inputs:childFrameId", f_base),
        ("tf_laser.inputs:parentFrameId", f_base),
        ("tf_laser.inputs:childFrameId", f_laser),
        # staticPublisher=True는 스택 재시작 시 구독자에 전달 안 됨(실측 —
        # tf2에 laser 프레임 부재로 AMCL 스캔 전량 드롭) → 동적 60Hz 발행
        ("tf_laser.inputs:translation", [0.0, 0.0, LIDAR_Z]),
        ("tf_laser.inputs:rotation", [0.0, 0.0, 0.0, 1.0]),
    ]
    connects = [
        ("tick.outputs:tick", "sub_twist.inputs:execIn"),
        ("tick.outputs:tick", "pub_odom.inputs:execIn"),
        ("tick.outputs:tick", "read_lidar.inputs:execIn"),
        ("tick.outputs:tick", "tf_odom.inputs:execIn"),
        ("tick.outputs:tick", "tf_laser.inputs:execIn"),
        ("read_lidar.outputs:execOut", "pub_scan.inputs:execIn"),
        ("read_lidar.outputs:depths", "pub_scan.inputs:linearDepthData"),
        ("ctx.outputs:context", "sub_twist.inputs:context"),
        ("ctx.outputs:context", "pub_odom.inputs:context"),
        ("ctx.outputs:context", "pub_scan.inputs:context"),
        ("ctx.outputs:context", "tf_odom.inputs:context"),
        ("ctx.outputs:context", "tf_laser.inputs:context"),
    ]
    if publish_clock:
        nodes.append(("pub_clock", "isaacsim.ros2.bridge.ROS2PublishClock"))
        values.append(("pub_clock.inputs:topicName", "clock"))
        connects += [("tick.outputs:tick", "pub_clock.inputs:execIn"),
                     ("ctx.outputs:context", "pub_clock.inputs:context")]
    og.Controller.edit({"graph_path": graph, "evaluator_name": "execution"},
                       {og.Controller.Keys.CREATE_NODES: nodes,
                        og.Controller.Keys.SET_VALUES: values,
                        og.Controller.Keys.CONNECT: connects})
    a = lambda p: og.Controller.attribute(f"{graph}/{p}")
    r.update(twist_lin=a("sub_twist.outputs:linearVelocity"), twist_ang=a("sub_twist.outputs:angularVelocity"),
             odom_ts=a("pub_odom.inputs:timeStamp"), odom_pos=a("pub_odom.inputs:position"),
             odom_ori=a("pub_odom.inputs:orientation"), odom_lin=a("pub_odom.inputs:linearVelocity"),
             odom_ang=a("pub_odom.inputs:angularVelocity"), scan_ts=a("pub_scan.inputs:timeStamp"),
             tfo_ts=a("tf_odom.inputs:timeStamp"), tfo_tr=a("tf_odom.inputs:translation"),
             tfo_rot=a("tf_odom.inputs:rotation"), tfl_ts=a("tf_laser.inputs:timeStamp"),
             clock_ts=a("pub_clock.inputs:timeStamp") if publish_clock else None)


for i, r in enumerate(robots):
    build_graph(r, publish_clock=(i == 0))

timeline = omni.timeline.get_timeline_interface()
app_utils.play()
app_utils.update_app(steps=10)

from http_stream import HttpViewer

viewer = HttpViewer(port=8211)
topics = ", ".join(f"/{n}/{{cmd_vel,odom,scan}}" for n in ROBOT_NAMES) if NAMESPACED else "/scan·/odom·/tf·/clock 발행, /cmd_vel 대기"
print(f"[wsim] ready — {topics}", flush=True)

CAM_OFFSET = np.array([-4.0, -4.0, 7.0])   # 고각 — 스폰 인근 사무실 벽(3m)에 안 가리게
CAM_ALPHA = 0.06
_cam = {"eye": None, "target": None}


def pose_and_vel(r):
    """월드 포즈 + 속도 (T1 검증 패턴 — get_velocities 부재 시 유한차분)."""
    robot, prev = r["robot"], r["prev"]
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
        if prev["t"] is None or t <= prev["t"]:
            lv, av = np.zeros(3), np.zeros(3)
        else:
            dt = t - prev["t"]
            lv = (p - prev["p"]) / dt
            dyaw = math.atan2(math.sin(yaw - prev["yaw"]), math.cos(yaw - prev["yaw"]))
            av = np.array([0.0, 0.0, dyaw / dt])
        prev.update(t=t, p=p.copy(), yaw=yaw)
    return t, p, q, lv, av


def step_robot(r):
    lin = np.asarray(og.Controller.get(r["twist_lin"])).reshape(-1)
    ang = np.asarray(og.Controller.get(r["twist_ang"])).reshape(-1)
    v = float(np.clip(lin[0], -MAX_LIN, MAX_LIN))
    w = float(np.clip(ang[2], -MAX_ANG, MAX_ANG))
    r["robot"].apply_wheel_actions(r["ctrl"].forward([v, w]))

    t, p, q, lv, av = pose_and_vel(r)
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q[0] * q[2] - q[3] * q[1]))))
    r["dbg"].update(v=v, w=w, p=(p[0], p[1]), z=float(p[2]), pitch=pitch)
    for key in ("odom_ts", "scan_ts", "tfo_ts", "tfl_ts", "clock_ts"):
        if r[key] is not None:
            og.Controller.set(r[key], float(t))
    ros_q = [float(q[1]), float(q[2]), float(q[3]), float(q[0])]  # (w,x,y,z)→(x,y,z,w)
    og.Controller.set(r["odom_pos"], [float(p[0]), float(p[1]), float(p[2])])
    og.Controller.set(r["odom_ori"], ros_q)
    og.Controller.set(r["odom_lin"], [float(lv[0]), float(lv[1]), float(lv[2])])
    og.Controller.set(r["odom_ang"], [float(av[0]), float(av[1]), float(av[2])])
    og.Controller.set(r["tfo_tr"], [float(p[0]), float(p[1]), float(p[2])])
    og.Controller.set(r["tfo_rot"], ros_q)
    return p


def debug_line(frame, r):
    d = r["dbg"]
    p = d.get("p", (0.0, 0.0))
    try:
        dof = np.asarray(r["robot"].get_dof_positions().numpy()).reshape(-1)
        ds = " ".join(f"{x:+.3f}" for x in dof)
    except Exception as e:
        ds = f"err:{type(e).__name__}"
    return (f"[wsim] f{frame} {r['name']} v={d.get('v', 0.0):+.2f} w={d.get('w', 0.0):+.2f} "
            f"p=({p[0]:.2f},{p[1]:.2f}) z={d.get('z', 0.0):+.4f} pitch={d.get('pitch', 0.0):+.3f} dof=[{ds}]")


reset_needed = False
frame = 0
while app.is_running():
    app.update()
    viewer.tick()
    frame += 1
    if frame == 1 or frame % 300 == 0:
        for r in robots:
            print(debug_line(frame, r), flush=True)
    if not app_utils.is_playing() and not reset_needed:
        reset_needed = True
    if app_utils.is_playing():
        if reset_needed:
            app_utils.stop()
            app_utils.update_app(steps=5)
            app_utils.play()
            app_utils.update_app(steps=5)
            reset_needed = False
        p0 = None
        for r in robots:
            p = step_robot(r)
            if p0 is None:
                p0 = p
        want_eye = p0 + CAM_OFFSET
        want_tgt = np.array([p0[0], p0[1], 0.3])
        if _cam["eye"] is None:
            _cam["eye"], _cam["target"] = want_eye.copy(), want_tgt.copy()
        else:
            _cam["eye"] += CAM_ALPHA * (want_eye - _cam["eye"])
            _cam["target"] += CAM_ALPHA * (want_tgt - _cam["target"])
        set_camera_view(eye=_cam["eye"].tolist(), target=_cam["target"].tolist())

app_utils.stop()
app.close()
