"""T3 warehouse_sim — 창고 씬 + iw.hub ×N + 물리 라이다, ROS 2 개통.

AMR 내부 = ROS 2 원칙에 따라 센서·구동을 전부 토픽으로 낸다:
  발행  /scan(LaserScan 720빔 0.5° 0.4~20m) /odom /tf(odom→base_link, base_link→laser) /clock(인스턴스당 1회)
  구독  /cmd_vel (Twist — teleop·플래너·T4 에이전트 공용)

환경변수:
  WSIM_N=3          로봇 수(기본 1). 2 이상이면 자동으로 네임스페이스 모드 (/amr01/{cmd_vel,odom,scan}, 프레임 amr01/…)
  WSIM_NS=1|0       N=1일 때 네임스페이스 강제/해제(기본 auto)
  WSIM_SPACING=3.0  스폰 간격 [m] (남측 코리도 y=40.8, x=36.0부터 +x)
  WSIM_SPAWN="x,y;x,y;…"  스폰 좌표 직접 지정(개수 = N)
  WSIM_LEAN=a~g     경량 프로파일(병렬 실측 구성). WSIM_INST=i 는 포트 분리(8211+i, 49100+i), WSIM_PHYS_USD 는 물리 전용 스테이지
  WSIM_MODEL=iw.hub 로봇 프로파일 이름

실행: source /opt/ros/humble/setup.bash && cd ~/isaacsim && ./python.sh <repo>/sil/apps/warehouse_sim.py
검증: python3 <repo>/sil/t3_warehouse/ros2/multi_check.py --n 3 --drive amr02
"""
import os
import sys

import _bootstrap  # noqa: F401  (sil/ 을 sys.path 에)

from sil_isaac import app as sil_app
from sil_isaac.robot.profile import get_profile
from sil_isaac.scene.warehouse import DEFAULT_SPACING, parse_spawn_list, robot_names, spawn_points

LOG_EVERY = 300


def read_config() -> dict:
    n = max(1, int(os.environ.get("WSIM_N", "1")))
    ns = os.environ.get("WSIM_NS", "auto").strip().lower()
    namespaced = (n > 1) if ns == "auto" else (ns in ("1", "true", "yes")) or n > 1
    raw = os.environ.get("WSIM_SPAWN", "").strip()
    explicit = parse_spawn_list(raw) if raw else None
    if explicit and len(explicit) != n:
        raise SystemExit(f"[wsim] WSIM_SPAWN 개수 {len(explicit)} ≠ WSIM_N {n}")
    spacing = float(os.environ.get("WSIM_SPACING", str(DEFAULT_SPACING)))
    profile, instance = sil_app.profile_from_env()
    return dict(n=n, namespaced=namespaced, points=spawn_points(n, spacing, explicit),
                names=robot_names(n, namespaced), model=os.environ.get("WSIM_MODEL", "iw.hub"),
                lean=profile, instance=instance)


def debug_line(frame: int, r) -> str:
    d = r.debug
    p = d.get("p", (0.0, 0.0))
    try:
        ds = " ".join(f"{x:+.3f}" for x in r.dof_positions())
    except Exception as e:  # noqa: BLE001
        ds = f"err:{type(e).__name__}"
    return (f"[wsim] f{frame} {r.name} v={d.get('v', 0.0):+.2f} w={d.get('w', 0.0):+.2f} "
            f"p=({p[0]:.2f},{p[1]:.2f}) z={d.get('z', 0.0):+.4f} pitch={d.get('pitch', 0.0):+.3f} dof=[{ds}]")


def main() -> None:
    cfg = read_config()
    app = sil_app.launch(cfg["lean"], instance=cfg["instance"])

    from sil_isaac.ros2.graph import GraphOptions, Ros2Graph
    from sil_isaac.robot.spawn import spawn_robot
    from sil_isaac.runtime.loop import SimLoop, current_sim_time, play_and_warmup
    from sil_isaac.scene.warehouse import load_warehouse
    from sil_isaac.sensors.raycast_lidar import LidarSpec, create_raycast_lidar
    from sil_isaac.stage import current_stage
    from sil_isaac.viewer.follow_camera import FollowCamera

    load_warehouse(physics_usd=cfg["lean"].physics_usd)
    stage = current_stage()
    model = get_profile(cfg["model"])
    lidar_spec = LidarSpec(z=model.lidar_z)
    robots = []
    for name, xy in zip(cfg["names"], cfg["points"]):
        r = spawn_robot(model, name, xy, stage)
        r.lidar_path = create_raycast_lidar(r.chassis_path, lidar_spec)
        robots.append(r)
    print(f"[wsim] 로봇 {cfg['n']}대 스폰 — "
          f"{'네임스페이스 ' + ', '.join(cfg['names']) if cfg['namespaced'] else '단일(/iw_hub, 평면 토픽)'}; "
          f"에셋 수술(깨진 콜리전 비활성 + 글라이더 + 바퀴 구체 + 헐 박스) 로봇별 적용", flush=True)

    sil_app.setup_physics()
    graphs = [Ros2Graph.build(GraphOptions(r.name, cfg["namespaced"], publish_clock=(i == 0),
                                           lidar_path=r.lidar_path, lidar=lidar_spec)) for i, r in enumerate(robots)]
    play_and_warmup()

    viewer = None
    if cfg["lean"].mjpeg:
        from sil_isaac.viewer.http_stream import HttpViewer
        viewer = HttpViewer(port=app.mjpeg_port)
    camera = FollowCamera()
    topics = (", ".join(f"/{n}/{{cmd_vel,odom,scan}}" for n in cfg["names"]) if cfg["namespaced"]
              else "/scan·/odom·/tf·/clock 발행, /cmd_vel 대기")
    print(f"[wsim] ready — {topics}", flush=True)

    def step(frame: int) -> None:
        t = current_sim_time()
        first = None
        for r, g in zip(robots, graphs):
            v, w = r.drive(*g.read_twist())
            s = r.sample(t)
            r.debug.update(v=v, w=w, p=(s.p[0], s.p[1]), z=float(s.p[2]), pitch=s.pitch)
            g.publish(s)
            first = first if first is not None else s.p
        camera.update(first)

    def log(frame: int) -> None:
        for r in robots:
            print(debug_line(frame, r), flush=True)

    SimLoop(app, step, viewer=viewer, log_every=LOG_EVERY, log_fn=log).run()
    app.close()


if __name__ == "__main__":
    main()
