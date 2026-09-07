"""T2 매핑 런 — iw.hub + RTX 2D 라이다, /scan·/odom·/tf·/clock 발행 + 패트롤 주행.

실제 AMR 커미셔닝 절차(설치 시 SLAM 1회 → 지도 고정)의 재현. 지도는 외부 slam_toolbox 가 만든다.
방(벽 4·박스 3)은 좌표를 아는 GT 라서 SLAM 지도와의 오차 비교(V&V)가 가능하다.

실행(서버):
  cd ~/isaacsim && ./python.sh <repo>/sil/apps/mapping_sim.py            # 패트롤(기본)
  cd ~/isaacsim && ./python.sh <repo>/sil/apps/mapping_sim.py --manual   # 텔레옵 조종
SLAM(별도 셸): ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true slam_params_file:=<repo>/sil/t2_mapping/slam_params.yaml
지도 저장: ros2 run nav2_map_server map_saver_cli -f <repo>/sil/t2_mapping/out/map --ros-args -p use_sim_time:=true
"""
import argparse
import math

import _bootstrap  # noqa: F401

from sil_isaac import app as sil_app
from sil_isaac.robot.profile import ASSETS, IWHUB

ENV_USD = ASSETS + "/Isaac/Environments/Grid/default_environment.usd"
SPAWN_Z = 0.05
LIDAR_OFFSET = (0.0, 0.0, 0.60)   # 로봇 몸체 위 — 자기 몸 반사 방지
LOG_EVERY = 1200
# 패트롤: 0.6 m/s 전진 360스텝(3.6 m) + π/8 rad/s 좌회전 240스텝(90°) — 회전 중 스캔 왜곡 완화
FWD_STEPS, TURN_STEPS = 360, 240
FWD_V, TURN_W = 0.6, math.pi / 8


class PatrolSequence:
    """프레임 번호 → (v, w). laps 바퀴 사각 패트롤 뒤 정지."""

    def __init__(self, laps: int) -> None:
        self.cycle = FWD_STEPS + TURN_STEPS
        self.total = laps * 4 * self.cycle
        self.done_announced = False

    def command(self, frame: int):
        if frame < self.total:
            return (FWD_V, 0.0) if (frame % self.cycle) < FWD_STEPS else (0.0, TURN_W)
        if not self.done_announced:
            self.done_announced = True
            print("[t2_mapping] patrol done — 지도 저장 가능", flush=True)
        return (0.0, 0.0)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--manual", action="store_true", help="패트롤 대신 /cmd_vel 텔레옵")
    parser.add_argument("--laps", type=int, default=2, help="패트롤 바퀴 수")
    parser.add_argument("--spawn", type=float, nargs=2, default=[-1.8, -1.8], help="로봇 시작 위치 x y")
    args, _ = parser.parse_known_args()
    return args


def main() -> None:
    args = parse_args()
    app = sil_app.launch(headless=True)
    app.update()

    from sil_isaac.ros2.graph import GraphOptions, Ros2Graph
    from sil_isaac.robot.spawn import find_chassis_link, spawn_robot
    from sil_isaac.runtime.loop import SimLoop, current_sim_time, play_and_warmup
    from sil_isaac.scene.gt_room import MovableObstacle, build_gt_room
    from sil_isaac.sensors.rtx_lidar import create_rtx_lidar
    from sil_isaac.stage import add_reference, current_stage, init_stage
    from sil_isaac.viewer.follow_camera import GRID_OFFSET, FollowCamera

    init_stage()
    add_reference(ENV_USD, "/World/env")
    stage = current_stage()
    build_gt_room(stage)
    obstacle = MovableObstacle(stage)
    robot = spawn_robot(IWHUB, "iw_hub", tuple(args.spawn), fixes=False, spawn_z=SPAWN_Z)
    chassis = find_chassis_link(stage, robot.root)   # 라이다는 움직이는 링크 밑에 (08-27 실증)
    print(f"[t2_mapping] lidar mount: {chassis}/lidar", flush=True)
    create_rtx_lidar(chassis, LIDAR_OFFSET)
    sil_app.setup_physics()
    app.update()
    # /scan 은 RTX writer 가 내고, TF base_link→lidar_link 는 그래프의 동적 TF 노드가 낸다 (원본과 동일)
    graph = Ros2Graph.build(GraphOptions("iw_hub", namespaced=False, publish_clock=True,
                                         extra_twist_subs=(("sub_obs", "obstacle_cmd"),),
                                         extra_tfs=(("tf_lidar", "base_link", "lidar_link", LIDAR_OFFSET),)))
    camera = FollowCamera(offset=GRID_OFFSET)
    camera.look_at((5.0, 5.0, 3.5), (0.0, 0.0, 0.3))
    play_and_warmup()
    patrol = PatrolSequence(args.laps)
    mode = "manual" if args.manual else f"patrol({args.laps}바퀴)"
    print(f"[t2_mapping] ready — 모드 {mode}, /scan·/odom·/tf·/clock 발행 중", flush=True)

    def step(frame: int) -> None:
        cmd = graph.read_twist() if args.manual else patrol.command(frame)
        robot.drive(*cmd)
        s = robot.sample(current_sim_time())
        graph.publish(s)
        obstacle.apply(graph.read_vector("sub_obs"))
        camera.update(s.p)

    SimLoop(app, step, log_every=LOG_EVERY, log_fn=lambda f: print(f"[t2_mapping] frame {f} alive", flush=True)).run()
    app.close()



if __name__ == "__main__":
    main()
