"""T1 Teleop — 격자 환경에 iw.hub 1대, /cmd_vel 구독 구동 + /clock·/odom 발행 (브리지 개통).

실행(서버): source /opt/ros/humble/setup.bash && cd ~/isaacsim && ./python.sh <repo>/sil/apps/teleop_sim.py
조종(별도 셸): ros2 run teleop_twist_keyboard teleop_twist_keyboard
관전: WebRTC 클라이언트 → 서버 IP
운동학 상수는 sil_isaac.robot.profile.IWHUB (반경 0.08 · 트랙 0.58 실측값). 옛 0.115/0.413 은 오류였고,
그때의 '0.835 m/s 상한'은 잘못된 반경으로 낸 지령을 실제 반경으로 굴린 결과(1.2 × 0.08/0.115)다.
"""
import _bootstrap  # noqa: F401

from sil_isaac import app as sil_app
from sil_isaac.robot.profile import ASSETS, IWHUB

ENV_USD = ASSETS + "/Isaac/Environments/Grid/default_environment.usd"
SPAWN_Z = 0.05
LOG_EVERY = 600


def main() -> None:
    app = sil_app.launch(headless=True)

    from sil_isaac.ros2.graph import GraphOptions, Ros2Graph
    from sil_isaac.robot.spawn import spawn_robot
    from sil_isaac.runtime.loop import SimLoop, current_sim_time, play_and_warmup
    from sil_isaac.stage import add_reference, init_stage
    from sil_isaac.viewer.follow_camera import GRID_OFFSET, FollowCamera

    init_stage()
    add_reference(ENV_USD, "/World/env")
    robot = spawn_robot(IWHUB, "iw_hub", (0.0, 0.0), fixes=False, spawn_z=SPAWN_Z)
    sil_app.setup_physics()
    graph = Ros2Graph.build(GraphOptions("iw_hub", namespaced=False, publish_clock=True))
    camera = FollowCamera(offset=GRID_OFFSET)
    camera.look_at((5.0, 5.0, 3.5), (0.0, 0.0, 0.3))
    play_and_warmup()
    print("[t1_teleop] ready — /cmd_vel 대기, /clock·/odom 발행 중", flush=True)

    def step(frame: int) -> None:
        robot.drive(*graph.read_twist())
        s = robot.sample(current_sim_time())
        graph.publish(s)
        camera.update(s.p)

    SimLoop(app, step, log_every=LOG_EVERY, log_fn=lambda f: print(f"[t1_teleop] frame {f} alive", flush=True)).run()
    app.close()


if __name__ == "__main__":
    main()
