"""T3 localization 런치 — map_server + AMCL + lifecycle + foxglove_bridge.

warehouse_sim.py(Isaac, /scan·/odom·/tf·/clock)가 떠 있는 상태에서:
  source /opt/ros/humble/setup.bash
  ros2 launch <repo>/sil/t3_warehouse/ros2/loc.launch.py
시각화: 브라우저 https://app.foxglove.dev → Open connection →
  ws://<서버IP>:8765  (웹소켓 = TCP — UDP 차단망 통과)
"""

from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node

HERE = Path(__file__).resolve().parent
MAP_YAML = str((HERE / ".." / ".." / "t3_warehouse_map" / "map" / "map.yaml").resolve())
PARAMS = str(HERE / "amcl_params.yaml")


def generate_launch_description():
    return LaunchDescription([
        Node(package="nav2_map_server", executable="map_server", name="map_server",
             output="screen",
             parameters=[PARAMS, {"yaml_filename": MAP_YAML}]),
        Node(package="nav2_amcl", executable="amcl", name="amcl",
             output="screen", parameters=[PARAMS]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_loc", output="screen",
             parameters=[{"use_sim_time": True, "autostart": True,
                          "node_names": ["map_server", "amcl"]}]),
        Node(package="foxglove_bridge", executable="foxglove_bridge",
             output="screen", parameters=[PARAMS]),
    ])
