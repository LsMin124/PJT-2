"""One AMR agent: VDA 5050 bridge + order executor + primitive controller (+ optional fake robot).

  ros2 launch amr_agent agent.launch.py serial:=amr01 mqtt_host:=localhost fake_robot:=true
  # against Isaac (warehouse_sim publishes /odom, /cmd_vel without namespace):
  ros2 launch amr_agent agent.launch.py serial:=amr01 flat_topics:=true pose_source:=odom
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context, *_args, **_kwargs):
    cfg = {k: LaunchConfiguration(k).perform(context) for k in (
        "serial", "manufacturer", "map_id", "mqtt_host", "mqtt_port", "pose_source",
        "fake_robot", "flat_topics", "fake_x", "fake_y", "fake_yaw", "use_sim_time")}
    ns = cfg["serial"]
    flat = cfg["flat_topics"].lower() in ("1", "true", "yes")
    remaps = [("odom", "/odom"), ("cmd_vel", "/cmd_vel"), ("amcl_pose", "/amcl_pose")] if flat else []
    common = {"serial": cfg["serial"], "manufacturer": cfg["manufacturer"],
              "use_sim_time": cfg["use_sim_time"].lower() in ("1", "true", "yes")}
    nodes = [
        Node(package="amr_agent", executable="vda5050_bridge", namespace=ns, output="screen",
             parameters=[common, {"mqtt_host": cfg["mqtt_host"], "mqtt_port": int(cfg["mqtt_port"])}]),
        Node(package="amr_agent", executable="order_executor", namespace=ns, output="screen",
             parameters=[common, {"map_id": cfg["map_id"], "pose_source": cfg["pose_source"]}],
             remappings=remaps),
        Node(package="amr_agent", executable="primitive_controller", namespace=ns, output="screen",
             parameters=[common, {"pose_source": cfg["pose_source"]}], remappings=remaps),
    ]
    if cfg["fake_robot"].lower() in ("1", "true", "yes"):
        nodes.append(Node(package="amr_agent", executable="fake_robot", namespace=ns, output="screen",
                          parameters=[{"x": float(cfg["fake_x"]), "y": float(cfg["fake_y"]),
                                       "yaw": float(cfg["fake_yaw"])}], remappings=remaps))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("serial", default_value="amr01"),
        DeclareLaunchArgument("manufacturer", default_value="santa"),
        DeclareLaunchArgument("map_id", default_value="wallA"),
        DeclareLaunchArgument("mqtt_host", default_value="localhost"),
        DeclareLaunchArgument("mqtt_port", default_value="1883"),
        DeclareLaunchArgument("pose_source", default_value="odom", description="odom | amcl"),
        DeclareLaunchArgument("fake_robot", default_value="false"),
        DeclareLaunchArgument("flat_topics", default_value="false",
                              description="remap odom/cmd_vel/amcl_pose to the global topics Isaac uses"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("fake_x", default_value="0.0"),
        DeclareLaunchArgument("fake_y", default_value="0.0"),
        DeclareLaunchArgument("fake_yaw", default_value="0.0"),
        OpaqueFunction(function=_nodes),
    ])
