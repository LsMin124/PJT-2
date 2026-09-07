"""sil_isaac 의 순수 계산부 — Isaac 없이 시스템 python 으로 검증."""
import math

import numpy as np
import pytest

from sil_isaac.app import LeanProfile, lean_profile
from sil_isaac.robot.kinematics import PoseTracker, pitch_from_wxyz, wxyz_to_ros, yaw_from_wxyz
from sil_isaac.robot.profile import IWHUB, get_profile
from sil_isaac.ros2.graph import GraphOptions, graph_spec
from sil_isaac.scene.warehouse import parse_spawn_list, robot_names, spawn_points
from sil_isaac.sensors.raycast_lidar import LidarSpec, ray_table


def test_iwhub_profile_matches_measured_constants():
    assert IWHUB.wheel_radius == pytest.approx(0.08)
    assert IWHUB.track == pytest.approx(0.58)
    assert IWHUB.wheel_dof_names == ("left_wheel_joint", "right_wheel_joint")
    assert get_profile("IW.HUB") is IWHUB
    with pytest.raises(KeyError):
        get_profile("nova")


def test_lidar_ray_table_is_720_unit_vectors_in_xy_plane():
    spec = LidarSpec()
    origins, dirs = ray_table(spec)
    assert spec.n_rays == 720 and len(origins) == len(dirs) == 720
    d = np.asarray(dirs)
    assert np.allclose(np.linalg.norm(d, axis=1), 1.0) and np.all(d[:, 2] == 0)
    assert d[0] == pytest.approx([-1.0, 0.0, 0.0], abs=1e-9)          # -180°
    assert d[180] == pytest.approx([0.0, -1.0, 0.0], abs=1e-9)        # -90°
    assert d[360] == pytest.approx([1.0, 0.0, 0.0], abs=1e-9)         # 0°


def test_graph_spec_flat_single_robot_has_clock_and_lidar():
    nodes, values, connects = graph_spec(GraphOptions("iw_hub", namespaced=False, publish_clock=True,
                                                      lidar_path="/World/iw_hub/chassis/lidar", lidar=LidarSpec()))
    names = {n for n, _ in nodes}
    assert {"tick", "ctx", "sub_twist", "pub_odom", "tf_odom", "pub_clock", "read_lidar", "pub_scan", "tf_laser"} <= names
    v = dict(values)
    assert v["sub_twist.inputs:topicName"] == "cmd_vel"
    assert v["pub_scan.inputs:frameId"] == "laser" and v["tf_laser.inputs:translation"] == [0.0, 0.0, 0.45]
    assert v["pub_scan.inputs:numCols"] == 720
    assert ("read_lidar.outputs:depths", "pub_scan.inputs:linearDepthData") in connects


def test_graph_spec_namespaced_robot_prefixes_topics_and_frames_and_skips_clock():
    _, values, _ = graph_spec(GraphOptions("amr02", namespaced=True, publish_clock=False,
                                           lidar_path="/World/amr02/chassis/lidar", lidar=LidarSpec()))
    v = dict(values)
    assert v["sub_twist.inputs:topicName"] == "amr02/cmd_vel"
    assert v["pub_odom.inputs:odomFrameId"] == "amr02/odom" and v["tf_laser.inputs:childFrameId"] == "amr02/laser"
    assert "pub_clock.inputs:topicName" not in v


def test_graph_spec_extra_twist_subscriber():
    nodes, values, _ = graph_spec(GraphOptions("iw_hub", extra_twist_subs=(("sub_obs", "obstacle_cmd"),)))
    assert ("sub_obs", "isaacsim.ros2.bridge.ROS2SubscribeTwist") in nodes
    assert ("sub_obs.inputs:topicName", "obstacle_cmd") in values


def test_graph_spec_extra_dynamic_tf():
    nodes, values, connects = graph_spec(GraphOptions("iw_hub", extra_tfs=(("tf_lidar", "base_link", "lidar_link", (0, 0, 0.6)),)))
    assert ("tf_lidar", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree") in nodes
    v = dict(values)
    assert v["tf_lidar.inputs:childFrameId"] == "lidar_link" and v["tf_lidar.inputs:translation"] == [0.0, 0.0, 0.6]
    assert ("tick.outputs:tick", "tf_lidar.inputs:execIn") in connects


def test_spawn_points_and_names():
    assert spawn_points(1) == [(34.0, 40.8)]
    assert spawn_points(3) == [(36.0, 40.8), (39.0, 40.8), (42.0, 40.8)]
    assert spawn_points(2, explicit=parse_spawn_list("1,2;3,4")) == [(1.0, 2.0), (3.0, 4.0)]
    with pytest.raises(ValueError):
        spawn_points(3, explicit=[(0, 0)])
    assert robot_names(2, True) == ["amr01", "amr02"] and robot_names(1, False) == ["iw_hub"]


def test_quaternion_helpers():
    q = (math.cos(math.pi / 8), 0.0, 0.0, math.sin(math.pi / 8))   # yaw 45° (w,x,y,z)
    assert yaw_from_wxyz(q) == pytest.approx(math.pi / 4)
    assert pitch_from_wxyz(q) == pytest.approx(0.0)
    assert wxyz_to_ros(q) == [0.0, 0.0, q[3], q[0]]


def test_pose_tracker_finite_difference():
    tr = PoseTracker()
    q0 = np.array([1.0, 0.0, 0.0, 0.0])
    lv, av = tr.finite_difference(0.0, np.zeros(3), q0)
    assert np.all(lv == 0) and np.all(av == 0)
    lv, av = tr.finite_difference(0.5, np.array([1.0, 0.0, 0.0]), q0)
    assert lv == pytest.approx([2.0, 0.0, 0.0]) and av[2] == pytest.approx(0.0)


def test_lean_profiles_follow_measurement_matrix():
    a, c, e, g = (lean_profile(n) for n in "aceg")
    assert a == LeanProfile() and a.launch_config()["window_width"] == 1280
    assert c.rtx_off and not c.livestream and c.viewport_updates
    assert not e.viewport_updates and not e.physics_cpu
    assert g.physics_cpu and g.launch_config()["physics_gpu"] == -1
    assert any("memoryBudget=0.05" in x for x in e.kit_args()) and not any("rtx" in x for x in a.kit_args())
    assert "signalPort=49103" in lean_profile("b").kit_args(instance=3)[0]
    assert lean_profile("d", physics_usd="phys.usda").physics_usd == "phys.usda"
    assert lean_profile("c", physics_usd="phys.usda").physics_usd is None     # c 는 스테이지 치환 없음
    with pytest.raises(ValueError):
        lean_profile("z")
