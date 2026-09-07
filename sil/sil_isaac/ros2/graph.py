"""로봇 1대분 ROS 2 I/O 그래프.

graph_spec() 은 omni 없이 (nodes, values, connects) 를 만들고(테스트 가능), Ros2Graph 가 그것을 OmniGraph 에 편집한다.
네임스페이스 모드: 토픽 amr01/odom → /amr01/odom, 프레임 amr01/base_link. /clock 은 인스턴스당 하나만.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

from ..sensors.raycast_lidar import LidarSpec

IDENTITY_QUAT_ROS = [0.0, 0.0, 0.0, 1.0]


@dataclass(frozen=True)
class GraphOptions:
    name: str                       # 로봇 이름 (amr01) — 그래프 경로·네임스페이스에 쓰임
    namespaced: bool = False
    publish_clock: bool = True
    lidar_path: Optional[str] = None
    lidar: Optional[LidarSpec] = None
    extra_twist_subs: Tuple[Tuple[str, str], ...] = ()   # (노드명, 토픽) — T2 의 obstacle_cmd 같은 추가 구독
    extra_tfs: Tuple[Tuple[str, str, str, Tuple[float, float, float]], ...] = ()  # (노드명, parent, child, xyz) 동적 TF

    @property
    def ns(self) -> str:
        return f"{self.name}/" if self.namespaced else ""

    @property
    def graph_path(self) -> str:
        return f"/World/ros2_graph_{self.name}" if self.namespaced else "/World/ros2_graph"

    def frame(self, base: str) -> str:
        return f"{self.ns}{base}"


def graph_spec(o: GraphOptions) -> Tuple[List[tuple], List[tuple], List[tuple]]:
    """OmniGraph 편집 명세. 순수 함수 — 노드 이름·토픽·프레임 규약이 여기 한 곳에 있다."""
    ns = o.ns
    f_odom, f_base = o.frame("odom"), o.frame("base_link")
    nodes = [("tick", "omni.graph.action.OnPlaybackTick"), ("ctx", "isaacsim.ros2.bridge.ROS2Context"),
             ("sub_twist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
             ("pub_odom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
             ("tf_odom", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree")]
    values = [("sub_twist.inputs:topicName", f"{ns}cmd_vel"),
              ("pub_odom.inputs:topicName", f"{ns}odom"), ("pub_odom.inputs:odomFrameId", f_odom),
              ("pub_odom.inputs:chassisFrameId", f_base), ("pub_odom.inputs:publishRawVelocities", True),
              ("tf_odom.inputs:parentFrameId", f_odom), ("tf_odom.inputs:childFrameId", f_base)]
    connects = [("tick.outputs:tick", "sub_twist.inputs:execIn"), ("tick.outputs:tick", "pub_odom.inputs:execIn"),
                ("tick.outputs:tick", "tf_odom.inputs:execIn"), ("ctx.outputs:context", "sub_twist.inputs:context"),
                ("ctx.outputs:context", "pub_odom.inputs:context"), ("ctx.outputs:context", "tf_odom.inputs:context")]
    if o.publish_clock:
        nodes.append(("pub_clock", "isaacsim.ros2.bridge.ROS2PublishClock"))
        values.append(("pub_clock.inputs:topicName", "clock"))
        connects += [("tick.outputs:tick", "pub_clock.inputs:execIn"), ("ctx.outputs:context", "pub_clock.inputs:context")]
    if o.lidar_path and o.lidar:
        s = o.lidar
        f_laser = o.frame(s.frame_id)
        nodes += [("read_lidar", "isaacsim.sensors.physics.IsaacReadRaycastSensor"),
                  ("pub_scan", "isaacsim.ros2.bridge.ROS2PublishLaserScan"),
                  ("tf_laser", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree")]
        values += [("read_lidar.inputs:raycastSensorPrim", ("SDF_PATH", o.lidar_path)),
                   ("pub_scan.inputs:topicName", f"{ns}scan"), ("pub_scan.inputs:frameId", f_laser),
                   ("pub_scan.inputs:horizontalFov", 360.0), ("pub_scan.inputs:horizontalResolution", s.h_res_deg),
                   ("pub_scan.inputs:numCols", s.n_rays), ("pub_scan.inputs:numRows", 1),
                   ("pub_scan.inputs:depthRange", [s.r_min, s.r_max]), ("pub_scan.inputs:rotationRate", 0.0),
                   ("pub_scan.inputs:azimuthRange", list(s.azimuth_range)),
                   ("tf_laser.inputs:parentFrameId", f_base), ("tf_laser.inputs:childFrameId", f_laser),
                   # staticPublisher=True 는 스택 재시작 시 구독자에 전달 안 됨(실측 — tf2 에 laser 프레임 부재로
                   # AMCL 스캔 전량 드롭) → 동적 60Hz 발행
                   ("tf_laser.inputs:translation", [0.0, 0.0, s.z]), ("tf_laser.inputs:rotation", IDENTITY_QUAT_ROS)]
        connects += [("tick.outputs:tick", "read_lidar.inputs:execIn"), ("tick.outputs:tick", "tf_laser.inputs:execIn"),
                     ("read_lidar.outputs:execOut", "pub_scan.inputs:execIn"),
                     ("read_lidar.outputs:depths", "pub_scan.inputs:linearDepthData"),
                     ("ctx.outputs:context", "pub_scan.inputs:context"), ("ctx.outputs:context", "tf_laser.inputs:context")]
    for node_name, topic in o.extra_twist_subs:
        nodes.append((node_name, "isaacsim.ros2.bridge.ROS2SubscribeTwist"))
        values.append((f"{node_name}.inputs:topicName", topic))
        connects += [("tick.outputs:tick", f"{node_name}.inputs:execIn"), ("ctx.outputs:context", f"{node_name}.inputs:context")]
    for node_name, parent, child, xyz in o.extra_tfs:
        nodes.append((node_name, "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"))
        values += [(f"{node_name}.inputs:parentFrameId", o.frame(parent)), (f"{node_name}.inputs:childFrameId", o.frame(child)),
                   (f"{node_name}.inputs:translation", [float(v) for v in xyz]), (f"{node_name}.inputs:rotation", IDENTITY_QUAT_ROS)]
        connects += [("tick.outputs:tick", f"{node_name}.inputs:execIn"), ("ctx.outputs:context", f"{node_name}.inputs:context")]
    return nodes, values, connects


@dataclass
class Ros2Graph:
    """편집된 그래프와 매 프레임 쓰는 어트리뷰트 핸들."""
    options: GraphOptions
    attrs: Dict[str, Any] = field(default_factory=dict)

    @classmethod
    def build(cls, options: GraphOptions) -> "Ros2Graph":
        import omni.graph.core as og
        import usdrt

        nodes, values, connects = graph_spec(options)
        values = [(k, usdrt.Sdf.Path(v[1]) if isinstance(v, tuple) and v and v[0] == "SDF_PATH" else v) for k, v in values]
        values = [(k, [v] if isinstance(v, usdrt.Sdf.Path) else v) for k, v in values]
        og.Controller.edit({"graph_path": options.graph_path, "evaluator_name": "execution"},
                           {og.Controller.Keys.CREATE_NODES: nodes, og.Controller.Keys.SET_VALUES: values,
                            og.Controller.Keys.CONNECT: connects})
        g = cls(options)
        a = lambda p: og.Controller.attribute(f"{options.graph_path}/{p}")  # noqa: E731
        g.attrs = dict(twist_lin=a("sub_twist.outputs:linearVelocity"), twist_ang=a("sub_twist.outputs:angularVelocity"),
                       odom_ts=a("pub_odom.inputs:timeStamp"), odom_pos=a("pub_odom.inputs:position"),
                       odom_ori=a("pub_odom.inputs:orientation"), odom_lin=a("pub_odom.inputs:linearVelocity"),
                       odom_ang=a("pub_odom.inputs:angularVelocity"),
                       tfo_ts=a("tf_odom.inputs:timeStamp"), tfo_tr=a("tf_odom.inputs:translation"), tfo_rot=a("tf_odom.inputs:rotation"))
        if options.publish_clock:
            g.attrs["clock_ts"] = a("pub_clock.inputs:timeStamp")
        if options.lidar_path and options.lidar:
            g.attrs["scan_ts"] = a("pub_scan.inputs:timeStamp")
            g.attrs["tfl_ts"] = a("tf_laser.inputs:timeStamp")
        for node_name, _ in options.extra_twist_subs:
            g.attrs[f"{node_name}_lin"] = a(f"{node_name}.outputs:linearVelocity")
            g.attrs[f"{node_name}_ang"] = a(f"{node_name}.outputs:angularVelocity")
        for node_name, *_ in options.extra_tfs:
            g.attrs[f"{node_name}_ts"] = a(f"{node_name}.inputs:timeStamp")
        return g

    def read_twist(self, node: str = "sub_twist") -> Tuple[float, float]:
        """구독 중인 Twist 의 (linear.x, angular.z)."""
        import numpy as np
        import omni.graph.core as og

        key = "twist" if node == "sub_twist" else node
        lin = np.asarray(og.Controller.get(self.attrs[f"{key}_lin"])).reshape(-1)
        ang = np.asarray(og.Controller.get(self.attrs[f"{key}_ang"])).reshape(-1)
        return float(lin[0]), float(ang[2])

    def read_vector(self, node: str) -> Sequence[float]:
        import numpy as np
        import omni.graph.core as og

        return np.asarray(og.Controller.get(self.attrs[f"{node}_lin"])).reshape(-1)

    def publish(self, sample) -> None:
        """PoseSample → clock·odom·tf 어트리뷰트에 쓴다 (실제 발행은 그래프 tick 이 한다)."""
        import omni.graph.core as og

        t = float(sample.t)
        for key, attr in self.attrs.items():
            if key.endswith("_ts"):
                og.Controller.set(attr, t)
        p = [float(sample.p[0]), float(sample.p[1]), float(sample.p[2])]
        q = sample.q_ros
        og.Controller.set(self.attrs["odom_pos"], p)
        og.Controller.set(self.attrs["odom_ori"], q)
        og.Controller.set(self.attrs["odom_lin"], [float(v) for v in sample.lv[:3]])
        og.Controller.set(self.attrs["odom_ang"], [float(v) for v in sample.av[:3]])
        og.Controller.set(self.attrs["tfo_tr"], p)
        og.Controller.set(self.attrs["tfo_rot"], q)
