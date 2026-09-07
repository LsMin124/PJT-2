"""RTX 회전 라이다 (T2 매핑용) — 렌더 기하 기준이라 투명 콜라이더 씬(T3)에는 부적합. slam_toolbox 입력."""
from __future__ import annotations

from typing import Sequence

DEFAULT_CONFIG = "Example_Rotary_2D"
DEFAULT_TICK_HZ = 10.0


def laser_scan_meta(prim) -> dict:
    """라이다 프림 메타 → RtxLidarROS2PublishLaserScan 인자."""
    rotation_rate = float(prim.GetAttribute("omni:sensor:Core:scanRateBaseHz").Get() or 0)
    near_range = float(prim.GetAttribute("omni:sensor:Core:nearRangeM").Get() or 0)
    far_range = float(prim.GetAttribute("omni:sensor:Core:farRangeM").Get() or 0)
    firing_rate = int(prim.GetAttribute("omni:sensor:Core:patternFiringRateHz").Get() or 0)
    if rotation_rate <= 0 or firing_rate <= 0:
        raise RuntimeError("lidar prim scan-rate metadata missing")
    return {"horizontalFov": 360.0, "horizontalResolution": 360.0 * rotation_rate / firing_rate,
            "depthRange": [near_range, far_range], "rotationRate": rotation_rate, "azimuthRange": [-180.0, 180.0]}


def create_rtx_lidar(link_path: str, offset: Sequence[float], topic: str = "scan", frame_id: str = "lidar_link",
                     config: str = DEFAULT_CONFIG, tick_hz: float = DEFAULT_TICK_HZ):
    """link_path 밑에 RTX 2D 라이다를 만들고 LaserScan ROS2 writer 를 붙인다. (lidar, sensor) 반환."""
    import isaacsim.core.experimental.utils.prim as prim_utils
    from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor

    lidar = Lidar.create(path=f"{link_path}/lidar", config=config, tick_rate=tick_hz, translations=[list(offset)])
    sensor = LidarSensor(lidar, annotators=[])
    sensor.attach_writer("RtxLidarROS2PublishLaserScan", topicName=topic, frameId=frame_id,
                         **laser_scan_meta(prim_utils.get_prim_at_path(lidar.paths[0])))
    return lidar, sensor
