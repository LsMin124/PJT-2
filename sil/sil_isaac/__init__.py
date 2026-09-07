"""sil_isaac — Isaac Sim 쪽 SIL 공용 라이브러리 (T1 텔레옵 · T2 매핑 · T3 창고 씬이 공유).

실행은 반드시 Isaac python(`~/isaacsim/python.sh`)으로 한다. 순수 계산 모듈(profile, kinematics,
sensors.raycast_lidar.ray_table, ros2.graph.graph_spec, scene.warehouse.spawn_points, app.LeanProfile)은
omni/pxr 없이 import 되므로 시스템 python 의 pytest 로 검증한다. Isaac API 는 함수 안에서 lazy import.
"""

__version__ = "0.1.0"
