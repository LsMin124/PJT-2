"""sil_ros — sil/ 의 ROS 2 쪽(시스템 python3 + rclpy, Isaac 불필요) 스크립트를 재사용 가능한 패키지로 묶은 것.

  geometry    순수 기하(yaw_of · wrap · ang_diff · Pose2D)            — 표준 math 만
  gridmap     팽창 장애물 그리드 + 스테이션 + A* 플래너(GridMap)       — numpy 만
  twist_ramp  가속 램프 · 사각형 안전 필드 · 히스테리시스 등 순수 제어 계산 — 표준 math 만
  nodes/      rclpy 노드: patrol · multi_check · measure_accel · follower · spin_probe (각 main())
  tools/      export_replay (맵 → isaac_replay 재생 JSON)

실행은 원래 경로의 호환 셔ム(예: sil/t3_warehouse/ros2/patrol.py)을 그대로 쓰거나, sil/ 에서
  source /opt/ros/humble/setup.bash && python3 -m sil_ros.nodes.patrol
맵 디렉토리 기본값은 sil/t3_warehouse_map/map — 인자(--map-dir) 또는 환경변수 SIL_MAP_DIR 로 변경.
검증: cd sil && python3 -m pytest -q tests/test_sil_ros_*.py   (rclpy 불필요, numpy 만)
"""
import os

SIL_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # …/sil
