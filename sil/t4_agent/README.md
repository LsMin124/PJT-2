# T4 — AMR agent (ROS 2 Humble) · VDA 5050 ↔ PIBT 틱

FMS(PIBT 틱 루프)와 Isaac 로봇 사이의 실행 계층. FMS 파트가 넘긴 요구사항
(스팟턴 · 경유점 오차 ≤ 0.1 m · 틱 배리어 동기 · 후진 1칸)을 VDA 5050 계약 위에서 구현한다.

```
src/amr_agent_msgs   MovePrimitive.action   (TURN | DRIVE, reverse 플래그)
src/amr_agent
  amr_agent/vda5050  topics · order(주문 모델·OrderBook 스티칭 규칙) · state(상태 메시지)  — 순수 파이썬
  amr_agent/control  primitives (TurnController · DriveController, 램프·정지거리)          — 순수 파이썬
  amr_agent/nodes    vda5050_bridge(MQTT↔ROS) · order_executor · primitive_controller · fake_robot
  amr_agent/tools    fake_fms — 배리어 틱으로 계획을 한 노드씩 풀어 주는 가짜 FMS
  launch/agent.launch.py · config/agent.yaml · test/
```

## FMS 틱 ↔ VDA 5050 매핑

| PIBT 액션 | VDA 5050 표현 | 에이전트 동작 |
|---|---|---|
| 전진 1칸 | 다음 노드 released, 엣지 orientation 없음 | 헤딩 정렬(필요 시 스팟턴) → 직진 → 셀 중심 정지 |
| 후진 1칸 | 엣지 `orientation = π`, `orientationType TANGENTIAL` | 헤딩 유지, v < 0 |
| 회전 ±90° | 같은 좌표의 노드 + 새 `theta` (길이 0 엣지) | 제자리 회전 |
| 대기 | 새 노드 없음 | 정지 유지 |
| 틱 완료 ack | `state.lastNodeId` = 방금 released 된 노드 (변화 즉시 발행) | FMS는 전 로봇 ack 후 다음 틱 |

1 틱 = orderUpdateId 를 1 올리며 노드 하나를 더 released 로 보내는 order 업데이트.
업데이트는 마지막 base 노드에서 시작해야 하고(스티칭), 새 orderId 는 base 가 끝난 뒤에만 받는다.
노드의 `allowedDeviationXY / Theta` 가 정지 허용치 상한이다(기본 0.05 m · 0.03 rad).

## 실행

```bash
cd sil/t4_agent
source /opt/ros/humble/setup.bash
colcon build --symlink-install && source install/setup.bash

# 1) 브로커 (EMQX 대용 로컬)
amqtt   # ~/.local/bin, 1883

# 2) 에이전트 + 가짜 로봇 (Isaac 없이)
ros2 launch amr_agent agent.launch.py serial:=amr01 fake_robot:=true

# 3) 가짜 FMS — F 전진 · B 후진 · L/R 회전, 한 노드씩 배리어 틱
python3 -m amr_agent.tools.fake_fms --serial amr01 --plan F,F,L,F,B

# Isaac 상대 (warehouse_sim.py + loc.launch.py 가 떠 있을 때)
ros2 launch amr_agent agent.launch.py serial:=amr01 flat_topics:=true pose_source:=odom use_sim_time:=true
```

단위 테스트(ROS 불필요): `cd src/amr_agent && python3 -m pytest -q test/`

## 남은 것

- Isaac 다중 로봇: warehouse_sim 이 로봇별 네임스페이스(`/amr01/odom` …)로 발행하도록 확장
- ~~앞축 종방향 위치·회전 각속도 실측~~ 완료(9/6, `../calibration/turn_probe/`): 회전 중심 = 구동축 = /odom 원점, 앞 0.40 m·뒤 1.03 m 돌출 → REAR_CELLS 1, 스윙 반경 1.09 m, 90° 폐루프 1.9 s → 1 s 틱 기준 TURN_TICKS 2. **개선 1건**: TurnController가 yaw만 보고 done 판정 → 관성으로 4.9° 초과 사례 1/4, 각속도 정지 조건을 done에 추가할 것
- sim-runner 데몬(`sim/control`)과 에이전트 ×N 기동 연동, 런 아티팩트(리플레이·영상)
- AMCL 폐루프(`pose_source:=amcl`)에서의 정지 정밀도 실측, 오돔 노이즈 주입
