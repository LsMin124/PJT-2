# T1 — Teleop (브리지 개통)

iw.hub 1대를 간단 씬(Grid)에 띄우고, ROS2 `/cmd_vel`로 구동하며 `/clock`·`/odom`을 발행한다.
기존 데모(스크립트 내장 주행)를 ROS2 토픽 제어로 교체한 것 — SIL 로드맵 T1.

## 실행 (검증된 절차, 2026-08-27)

```bash
# 1) 시뮬 (홈서버) — Humble 소싱 후 Isaac 기동
source /opt/ros/humble/setup.bash
cd ~/isaacsim && ./python.sh ~/workspace/PJT-2/sil/t1_teleop/teleop_sim.py
# "[t1_teleop] ready" 로그 후 조작 가능 (부팅 ~80초)

# 2) 키보드 조종 (서버 SSH 셸 어디서든)
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 3) 관전 — Isaac Sim WebRTC Streaming Client → 서버 IP (tailscale 100.89.12.112)
```

- Isaac은 호스트당 동시 1인스턴스 (OOM 실측 사고 기록)
- teleop_twist_keyboard는 키를 누를 때만 발행 — 정지는 `k` 또는 스페이스
- 스크립트 안전 클램프: 선속 1.2 m/s · 각속 1.5 rad/s

## 검증 결과 (2026-08-27 실측)

| 항목 | 결과 |
|---|---|
| 토픽 | /cmd_vel(구독) · /clock 185Hz · /odom 200Hz |
| 직진 | 0.8 m/s 명령 → 5.37m 주행, 횡편차 9mm |
| 회전 | +z 명령 → 좌회전(CCW), 쿼터니언 xyzw 변환 정확 |
| 정지 | 제로 트위스트 1발로 정지 |

## DES 환류 — 가감속 실측 (`out/accel_step_max.csv`)

측정: `measure_accel.py`(시스템 Humble python)로 /odom 기록 + 스텝 명령. 타임스탬프는 시뮬 시간이라 RTF 무관.

| 항목 | 실측값 | 시사점 |
|---|---|---|
| 최고 속도 | **0.836 m/s** (명령 1.2 클램프에도 불구) | 에셋 휠 조인트 한계 추정(휠 7.27 rad/s). 공식 스펙과의 괴리 — 보정 루프 의제 |
| 가속 0→95% | 0.15 시뮬s (~5.3 m/s²) | 즉답형 응답 — 물리에 가속 램프 없음 |
| 감속 →0 | 0.12 시뮬s (~5.7 m/s²) | 동일 |

**핵심 발견**: 기본 물리 응답이 즉답형이라, 실제 iw.hub의 가감속(통상 ≤1 m/s² 급)은
물리가 아니라 **명령 램프(컨트롤러 수준 가속 제한)**로 모델링해야 한다.
→ DES의 가감속 파라미터는 "물리 측정값"이 아니라 "램프 설정값"이고, T2부터 추종기에 램프를 넣는다.

## 파일

- `teleop_sim.py` — Isaac 쪽: 검증된 WheeledRobot 구동 경로 + OmniGraph ROS I/O(SubscribeTwist·PublishClock·PublishOdometry)
- `measure_accel.py` — 외부 rclpy /odom 로거 → CSV + 요약
- `out/` — 실측 결과

## 운영 메모

- 백그라운드 기동은 하네스 추적 실행으로 — `setsid nohup &` 분리 기동은 세션 정리 시 함께 죽는 것 확인(08-27)
- 브리지 기동 시 `[humble.rclpy] Could not import rclpy` 경고는 무해(파이썬 3.12 vs Humble 3.10) — 브리지는 C++ 레벨로 동작
- livestream `StreamSdk ... INVALID_STATE` Fatal 로그는 클라이언트 미접속 상태의 소음 — 앱은 정상 지속
