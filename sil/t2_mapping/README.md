# T2 스테이지 A — SLAM 매핑 런 (커미셔닝 재현)

실제 AMR 설치 절차(현장 SLAM 1회 → 지도 고정)의 재현. GT 좌표를 아는 방(벽 4·박스 3)을 직접 만들어
SLAM 지도와의 오차를 정량 비교했다 — V&V 자료.

## 구성

| 파일 | 역할 |
|---|---|
| `mapping_sim.py` | Isaac 쪽 — GT 방 + iw.hub + 2D RTX 라이다(체시스 링크 부착), /scan·/odom·/tf·/clock 발행, 사각 패트롤 |
| `slam_params.yaml` | slam_toolbox 파라미터 (프레임: base_link/odom/map) |
| `out/map.pgm·yaml` | 산출 지도 (5cm 해상도) |
| `out/map_vs_gt.png` | 검증 오버레이 — 검정=SLAM 점유, 빨강=미매핑 GT 표면 |

## 실행 (검증된 절차, 2026-08-27)

```bash
# 1) 시뮬 (패트롤 3바퀴 자동 주행)
source /opt/ros/humble/setup.bash
cd ~/isaacsim && ./python.sh <repo>/sil/t2_mapping/mapping_sim.py --laps 3

# 2) SLAM (별도 셸, ready 후)
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true \
    slam_params_file:=<repo>/sil/t2_mapping/slam_params.yaml

# 3) "patrol done" 로그 후 지도 저장
ros2 run nav2_map_server map_saver_cli -f <repo>/sil/t2_mapping/out/map --ros-args -p use_sim_time:=true
```

## 결과 — 실험 3회의 교훈이 곧 수확

| 런 | 조건 | 평균 오차 | 커버리지 | 비고 |
|---|---|---|---|---|
| 2 | 구석 시작·π/4 회전·2바퀴 | 1.5cm | 41% | 정밀하나 시야 부족 |
| 3 | 중앙 시작·π/4 회전·3바퀴 | 18.1cm | 81% | 지도 ~2° 회전 바이어스 + 스미어 |
| **4 (최종)** | **중앙 시작·π/8 회전·3바퀴** | **3.4cm (95% 7.5cm)** | **91.6%** | 전 점유셀 10cm 이내, 지도 범위 = 방 정확 일치 |

## 핵심 발견 (환류 지식)

1. **센서는 반드시 움직이는 링크에** — articulation 루트 Xform에 붙이면 물리는 링크만 움직여
   센서가 스폰 위치에 고정된다. 증상: 지도 전체가 회전·오프셋. 진단: 원점 고정 가설의 GT 레이캐스트 잔차 0.2cm
2. **회전 중 스캔 왜곡이 정밀도를 지배** — 10Hz 회전 라이다가 π/4 rad/s로 도는 동안 스캔당 4.5° 스미어
   (slam_toolbox는 순간 스캔 가정). 회전 속도 절반(π/8)으로 평균 오차 18cm → 3.4cm.
   → 실전 시사점: 매핑 주행은 저속 회전으로, T4 다중 로봇의 교차로 회전 속도도 지도 품질 아닌 별도 파라미터로 관리
3. **스캔 자체는 mm급** — 정지 상태 GT 레이캐스트 대비 잔차 0.2cm. 오차의 원천은 센서가 아니라 운동·매칭

## 남은 것 (T2 스테이지 B)

지도 고정 → 노드-엣지 경로망 정의 → 추종기 노드(Pure Pursuit + 가속 램프 + 안전 필드 정지) → 엣지 통과시간 분포 실측(DES 환류)
