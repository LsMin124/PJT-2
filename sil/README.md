# sil/ — Isaac Sim · ROS 2 구현 트랙 (SIL, 가상 시운전)

정본은 팀 레포 `S15P21A106/2_Simulation/`이고 여기는 작업 사본이다. 2026-09-07 리팩토링(모듈 분할)은 이 사본에서 먼저 했고,
2026-09-08 팀 레포(feature/S15P21A106-171)로 이식했다 — 이후 변경은 팀 레포에 먼저 넣는다. 실행 환경: 홈 GPU 서버(RTX 5080 16 GB · RAM 64 GB), Isaac Sim 6.0.1 `~/isaacsim` + `/opt/ros/humble`.

## 구조 (2026-09-07 리팩토링)

```
sil/
├─ apps/            실행 엔트리 (Isaac python.sh). 얇다 — 설정 읽고 라이브러리를 조립만 한다
│   warehouse_sim.py  T3 창고 씬 + iw.hub ×N + 물리 라이다 + ROS 2   (WSIM_N · WSIM_LEAN · WSIM_INST …)
│   teleop_sim.py     T1 격자 환경 텔레옵            mapping_sim.py  T2 GT 방 + RTX 라이다 + 패트롤
│   view_scene.py     T3 씬 관전                     build_scene.py  그리드 → USD 씬 빌더
├─ sil_isaac/       Isaac 쪽 공용 라이브러리 (omni/pxr 는 함수 안에서 lazy import → 순수부는 시스템 python 으로 테스트)
│   app.py            SimulationApp 기동 · 확장 · 물리 · 경량 프로파일 a~g(LeanProfile) · 인스턴스 포트
│   stage.py          스테이지 초기화 · 참조 로드(물리 전용 스테이지 치환)
│   robot/profile.py  RobotProfile(기종 하나의 숫자 전부) · IWHUB 실측값     robot/iwhub.py  에셋 런타임 수술 3단계
│   robot/spawn.py    spawn_robot → RobotHandle(drive · sample · clamp)     robot/kinematics.py  포즈·속도·쿼터니언
│   sensors/raycast_lidar.py  LidarSpec · ray_table · 물리 레이캐스트 라이다   sensors/rtx_lidar.py  RTX 회전 라이다(T2)
│   ros2/graph.py     graph_spec(순수 명세) · Ros2Graph(편집·핸들·publish)  — 네임스페이스·/clock 규칙이 여기 한 곳
│   viewer/           http_stream.py(TCP MJPEG) · follow_camera.py
│   scene/            warehouse.py(씬 로드·스폰 좌표) · gt_room.py(T2 GT 방·장애물)
│   runtime/loop.py   SimLoop — 재생/리셋 처리 · 프레임 콜백 · 주기 로그
├─ sil_ros/         ROS 2 쪽 (시스템 python3 + rclpy). geometry · gridmap(A*) 공용 + nodes/(patrol · multi_check · follower · measure_accel · spin_probe) + tools/export_replay
├─ scene_builder/   build_scene.py 분할 패키지 (config · prims · materials · building · roof · racks · stations · offices · vnv · shots · build)
├─ tests/           pytest — Isaac·rclpy 없이 도는 순수부 (sil_isaac 명세·운동학·스폰 좌표·경량 프로파일, sil_ros gridmap, scene_builder gridutil)
├─ t4_agent/        ROS 2 에이전트 패키지 (VDA 5050 브리지 · order executor · 그리드 프리미티브) — 구조 변경 없음
├─ t1_teleop/ t2_mapping/ t2_follower/ t3_warehouse/  옛 경로: 실행 셔임 + 데이터(USD·YAML·out)   ← 문서·README 의 경로가 그대로 동작
├─ t3_warehouse_map/  DXF → occupancy grid 파이프라인 (단일 소스 map/)
└─ calibration/ tools/  스팟턴 실측(wsim_wrap 은 새 엔트리에 훅) · 에셋 조사
```

원칙: 엔트리는 설정·조립만, 숫자는 `RobotProfile`·`LidarSpec`·`LeanProfile` 한 곳에, 토픽·프레임 규약은 `ros2/graph.py` 한 곳에.
새 기종은 `robot/profile.py` 에 `RobotProfile` 하나를 더 적으면 spawn·수술·라이다·factsheet 가 같은 값을 본다.

## 실행

```bash
source /opt/ros/humble/setup.bash
cd ~/isaacsim && ./python.sh <repo>/sil/apps/warehouse_sim.py                 # T3 단일 (/cmd_vel /odom /scan)
cd ~/isaacsim && WSIM_N=3 ./python.sh <repo>/sil/apps/warehouse_sim.py        # 3대 (/amr01~03/…)
cd ~/isaacsim && WSIM_N=6 WSIM_LEAN=e WSIM_INST=0 ./python.sh <repo>/sil/apps/warehouse_sim.py   # 경량 e (렌더 off) — 병렬 실측 구성
python3 <repo>/sil/t3_warehouse/ros2/multi_check.py --n 3 --drive amr02       # 개통 검증 (PASS 면 exit 0)
ros2 launch <repo>/sil/t3_warehouse/ros2/loc.launch.py                        # AMCL (단일 모드)
cd ~/isaacsim && ./python.sh <repo>/sil/apps/teleop_sim.py                    # T1
cd ~/isaacsim && ./python.sh <repo>/sil/apps/mapping_sim.py --laps 2          # T2
```

옛 경로(`t3_warehouse/warehouse_sim.py`, `t1_teleop/teleop_sim.py`, `t2_mapping/mapping_sim.py`, `t3_warehouse/view_scene.py`,
`t3_warehouse/ros2/patrol.py` 등)는 셔임이라 그대로 실행된다. 병렬 실측의 래퍼(`wsim_lean.py`)가 하던 몽키패치는 `WSIM_LEAN` 프로파일로 대체됐다.

## 테스트

```bash
cd <repo>/sil && python3 -m pytest            # sil_isaac 순수부 · sil_ros · scene_builder · t4_agent 단위 테스트 (Isaac 불필요)
```
Isaac 실물 검증(리팩토링 후 2026-09-07): `WSIM_N=2 apps/warehouse_sim.py` 부팅 → `multi_check --n 2 --drive amr02` PASS
(odom·scan 45 Hz, amr02 만 0.68 m 이동, TF 프레임 전부 확인).

## 실측 상수 (FMS 인도용, 상세 `calibration/turn_probe/README.md`)

- 회전 중심 = 구동축 = 포즈(/odom) 원점. 회전 중 원점 이탈 ≤ 1.3 cm.
- 차체: 축 기준 앞 0.40 m · 뒤 1.03 m · 폭 0.66 m → 1 m 격자에서 자기 칸 + 뒤 칸 1개 점유, 스팟턴 스윙 반경 1.09 m.
- 각속도: 지령 대비 95~97 %, 90° 폐루프 회전 1.9 s(T4 한계 1.2 rad/s), 1 m 직진 2.2 s(T4 E2E).
- 운동학: 바퀴 반경 0.08 m · 트랙 0.58 m (`sil_isaac/robot/profile.py`). T1 의 옛 0.115/0.413 은 오류였고 그때의 0.835 m/s 상한은 그 결과다.

동시 실행(9/5 실측, `pages/working/isaac_parallel_measure.html`): 원본 구성 2개, 경량 e 6개 동시 정상(RTF 1.25), 7개는 OOM.

남은 것: sim-runner 데몬(Sparkplug NCMD) · 에이전트 ×N 기동, AMCL 폐루프 정지 정밀도 · 오돔 노이즈, 엣지 통과 · 도킹 시간 분포 실측(DES 환류).
