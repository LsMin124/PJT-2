# sil/ — Isaac Sim · ROS 2 구현 트랙 (SIL, 가상 시운전)

정본은 팀 레포 `S15P21A106/2_Simulation/`이고 여기는 작업 사본이다(두 사본은 경로 문자열 `sil/` ↔ `2_Simulation/`만 다름).
로드맵: `pages/working/sil_roadmap.html` — T1 텔레옵 → T2 매핑·추종 → T3 창고 씬 → T4 에이전트 → 다중 로봇 검증 런.
실행 환경: 홈 GPU 서버(RTX 5080 16 GB · RAM 64 GB), Isaac Sim 6.0.1 `~/isaacsim` + `/opt/ros/humble`.

| 디렉토리 | 내용 | 상태 |
|---|---|---|
| `t1_teleop/` | /cmd_vel 텔레옵 + /clock·/odom 발행, 가감속 실측(최고 0.835 m/s) | 완료 (8/27) |
| `t2_mapping/` | slam_toolbox 매핑 — GT 대비 오차 평균 3.4 cm · 커버리지 91.6 % | 완료 (8/27) |
| `t2_follower/` | 자체 추종기(램프·보호 필드) 20엣지 완주 — 이후 T4 그리드 프리미티브로 대체 | 완료·대체 (8/27) |
| `t3_warehouse_map/` | DXF 도면 → occupancy grid → 랙·스테이션·구역(v6.0.1: A~V 22구역 · 랙 132 · 스테이션 46 · 충전 6). `map/`이 단일 소스, `map2/`는 wallA 변형(보류) | 완료 (8/31) |
| `t3_warehouse/` | 그리드 → USD 씬 빌더(건물 실측 · 화물 드레싱 · omap V&V) + `warehouse_sim.py`(iw.hub ×N · 물리 라이다 720빔 · ROS 2 개통, `WSIM_N`으로 `/amr01~/…` 네임스페이스) + AMCL 런치 · patrol · multi_check | 완료 (8/31 · 다중 로봇 9/6) |
| `t4_agent/` | ROS 2 에이전트 — VDA 5050 MQTT 브리지 + order executor + 그리드 프리미티브(스팟턴 · 직진 · 후진) + fake robot/FMS. Isaac 없이 배리어 틱 E2E 7틱 통과 | 뼈대 완료 (9/3) |
| `calibration/turn_probe/` | iw.hub 스팟턴 실측 — 회전 중심 · 차체 돌출 · 각속도 응답 · 90° 회전 시간 → FMS 헤딩 상수 인도 | 완료 (9/6) |
| `tools/asset_probe/` | Isaac 6.0 로봇 에셋 정적 조사 + 헤드리스 주행 프로브(두 번째 AMR 후보: Nova Carter 등) | 완료 (9/4) |

실측 상수(FMS 인도용, 상세 `calibration/turn_probe/README.md`):
- 회전 중심 = 구동축 = 포즈(/odom) 원점. 회전 중 원점 이탈 ≤ 1.3 cm.
- 차체: 축 기준 앞 0.40 m · 뒤 1.03 m · 폭 0.66 m → 1 m 격자에서 자기 칸 + 뒤 칸 1개 점유, 스팟턴 스윙 반경 1.09 m(주변 8칸에 걸침).
- 각속도: 지령 대비 95~97 %, 90° 폐루프 회전 1.9 s(T4 한계 1.2 rad/s), 1 m 직진 2.2 s(T4 E2E).
- 운동학: 바퀴 반경 0.08 m · 트랙 0.58 m(에셋 함정은 `t3_warehouse/README.md`).

동시 실행(9/5 실측, `pages/working/isaac_parallel_measure.html`): 원본 구성은 인스턴스당 RAM 8.1 GB · VRAM 5.3 GB로 2개, 렌더·시각화를 끈 경량 구성은 6개 동시 정상(RTF 1.25). 이전의 "동시 1인스턴스" 규칙은 폐기.

남은 것: sim-runner 데몬(`sim/control`) · 에이전트 ×N 기동, AMCL 폐루프 정지 정밀도 · 오돔 노이즈, 엣지 통과 · 도킹 시간 분포 실측(DES 환류).
