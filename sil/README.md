# sil/ — Isaac Sim · ROS2 구현 트랙

SIL(가상 시운전) 구현 코드가 들어올 자리. 로드맵과 기술 전제:

- 로드맵: `pages/working/sil_roadmap.html` — T1 Teleop → T2 SLAM 매핑+자율 추종 → T3 창고 반송 → T4 다중 로봇+FMS
- 아키텍처: `pages/working/runtime_architecture.html` — Humble + Fast DDS + 자체 추종기(Pure Pursuit), VDA 5050/MQTT 경계
- 실행 환경: 홈서버 Isaac Sim 6.0.1 (~/isaacsim) + /opt/ros/humble — Isaac 동시 1인스턴스 규칙

현재 구성:
- `sil/t3_warehouse/` — **T3-A 진행(2026-08-27)**: 씬 빌더 — 그리드→USD(벽·컨베이어 박스 + NVIDIA 랙 부품 조립), omap 역생성 V&V
- `sil/t3_warehouse_map/` — **T3 환경 확정(2026-08-27)**: DXF 공장 도면 → occupancy grid → 랙·스테이션 배치(map_gen v6.3 — 크로스-아일·리저브 선언·플래너 뷰 자가검증)
- `sil/t2_follower/` — **T2-B 완료(2026-08-27)**: 자체 추종기(램프·사각 보호 필드) — 20엣지 완주, 장애물 정지·재개 검증, 엣지 시간 결정성(±0.2s)·정지 페널티(+7.6s) 실측
- `sil/t2_mapping/` — **T2-A 완료(2026-08-27)**: SLAM 매핑 런 — GT 방 vs 지도 오차 평균 3.4cm·커버리지 91.6%
- `sil/t1_teleop/` — **T1 완료(2026-08-27)**: /cmd_vel 텔레옵 + /clock·/odom 발행 + 가감속 실측(DES 환류)

예정 구성:
- `sil/t3_warehouse/warehouse_sim.py` — iw.hub+라이다 투입, 스테이션 왕복 order 연쇄, 리프트 도킹
- `sil/adapter/` VDA 5050 어댑터 (order 수신·state 보고, T4)
- `sil/calibration/` 엣지 통과시간·도킹 분포 실측·환류 스크립트
