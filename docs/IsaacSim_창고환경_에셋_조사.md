# Isaac Sim 물류창고 환경 에셋 조사 (3PL 시나리오 기준)

기획 보충자료 부속 문서 — 2026-08 웹 조사 기준
결론: 완성형 씬 탐색이 아니라 **NVIDIA 공식 모듈러 에셋을 파라메트릭 제너레이터의 부품으로 조합**하는 전략. 에셋 제작에 시간을 쓰지 않으면서 "NVIDIA 공식 에셋만 사용"이라는 재현성 서사를 확보한다.

---

## 1. 권장 구성 요약

| 계층 | 사용 에셋 | 용도 |
|---|---|---|
| 로봇 | Idealworks iw.hub (공식 내장, 3종 variant) | 기준 차량 |
| 개발용 환경 | 내장 warehouse 환경 4종 + small warehouse digital twin | ROS2 브리지·멀티로봇 파이프라인 개발 |
| 본 씬 구조 | Modular Warehouse 팩 (+Warehouse Creator 확장 참조) | 파라메트릭 제너레이터의 건물 부품 |
| 씬 내부 | SimReady Warehouse 01 팩 (팔레트·랙·램프) | 물리·시맨틱 라벨 내장 오브젝트 |
| 외란 연출 | 사람 시뮬레이션 (버전별 상이 — 7절 주의) + 지게차 에셋 | 통로 점유 외란의 SIL 대응물 |
| 3PL 특화 | 도크/스테이징: 마킹+텍스처 경량 처리 (9절) | 시각 연출 |

---

## 2. 로봇 에셋: iw.hub (공식 내장 확인됨)

Isaac Sim 공식 로봇 에셋 라이브러리에 iw.hub가 3종 variant로 포함되어 있다.

**공식 스펙 기재**: LiDAR·카메라 탑재 자율주행 모바일 베이스, NVIDIA AGX 기반, **적재하중 1,000kg, 최고 속도 2.2m/s**. → DES 엣지 통과시간 모델과 VDA 5050 factsheet 파라미터의 1차 기준값으로 사용.

**variant 구성** (경로는 버전에 따라 `[Isaac Sim Assets Path]/Isaac/Robots/Idealworks/iwhub/` 계열):

| 파일 | 내용 | 용도 |
|---|---|---|
| `iw_hub.usd` | 모바일 베이스 물리 리깅 완료 | 멀티로봇 물리 시뮬 기본 |
| `iw_hub_sensors.usd` | 물리 + 센서 API (LiDAR, 1인칭/3인칭 내비 카메라) | 매핑 런·AMCL 검증 실험용 |
| `iw_hub_static.usd` | 물리 없는 정적 모델 | 배경 연출·경량 씬 |

메뉴 생성 경로: Create > Isaac > Robots > Wheeled Robots > Idealworks. 최신(6.0) 문서 기준 PhysX RigidBody/Articulation/Joint/Collision API 리깅 확인, 7 joints / 8 links / 7 DOF (리프트 관절 포함).

**활용 팁**: 멀티로봇 대수가 늘면 센서 variant를 전 차량에 쓰지 말 것 — 렌더 패스가 병목. 운영 시뮬은 `iw_hub.usd`(GT+노이즈 localization), 검증 1대만 `iw_hub_sensors.usd`.

**부가 발견**: Replicator Agent(IRA)용 iw.hub 애니메이션 컨트롤러 설정(yaml)이 별도로 존재 — 물리 없이 애니메이션 기반으로 로봇을 움직이는 방식(navmesh 경로), 리프트 상승/하강 애니메이션 포함. 데모 영상용 대량 배경 로봇 연출에는 이쪽이 물리 시뮬보다 훨씬 가볍다. 단 이것은 연출용이며 검증용 물리 시뮬과 혼용 금지(리포트에 구분 명시).
→ https://docs.isaacsim.omniverse.nvidia.com/latest/action_and_event_data_generation/ext_replicator-agent/ext_isaacsim_anim_robot.html

**지게차**: ForkliftB 에셋(물리 리깅 + 센서 variant) 공식 포함 — 3PL 혼재 환경 연출·외란 오브젝트로 활용 가능.
→ 로봇 에셋 전체 목록: https://docs.isaacsim.omniverse.nvidia.com/6.0.0/assets/usd_assets_robots.html

---

## 3. 내장 기본 창고 환경 — 개발 단계용

Isaac Sim 기본 제공 환경에 창고가 포함된다: **선반·오브젝트가 배치된 창고 환경 4가지 구성**(warehouse, full_warehouse, warehouse_with_forklifts 등) + **소형 창고 디지털 트윈**.
→ https://docs.isaacsim.omniverse.nvidia.com/latest/assets/usd_assets_environments.html

**위치**: Create > Environments 메뉴 또는 Content Browser에서 usd 검색.
**용도 한정**: ROS2 브리지, 네임스페이스/TF, 멀티로봇 스폰, SLAM 매핑 런 등 파이프라인 개발·디버깅. 규모가 소~중형이라 3PL 데모 씬으로는 부적합(차별성 없음).

---

## 4. Modular Warehouse 팩 + Warehouse Creator 확장 — 파라메트릭 제너레이터의 기반

**핵심 발견.** 최신 Isaac Sim에 Warehouse Creator 확장이 공식 포함되어 있으며, 동작 방식이 본 프로젝트의 파라메트릭 접근과 동일하다:

- **논리 그리드 위에서 동작**: 생성 시 점유 셀마다 Center/Floor 프림, 노출된 셀 경계마다 Wall 프림을 생성. 인접 점유 셀 사이 벽은 자동 생략.
- **에셋 팩 구조**: Center/Floor·Wall 에셋을 variant 시스템으로 묶은 팩 단위. 기본값은 Isaac Sim 에셋 팩(`[Isaac Sim Assets Path]/Isaac/Environments/Modular_Warehouse_New/`)이며 **커스텀 팩 지원** — 팩 구조 예시로 Modular Warehouse 팩이 공식 참조됨.
- **로컬 성능 권장**: 에셋 팩을 로컬 다운로드 후 Asset Root Path 지정.
→ https://docs.isaacsim.omniverse.nvidia.com/latest/digital_twin/warehouse_logistics/ext_omni_warehouse_creator.html

**프로젝트 적용 방식 (권장)**: GUI 확장을 직접 쓰기보다, 파라메트릭 제너레이터(Python)가 Modular Warehouse 팩의 동일 부품을 프로그래매틱하게 배치하도록 구현. 즉 "그리드 사양(JSON) → USD 씬"을 자체 코드로 생성하되 부품은 공식 팩 사용. 이러면 (a) 씬 생성이 파이프라인 코드에 통합되고, (b) Warehouse Creator의 그리드 규약을 참조 설계로 삼아 검증된 패턴을 따르게 되며, (c) "NVIDIA 공식 모듈러 에셋 기반"이라는 문구가 성립한다.

---

## 5. SimReady Warehouse 01 팩 — 씬 내부 채우기

SimReady 에셋은 **시맨틱 라벨·상세 캡션·USDPhysics 물리 속성이 내장**된 OpenUSD 3D 모델 규격이며, SimReady Warehouse 01 팩에 **팔레트, 스토리지 랙, 램프 등 창고 오브젝트 USD 모델이 대량 포함**되어 있다. 드래그 앤 드롭으로 씬 배치 가능.
→ https://developer.nvidia.com/blog/build-synthetic-data-pipelines-to-train-smarter-robots-with-nvidia-isaac-sim/

**본 프로젝트에서의 가치**:
1. **물리 사전 세팅** — 라이다가 랙을 실제처럼 감지, 충돌 정상 동작 → 안전 필드 정지 로직 검증에 즉시 사용 가능.
2. **시맨틱 라벨 내장** — 향후 Replicator 합성 데이터 확장(기존 Isaac Sim Replicator 경험과 연결) 시 라벨링 공짜.
3. **파라메트릭 제너레이터의 내부 부품** — 랙 열·팔레트 배치를 파라미터로 SimReady 오브젝트 인스턴싱.

---

## 6. NVIDIA Assets 브라우저 — 사용 시 주의

Window > Browsers > NVIDIA Assets에서 Industrial > Buildings > Warehouse 등 추가 에셋 접근 가능(Warehouse01 등 건물 단위 에셋). 단 공식 문서가 명시하는 함정 두 가지:

1. **단위 불일치**: NVIDIA Assets는 아트팀 제작물이라 **센티미터 스케일인 경우가 있음** — Isaac Sim 전용 에셋(미터)과 혼용 시 수동 스케일 조정 필요. 스케일이 어긋나면 라이다 감지·물리·경로 좌표가 전부 틀어진다.
2. **물리 미설정**: 정적 장식물에는 Rigid Body/Collider가 없을 수 있음 — Add > Physics > Rigid Body with Colliders 프리셋으로 수동 부여 후 로컬 레이어에 저장하는 워크플로가 공식 안내됨.
→ https://docs.isaacsim.omniverse.nvidia.com/latest/digital_twin/warehouse_logistics/tutorial_static_assets.html

**팀 규칙 권장**: 에셋 소스별 체크리스트(단위 확인 → 콜라이더 유무 → 시맨틱 라벨 유무)를 만들어 씬에 넣기 전 통과시킬 것.

---

## 7. 사람 시뮬레이션 — 버전 함정 주의 (중요)

3PL 창고는 사람·지게차 혼재 환경이므로, DES의 "통로 점유 외란" 모델에 대응하는 SIL 연출이 필요하다. 그런데 이 기능의 API가 **버전에 따라 완전히 다르다**:

- **Isaac Sim 4.2 계열**: `omni.anim.people` 확장 + People Simulation UI. 텍스트 명령(Spawn/GoTo 등)으로 캐릭터 스폰·이동, 지원 동작은 idle/look around/queue/sit·stand/walk. 정적 장애물 회피에는 NavMesh 베이크 필요.
  → https://docs.isaacsim.omniverse.nvidia.com/4.2.0/features/warehouse_logistics/ext_omni_anim_people.html
- **Isaac Sim 4.5 이후**: 공식 경고 — **omni.anim.people는 Isaacsim.Replicator.Agent(IRA)로 대체 중**이며, 기존 코드를 IRA로 이관할 것을 강력 권고. 캐릭터 스폰·제어가 IRA config/Command Editor 방식으로 변경.
  → https://docs.isaacsim.omniverse.nvidia.com/4.5.0/replicator_tutorials/ext_replicator-agent/ext_omni_anim_people.html
- **5.x 실사용 보고**: People Simulation 탭이 사라져 IRA 사용이 사실상 강제되고, "사람 한 명 걷게 하기엔 IRA가 오버킬"이라는 커뮤니티 불만·NavMesh 베이크 이슈(NavMesh Volume이 바닥을 덮어야 동작) 보고 존재.
  → https://github.com/isaac-sim/IsaacSim/discussions/477
  → https://forums.developer.nvidia.com/t/people-walk-simulation-with-isaacsim5-0/341979

**권고**: 프로젝트 초기에 Isaac Sim 버전을 고정하고(버전 선택 자체가 이 기능에 좌우될 수 있음), 사람 연출 스코프를 "통로 횡단 → 로봇 안전 필드 정지" 데모 컷 1~2개로 한정할 것. 외란의 통계 모델링은 DES가 담당하므로 SIL의 사람은 연출·검증용 최소 구현이면 충분하다.

---

## 8. 부가 유틸리티

- **컨베이어 벨트 유틸리티**: 공식 제공(2022.2부터) — 피킹/합류 스테이션 연출용.
- **cuOpt 연동**: 공식 통합 존재 — 본 프로젝트는 자체 플릿 매니저가 있으므로 미사용이나, "NVIDIA 라우팅 스택 대신 VDA 5050 자체 관제를 쓴 이유"를 어펜딕스 Q&A로 준비해두면 좋음.
→ https://forums.developer.nvidia.com/t/isaac-sim-2022-2-0-is-out-and-live-now/237375

---

## 9. 3PL 특화 요소의 처리

공식 에셋으로 랙 통로·팔레트·지게차·사람까지는 커버되나, 3PL 창고의 특징 요소는 완성 에셋이 없다:

| 요소 | 처리 방안 | 근거 |
|---|---|---|
| 도크 도어 구역 | 벽면 셔터 텍스처 + 도크 번호 마킹 | 시뮬레이션상 도크는 "팔레트 생성/소멸 스테이션 노드"라 물리 디테일 불필요 |
| 스테이징 에어리어 | 바닥 마킹(페인트 라인) + SimReady 팔레트 더미 | 동일 — 시각 연출로 충분 |
| 통로 방향 표시 | 바닥 화살표 데칼 | 경로망 그래프의 일방/양방 설정과 시각 일치 → 데모 이해도 상승 |

서드파티 유료 에셋(예: Physicl 등 물리 특화 커스텀 팩 제작사)이 생겨나는 추세이나, 포트폴리오 단계에서는 불필요 — "NVIDIA 공식 에셋만 사용"이 재현성·인용 관점에서 오히려 유리.

---

## 10. 실행 체크리스트

1. Isaac Sim 버전 확정 (사람 시뮬 API 기준으로 결정 — 7절)
2. 에셋 팩 로컬 다운로드 (Modular Warehouse, SimReady Warehouse 01) 및 Asset Root 설정
3. 내장 full_warehouse에서 ROS2 멀티로봇 파이프라인 개발
4. 파라메트릭 제너레이터를 Modular 팩 부품 기반으로 재구성 (그리드 사양 JSON → USD)
5. SimReady 오브젝트 인스턴싱으로 랙/팔레트 파라메트릭 배치
6. iw.hub: 운영용 base variant + 검증 1대 sensors variant 이원화
7. 에셋 반입 체크리스트 운영 (단위 → 콜라이더 → 라벨)
8. 도크/스테이징 마킹 텍스처 제작 (경량)
9. 사람 횡단 데모 컷 구현 (버전별 API 확인 후)

---

## 부록: 출처 링크 일람

| 항목 | 링크 |
|---|---|
| 환경 에셋 공식 목록 (창고 4종 + 디지털 트윈) | https://docs.isaacsim.omniverse.nvidia.com/latest/assets/usd_assets_environments.html |
| 로봇 에셋 공식 목록 (iw.hub 3종, ForkliftB) | https://docs.isaacsim.omniverse.nvidia.com/6.0.0/assets/usd_assets_robots.html |
| iw.hub 스펙 기재 (4.2 문서 미러) | https://docs.robotsfan.com/isaacsim/4.2.0/features/environment_setup/assets/usd_assets_robots.html |
| Warehouse Creator 확장 | https://docs.isaacsim.omniverse.nvidia.com/latest/digital_twin/warehouse_logistics/ext_omni_warehouse_creator.html |
| Static Warehouse Assets 튜토리얼 (단위·물리 주의사항) | https://docs.isaacsim.omniverse.nvidia.com/latest/digital_twin/warehouse_logistics/tutorial_static_assets.html |
| SimReady Warehouse 01 팩 소개 (NVIDIA 기술 블로그) | https://developer.nvidia.com/blog/build-synthetic-data-pipelines-to-train-smarter-robots-with-nvidia-isaac-sim/ |
| omni.anim.people (4.2 문서) | https://docs.isaacsim.omniverse.nvidia.com/4.2.0/features/warehouse_logistics/ext_omni_anim_people.html |
| omni.anim.people → IRA 대체 경고 (4.5 문서) | https://docs.isaacsim.omniverse.nvidia.com/4.5.0/replicator_tutorials/ext_replicator-agent/ext_omni_anim_people.html |
| 5.x 사람 시뮬 이슈 (커뮤니티) | https://github.com/isaac-sim/IsaacSim/discussions/477 / https://forums.developer.nvidia.com/t/people-walk-simulation-with-isaacsim5-0/341979 |
| IRA 애니메이션 로봇 컨트롤러 (iw.hub yaml 예시) | https://docs.isaacsim.omniverse.nvidia.com/latest/action_and_event_data_generation/ext_replicator-agent/ext_isaacsim_anim_robot.html |
| 컨베이어·People 도입 릴리스 노트 (2022.2) | https://forums.developer.nvidia.com/t/isaac-sim-2022-2-0-is-out-and-live-now/237375 |
| idealworks–NVIDIA 생태계 통합 발표 (iw.hub Jetson Orin, Omniverse) | https://idealworks.com/en/idealworks-robotics-ecosystem-to-integrate-nvidia-ai-nvidia-omniverse-and-nvidia-isaac-technologies/ |

주의: Isaac Sim 에셋 경로·확장 구성은 버전 간 변동이 크므로(위 사람 시뮬 사례), 기획서 인용 시 확정한 버전의 문서로 링크를 통일할 것.
