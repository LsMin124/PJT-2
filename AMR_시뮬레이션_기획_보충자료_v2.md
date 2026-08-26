# AMR 도입 타당성 시뮬레이션 파이프라인 — 기획 보충자료 (v2)

설계 결정 및 근거 정리 (2026-08 논의 기준)
v2 변경사항: 타겟 시장을 중견 3PL로 확정, 전 출처에 링크 부여, VDA 5050 factsheet 활용 방안 추가

---

## 1. 프로젝트 정의와 타겟 시장

### 1.1 프로젝트 정의

실제 산업 벤더의 AMR 도입 프로세스(현장 매핑 → 플릿 구성 설계 → 가상 시운전 → 견적)를 공개 표준 기반으로 재현하고, 그 위에 자동 구성 최적화와 불확실성 정량화를 얹은 **2단계 시뮬레이션 파이프라인**.

기존 정의였던 "견적 SaaS + 검증용 시뮬레이션"은 Isaac Sim이 액세서리로 전락하는 구조적 문제가 있었다. 재정의 후에는 두 시뮬레이션 계층이 각각 실존하는 산업 프로세스를 담당한다. DES는 개략 검토(rough planning) 단계, ROS2+Isaac Sim SIL은 가상 시운전(virtual commissioning) 단계에 대응하며, 최종 산출물이 견적+검증 리포트가 된다.

파이프라인 전체 흐름:

```
현장 스캔(SLAM 매핑) 또는 파라메트릭 사양
  → 2D 점유격자 지도 → 경로망 그래프 (지도 위 에디터 + 스켈레톤 자동 초안)
  → [탐색 단계] 자동 구성 최적화 루프 (평가기 = 보정된 SimPy DES)
  → [검증 단계] Isaac Sim 가상 시운전 (최적해 후보 최종 검증)
  → 견적 + 위험 정량화 리포트 (LLM 자동 해설, 제안서 삽입용 섹션 포함)
```

### 1.2 타겟 시장: 계약물류 중심 중견 3PL

**타겟 정의**: 자체 시뮬레이션/엔지니어링 조직을 둘 규모는 아니지만, 화주 입찰 때마다 자동화 제안을 요구받는 계약물류(팔레트/토트 중심) 중견 3PL. 포지셔닝 문장: **"엔지니어링 조직 없이 엔지니어링 조직급 제안서를 내게 해주는 툴."**

화주(창고 직접 소유 기업)가 아닌 3PL을 타겟하는 구조적 근거:

1. **반복 사용이 구조적으로 보장됨**: 화주에게 AMR 도입 검토는 일생 1~2회 이벤트라 툴이 1회성 컨설팅이 되지만, 3PL은 화주 RFP 입찰마다 "이 창고에서 이 구성으로 이 단가에 처리"라는 제안 숫자(대수·처리량·cost-per-line)를 마감 내에 산출해야 한다. 툴이 도입 검토 도구가 아니라 **수주 영업 무기**가 되며, SaaS 구독 과금이 성립한다.
2. **SLA·계약기간이 불확실성 정량화의 필수 수요를 만듦**: 3PL 계약(통상 3~5년)은 자동화 투자 회수가 계약 기간 내에 끝나야 하고, 화주와의 처리량 보장 조항(SLA) 준수가 생존 문제다. P10 처리량("최악의 달에도 보장되는 숫자")과 회수기간 신뢰구간은 이 맥락에서 부가 기능이 아니라 계약 리스크 판단의 필수 입력이다.
3. **다현장 운영의 템플릿 재사용**: 수십 개 창고를 운영하므로 파라메트릭 레이아웃과 캘리브레이션 파라미터가 현장 간 재활용되며, 캘리브레이션 라이브러리가 자산으로 축적된다.
4. **고객 교체 시 반복 재설계 수요**: 화주가 바뀌면 동일 창고의 물량 프로파일·레이아웃이 바뀌어 재시뮬레이션이 필요 — 브라운필드 문제 정의와 정확히 일치하는 반복 수요.
5. **벤더 중립성의 실수요자**: 현장마다 다른 로봇을 쓸 수 있고 특정 벤더 종속 시 협상력을 잃는 주체가 3PL이므로, VDA 5050 기반 중립 비교 포지셔닝의 1차 수요자다.

**시장 타이밍 근거**: AMR 투자 회수기간이 5년 전 36~48개월에서 18~24개월로 단축되며, 과거 자동화를 대기업 전유물로 여기던 미드마켓 3PL 사업자의 도입이 확산 중.
→ 출처: Dataintelo, "AMR Simulation Platforms Market Research Report 2034" — https://dataintelo.com/report/amr-simulation-platforms-market

**제외 대상 명시**: (a) 대형 3PL — 자체 엔지니어링 조직 보유로 고객이 아니라 경쟁자에 가까움. (b) 이커머스 풀필먼트 특화 3PL — GTP 그리드 수요 중심이라 스코프 밖(4절 차량 선정 논리와 연동).

---

## 2. 산업용 AMR 주행 방식 — 자율성 제거의 근거

시뮬레이션 모델링의 대전제. 실제 산업 AMR은 연구용 자율주행과 달리 **자율성을 의도적으로 낮춰 예측 가능성과 가동률을 확보**하는 방향으로 설계되어 있다.

**위치추정**: 설치 시 SLAM으로 지도를 1회 생성한 뒤, 운영 중에는 지도를 고정하고 localization(스캔 매칭)만 수행. 방식은 자연지형 LiDAR 매칭(주류), 반사판 삼각측량, 바닥 QR 그리드(GTP 계열). 휠 오도메트리+IMU와의 EKF 융합이 표준.

**경로**: 자유 주행이 아니라 지도 위에 정의된 노드-엣지 경로망(가상 레일) 위에서만 주행. VDA 5050의 주행 모델 자체가 "중앙 관제가 내려준 노드 시퀀스를 따라가라"이다. 근거는 (a) 작업자 관점의 예측 가능성/안전, (b) 그래프 기반이어야 풀리는 다중 로봇 통행권 관리, (c) 재현성/디버깅.
→ 출처: VDA 5050 공식 사양 (VDA/VDMA 공동 개발, KIT-IFL 관리) — https://github.com/VDA5050/VDA5050

**장애물 대응**: 회피가 아니라 감속→정지→대기→재개가 기본. 우회는 경로망 내 대체 엣지 재라우팅 수준.

**안전 계층**: 주행 SW와 완전히 분리된 독립 하드웨어. 안전 인증 LiDAR의 보호 필드 침범 시 안전 PLC 경로로 모터 전원 직접 차단(ISO 3691-4 인증 대상). 항법 SW가 죽어도 안전은 유지되는 구조.

**주행 정확도의 원리** (참고): 쉬운 문제 설정(고정 경로·저속·균일 바닥) × 고주파 서보 루프(경로추종 10~50Hz + 휠 서보 수백 Hz) × 절대 기준 위치 리셋(누적 오차 구간 최소화) × 사전 계산된 피드포워드 × 도킹만 별도 정밀 정렬(±5~10mm). 단일 첨단 기술이 아니라 계층별 오차 억제의 곱.

**시뮬레이션 시사점**: (1) DES와 SIL 모두 "그래프 엣지 이동 + 예약 대기 + 정지 이벤트"로 모델링하는 것이 실제 산업 거동에 충실하다. (2) SIL에서 궤적의 mm 단위 충실도는 불필요하며, 중요한 것은 가감속 프로파일과 엣지/교차로 통과 시간이다. (3) 도킹 시간은 미세 정렬 재시도로 분산이 크므로 DES에서 상수가 아닌 분포로 모델링한다.

---

## 3. 2계층 아키텍처와 정합성 전략

**핵심 원칙**: 플릿 매니저(태스크 할당 + MAPF + space-time 예약)를 시뮬레이터 중립 모듈로 분리하고, DES와 Isaac Sim이 **물리적으로 같은 코드**를 공유한다. 두 레이어의 차이는 실행기뿐이다.

```
        [플릿 매니저: 할당 + MAPF + 예약]  ← 단일 코드베이스
              ↓ (노드 시퀀스 order, VDA 5050 모델)
   ┌──────────┴──────────┐
   DES 실행기              Isaac Sim 실행기
   (엣지 통과시간 샘플링)     (경로추종 제어 + 물리)
```

이 구조에서 정합성 검증은 "엣지 통과시간 분포의 일치" 문제로 환원되어 명확한 정량 지표가 생긴다. 또한 **캘리브레이션 루프**가 성립한다: Isaac Sim SIL에서 엣지 통과시간/도킹 시간 실측 분포를 뽑아 DES 입력 분포로 역주입 → 보정된 DES로 대규모 스윕. VDI 3633 관점의 정석적 V&V 스토리. 캘리브레이션은 서비스 런타임이 아니라 오프라인 1회 캠페인으로 수행해 파라미터 라이브러리화한다(1.2절 3항의 3PL 다현장 재사용 자산과 연결).

**유사 선행연구**: 실제 로봇 관제 SW를 시뮬레이션과 실배치 양쪽에 동일하게 연동(FlexSim/Fleet Manager 인터페이스)하여 플릿 사이즈 시나리오를 평가한 프레임워크 존재 — 아키텍처 타당성의 학술 근거이자 관련연구 인용 대상.
→ 출처: "Decision-Making Framework For AMR Fleet Size In Manufacturing Environments" (2025) — https://www.researchgate.net/publication/390515227_Decision-Making_Framework_For_AMR_Fleet_Size_In_Manufacturing_Environments

**DES 모델링 요소**: 엣지별 통과시간 분포, 교차로 예약 대기, 도킹 시간 분포, 충전 로직, 통로 점유 외란(포아송 도착+지속시간), 로봇별 MTBF/MTTR 가용성 파라미터. 마지막 항목은 자율성 제거로 인한 낙관 편향을 막는 장치이며, SLA 리스크를 판단해야 하는 3PL 고객에게는 보수적 숫자가 상품 가치다.

**Nav2 배제 근거**: 글로벌 플래너(플릿 매니저가 대체), 로컬 회피(의도적 제거), 복구 비헤이비어(산업 관행은 정지·보고) 모두 불필요. AMCL도 상용 로봇 도입 시나리오에서는 제조사 검증 영역이므로, 운영 시뮬레이션은 ground truth + 노이즈 주입으로 대체. Isaac Sim 측 스택은 "노드 시퀀스 수신 → 경로 보간 → Pure Pursuit → cmd_vel + 안전 필드 정지"로 최소화. 단, 로봇 1대 AMCL 검증 실험과 Nav2 비교 실험을 부록으로 두어 "알고 배제했다"를 물증화한다.

**AGV vs AMR 논점**: 현대 산업 AMR의 실질은 "자연지형 localization을 쓰는 AGV"에 가깝고, 차이의 본질은 주행 자율성이 아니라 배치·변경의 유연성(인프라 시공 불필요, 경로 변경이 SW 작업)이다. 시뮬레이터가 "FMS 관리하의 AGV"처럼 보이는 것은 모델링 오류가 아니라 현실의 정확한 포착이며, 리포트에 "VDA 5050 중앙 관제 모델 = 주요 상용 AMR의 실제 운영 방식"임을 명기한다. 참고로 VDA 5050 자체가 이기종 AGV/AMR 혼합 플릿의 단일 관제를 목적으로 만들어진 표준이다.

**신규 반영 — VDA 5050 AGV Factsheet 활용**: VDA 5050 사양의 factsheet는 차량 시리즈의 물리 특성·기구학·프로토콜 지원 범위를 기술하는 기계판독 가능(JSON) 문서로, 사양 원문에 "AGV 시스템의 계획(planning), 규모 산정(dimensioning), 시뮬레이션(simulation)에 활용" 목적이 명시되어 있다. 본 시뮬레이터의 차량 파라미터 입력을 이 factsheet 포맷으로 받으면 (a) 표준이 의도한 용도의 정확한 구현이라는 정당성, (b) 타 VDA 5050 차량으로의 파라미터 교체 일반화(4절)가 포맷 수준에서 보장된다.
→ 출처: VDA 5050 v2.x Factsheet 장 — https://github.com/VDA5050/VDA5050/blob/2.1.0/VDA5050_EN.md
→ 참고: VDA 5050 v3.0 (2026-03 릴리스, 자유주행 로봇의 계획 경로 공유·존 개념 추가) — https://github.com/VDA5050/VDA5050/releases

---

## 4. 기준 차량 선정: iw.hub (vs Geek+)

"리소스 공개" 같은 편의성 논거가 아니라 다음 4단 논리로 문서화한다.

**(1) 문제 범위 정합**: 본 서비스의 대상은 기존 레이아웃 유지 하의 팔레트/토트 운송 도입(브라운필드) — 1.2절의 계약물류 3PL 타겟과 일치. Geek+의 시장 지위 원천인 GTP 그리드(바닥 QR + pod 운반)는 레이아웃을 재설계하는 그린필드 문제로, 스캔/도면 기반 경로망 생성이라는 본 파이프라인의 핵심 기술이 적용될 자리가 없다. → "업계 1위 제품군은 본 문제의 후보군에 애초에 없다."

**(2) 제어 아키텍처 정합**: 확정된 뼈대가 VDA 5050 중앙 디스패치인데, iw.hub 제조사 idealworks는 BMW 스핀오프로 VDA 5050 진영의 핵심이며 iw.hub는 사실상 표준의 레퍼런스 구현체다. 반면 Geek+는 로봇-관제 프로토콜이 비공개인 수직통합 생태계로, 시뮬레이션하려면 디스패치 로직을 추측 재구성해야 하며 그 순간 V&V가 무너진다. 모델 타당성을 공개 사양 문서로 입증 가능한 것이 결정적 근거.
→ 출처: idealworks 공식 — https://idealworks.com (iw.hub 제품·VDA 5050 지원 명세는 기획서 인용 전 최신 페이지 재확인 권장)

**(3) 벤더 중립성**: 턴키 벤더 검토는 벤더 자체 시뮬레이션이 견적을 뽑아주므로 제3자 툴의 시장이 없다. 본 서비스의 존재 이유인 중립 타당성 검토는 상호운용 가능한 로봇 클래스를 전제하며, VDA 5050 준수 차량으로 시뮬레이션하면 결과가 "iw.hub의 예측"이 아니라 "VDA 5050 호환 AMR 일반의 예측"으로 일반화된다(factsheet 포맷 파라미터 교체로 MiR, Safelog 등 동급 차량 적용 가능 — 3절 신규 반영 항목과 연결). VDA 5050이 업계 보편 통신 표준으로 자리잡고 있다는 제3자 관찰도 확보됨.
→ 출처: Robotomated, "AMR Fleet Management Software: What to Look for in 2026" — https://robotomated.com/learn/warehouse/amr-fleet-management-software

**(4) 자산 출처 타당성**: iw.hub의 Isaac Sim 에셋은 BMW-NVIDIA 디지털 트윈 협업(iFactory/Omniverse) 유래의 제조사 연계 공식 에셋으로, 모델 데이터 타당성을 출처로 보증 가능. BMW 사례는 가상 시운전의 업계 대표 사례로 프로젝트 정체성과 서사가 연결된다. (기획서 인용 시 NVIDIA/BMW 공식 발표 페이지 링크를 최신으로 재확인해 부착할 것.)

로드맵에 "GTP 그리드형 모듈은 별도 확장" 한 줄을 남겨 스코핑 물증으로 삼는다.

---

## 5. 스코프 조정: 도면 파싱 제거와 3중 출처 재지정

도면 파싱은 기간 내 해결이 어려운 최고 난도 구간이므로 v2 로드맵으로 이관. 단, 도면이 먹여살리던 세 가지의 출처를 각각 재지정해야 한다.

| 산출물 | 기존 출처 | 변경 후 출처 |
|---|---|---|
| Isaac Sim 씬 (3D 환경) | 도면 IR | **파라메트릭 레이아웃 제너레이터** (기보유) |
| Localization 지도 | 도면 IR | **로봇 스캔 SLAM 1회 수행** (slam_toolbox, 2D 점유격자) |
| 경로망 그래프 | 도면 IR | **지도 위 그래프 에디터** (+선택: Voronoi/medial axis 스켈레톤 자동 초안) |

스캔 기반 매핑은 회피가 아니라 실제 AMR 커미셔닝 절차(설치 시 1회 SLAM → 지도 고정)의 충실한 재현이며, 가상 시운전 정체성과 정합한다. 지도 위 수동 그래프 작도 역시 상용 플릿 매니저 커미셔닝의 실제 방식이다. 3D 맵은 불필요하다 — 산업 표준은 안전 라이다 높이의 2D 점유격자다.

운영 중 localization 구성: 매핑 런은 실제 SLAM 1회(산출물: 지도 + SLAM 지도 vs 씬 ground truth 오차 비교 = V&V 자료), 멀티로봇 운영은 GT+노이즈, 로봇 1대만 AMCL 검증 실험. 제품 서사는 "v1 입력 = 현장 스캔 데이터 또는 파라메트릭 사양, 도면 파싱 = v2 모듈"로 정리한다. 3PL 맥락 보강: AMR 검토 실무에서 현장 실측/스캔(site survey)은 어차피 표준 선행 단계이므로 입력 요구가 어색하지 않다.
→ 참고 출처(도입 절차에서 site survey·digital twin 검증 단계): SmartLoadingHub, "Sizing AMR Performance Cost for Modern Warehouses" — https://www.smartloadinghub.com/insights/agv-amr/sizing-amr-performance-cost-modern-warehouses/

---

## 6. 차별성 전략 ① 자동 구성 최적화 (헤드라인 기여)

**시장 현황 (2026-08 웹 조사 검증)**: "벤더는 경험 기반으로 한다"는 초기 가설은 과장이었다. 시뮬레이션 기반 평가는 이미 업계 표준 관행이다:

- 대부분의 플릿 SW 벤더가 배치 전 시설·주문 프로파일·플릿 모델링 시뮬레이션 툴 제공 ("2주 시뮬레이션이 6개월의 저성능을 막는다")
  → 출처: Robotomated (2026) — https://robotomated.com/learn/warehouse/amr-fleet-management-software
- 인테그레이터/영업 측도 FlexSim 등으로 배치 전 시뮬레이션을 견적·영업 도구로 상용화
  → 출처: AGV Network, "AGV Simulation Benefits" — https://www.agvnetwork.com/agv-simulation-benefits
- 벤더 관점 플릿 사이징 방법론에서도 시뮬레이션이 표준 단계
  → 출처: TGW Logistics, "A Guide to AGV and AMR Robots" — https://www.tgw-group.com/us/news/detail/agv-amr-robots-guide/

그러나 **지배적 워크플로는 엔지니어가 시나리오 3~5개를 수동 정의 → 시뮬레이션 → 비교**다:

- FlexSim의 대표 기능이 시나리오 매니저 기반 대안 비교
  → 출처: Autodesk FlexSim 공식, "Warehouse Simulation" — https://www.flexsim.com/warehousing-simulation/
- 실제 사례에서도 3개 대안 시나리오 수동 구성·비교 방식
  → 출처: FlexSim 사례, "Optimizing Warehouse Throughput While Physical Distancing" — https://www.flexsim.com/material-handling/optimizing-warehouse-throughput-while-physical-distancing/
- FlexSim에 GA 기반 옵티마이저가 내장되어 있으나 목적함수 정의가 전제인 부가 기능 위치
  → 출처: Autodesk 커뮤니티 공식 답변 — https://forums.autodesk.com/t5/flexsim-forum/warehouse-optimization/m-p/13496577

즉 **"평가는 자동, 탐색은 수동"**이 현재 지형이다. 한편 대형사 신제품이 병렬 시뮬레이션 대량 탐색으로 진입 중 — 방향의 시장 검증 근거로 인용:

- GreyOrange Foundry (2026): 실운영 플릿 데이터 기반, 수천 개 시나리오 병렬 탐색·비용 예측
  → 출처: Logistics Business (2026-04) — https://logisticsbusiness.com/materials-handling/amr-agv/simulator-predicts-warehouse-performance/

**기여 주장 (정확한 문구)**: "기존 관행은 시뮬레이션 기반 수동 시나리오 비교에 머무는 반면, 본 시스템은 보정된 DES를 평가 함수로 한 자동 탐색 루프로 구성 공간을 체계적으로 탐색한다." 새 최적화 알고리즘 주장이 아니라 워크플로 자동화 기여이므로 SOTA와 싸울 필요가 없다.

**최적화 대상은 거시 층**: MAPF/교통관리(미시 층)는 연구 포화 지대이므로 인프라로 취급. 최적화는 구성 변수 — 로봇 대수, 그래프 토폴로지(일방/양방), 스테이션·충전기 배치, 디스패치/충전 정책 — 에 적용하며 목적함수는 기확정 지표인 cost-per-line. 탐색기는 그리드 스윕 → 베이지안 최적화/GA로 고도화. 최적해 후보는 Isaac Sim 가상 시운전으로 최종 검증.

**차별점 방어선**: FlexSim Experimenter 대비 — 범용 툴 애드온이 아니라 VDA 5050 관제 로직을 공유하는 도메인 특화 파이프라인 + SIL 검증 연결. GreyOrange Foundry 대비 — 자사 생태계 폐쇄형 vs 공개 표준 벤더 중립 + 물리 시뮬레이션 보정. 3PL 관점 한 줄: "Foundry는 GreyOrange 생태계 안의 도구지만, 입찰마다 다른 로봇을 검토해야 하는 3PL에게는 중립 툴이 필요하다."

**부수 서사**: "벤더 내부에만 존재하던 폐쇄 프로세스를 공개 표준(VDA 5050) + 공개 방법론(VDI 3633) + 중립 차량 모델의 개방형 파이프라인으로 재구성"한 것 자체가 정직한 기여이며, 충실한 재현이 곧 셀링포인트다.

---

## 7. 차별성 전략 ② 불확실성 정량화 견적 (서브 헤드라인, 공수 최소)

업계 산출물은 단일 점추정("12대, 340라인/h, 회수 2.4년")이다. DES는 확률 모델이므로 시드 반복 실행만으로 분포가 나오며, 이를 버리지 않고 상품화한다:

- 처리량 P50/P10 병기: "성수기 보수 계획은 P10 기준 권장"
- 회수기간 신뢰구간: "중앙값 2.4년, 90% 신뢰수준 3.1년 이내"
- 한계 분석: "12번째 로봇은 평균(+6%)이 아니라 안정성(P10 +14%)을 사는 것"

**3PL 타겟 확정에 따른 격상**: 이 기능은 3PL 맥락에서 부가 기능이 아니라 핵심 수요다. (a) P10 처리량 = 화주 SLA(처리량 보장 조항) 준수 가능성 판단의 직접 입력. (b) 회수기간 신뢰구간 = "3~5년 계약 기간 내 회수 완료" 판단의 직접 입력. (c) 리포트 문구를 SLA 언어("보장 가능 처리량")로 작성하면 3PL이 화주 제안서에 그대로 인용 가능.

기술 공수는 반복 실행 + percentile 추출로 최소이며, 반복 횟수 결정·신뢰구간은 DES 교과서 표준 절차이자 VDI 3633 요구사항이라 "표준 준수"로 포장된다. 방어 논리는 "통계적으로 당연한 것", 차별화 논리는 "그 당연한 걸 제안서에 넣을 수 있는 형태로 주는 서비스가 없다".

**부가 산출물 (기존 스윕 인프라의 후처리)**: (1) 민감도 토네이도 차트 — "견적을 흔드는 요인 Top 5" 자동 생성. (2) 단계적 도입 로드맵 — 수요 수준별 스윕 재배열로 "지금 6대 → 주문 X 돌파 시 9대" 증설 시점표. 3PL 맥락에서는 화주와의 물량 연동 조항(볼륨 커밋) 설계 자료로 용도 확장.

최적화 루프 방어와도 연결: "최적해 신뢰성" 공격에 "모든 후보를 분포로 평가, 최종 추천은 위험 조정 기준"으로 응수. *Measurement* 논문(계측·불확도)과 "측정 불확도 개념의 시뮬레이션 견적 이식"이라는 개인 서사 연결 고리.

---

## 8. AI 포지셔닝 맵

원칙: AI 딱지는 학습/적응의 실체가 있는 곳에만. 하나가 과장으로 판정되면 전체가 도매금이 된다.

```
탐색 층: AI ⭕ — 베이지안 최적화 ("해볼수록 감을 잡아가며 유망한 배치부터 시험")
해설 층: AI ⭕ — LLM 리포트 내레이션 (시뮬 숫자 → 제안서 삽입용 섹션 자동 생성)
실행 층: AI ❌ (의도적) — MAPF는 "탐색 알고리즘 기반 교통관리"로 명명
로드맵: RL 배차 학습 (SimPy DES를 gym 환경으로 재사용)
```

MAPF는 학술 분류상 고전 AI(휴리스틱 탐색)가 맞지만, 2026년 청중의 "AI" 해석(학습형 ML/LLM)과 어긋나 방어 후퇴를 유발하므로 딱지를 붙이지 않는다. 오히려 "안전·예측 가능성이 필요한 하층은 검증된 결정론적 알고리즘, 탐색적 상층은 AI"라는 명시가 실제 산업 설계 철학과 일치하며 신뢰도를 올린다.

**LLM 리포트 내레이션**: P10/P50, 민감도 순위, 병목, 증설 시점표를 구조화 입력으로 LLM에 넣어 경영 언어 보고서를 자동 생성. 3PL 타겟 확정에 따라 출력 타겟을 "경영진 보고서"에서 **"화주 제안서 삽입용 섹션"**(처리량 보장 근거, 단계 도입 로드맵, 리스크 분석)으로 조정 — 툴의 출력이 고객의 매출 문서에 직접 들어가는 구조. 숫자는 시뮬레이션이 만들고 LLM은 서술만 하는 구조로 환각 방어선 명확.

"AI 들어가요?" 표준 답변: "네, 세 군데요. 수백 가지 배치를 학습하며 탐색하는 최적화, 결과를 제안서로 풀어 쓰는 AI 컨설턴트, 향후 배차를 스스로 학습하는 강화학습. 반대로 로봇 주행 자체엔 일부러 AI를 안 썼습니다 — 안전이 필요한 곳엔 검증된 알고리즘을 쓰는 게 실제 산업 방식이거든요."

---

## 9. MAPF 솔버 전략

**v1**: Prioritized Planning + space-time 예약. 근거 — 산업 관행 충실 재현, 비동기 실행 및 VDA 5050 노드 release 모델과의 자연스러운 궁합, 타겟 규모(10~30대)에서 성능 충분.

**부록 벤치마크 (아키텍처 자랑거리)**: 플래너 인터페이스를 플러그인화하여 동일 DES 위에서 솔버를 스왑 비교. 세 축의 대표를 모아 트레이드오프 공간 전체를 측정한다.

| 솔버 | 성격 | 벤치마크 역할 |
|---|---|---|
| PP+예약 | 빠름, 불완전, 산업 관행 | v1 주인공 — 검증 대상 |
| CBS (소규모) | 최적해 보장, 밀집 시 지수 폭발 | **품질 기준점** — "PP가 최적 대비 손실 X% 이내" 정량화 |
| ECBS/EECBS | 유계 준최적(bounded-suboptimal) | 품질-속도 중간지대 |
| PIBT/LaCAM | 초고속, 준최적, 완전성 보장 | **확장성 기준점** — 100대+ 스케일 |

의미: (a) "솔버 교체가 플러그인 수준"이라는 설계 증명, (b) CBS 대비 정량화로 "PP 충분" 주장의 정량 근거 확보, (c) "초대형 플릿 확장 시 LaCAM 계열 전환 가능" 로드맵 근거.

핵심 문헌 (링크 검증됨):
- CBS: Sharon, Stern, Felner, Sturtevant, "Conflict-Based Search for Optimal Multi-Agent Pathfinding", Artificial Intelligence, 2015 (저널 원문은 ScienceDirect에서 제목 검색으로 접근)
- LaCAM: Okumura, "LaCAM: Search-Based Algorithm for Quick Multi-Agent Pathfinding", AAAI 2023 — 32×32 그리드 400 에이전트 전 인스턴스를 중앙값 1초에 해결, 완전성 보장 — https://ojs.aaai.org/index.php/AAAI/article/view/26377
- LaCAM* (anytime 최적 수렴): Okumura, "Improving LaCAM for Scalable Eventually Optimal Multi-Agent Pathfinding", 2023 — https://arxiv.org/abs/2305.03632
- 실전 검증 무대: League of the Robot Runners (Amazon Robotics 후원 lifelong MAPF 경진대회, 학계-산업 격차 해소 목적) — https://github.com/MAPF-Competition / 2023 우승팀(Overall Best) 솔루션 공개 저장소 — https://github.com/DiligentPanda/MAPF-LRR2023

주의사항: LaCAM 계열은 one-shot MAPF 기준 설계라 lifelong 창고 운영에는 RHCR류 윈도우 재계획 프레임이 필요하며, 동기화 이산 타임스텝 가정과 실제 비동기 실행 간 간극이 존재한다. 벤치마크는 이 한계를 명시한 조건 하에 수행한다.

---

## 10. 발표 전략 (일반 청중 대상)

**한 줄 정의**: "물류창고에 로봇을 들이기 전에, 가상 창고에서 미리 돌려보고 '몇 대 사야 하고 얼마가 드는지' 알려주는 서비스."

**도입 서사 (3PL 버전으로 교체)**: "중견 3PL 제안팀에 화주 RFP가 왔습니다. 마감은 2주인데, 로봇 벤더에 물어보면 견적이 4주 걸립니다. 게다가 그 견적은 그 벤더 로봇을 파는 쪽의 숫자죠. 저희 툴은 중립적인 답을 당일에 냅니다." — DES가 초 단위인 기술 특성이 "입찰 마감"이라는 비즈니스 시계와 연결되는 구조. 보조 비유: "자동차 살 때 딜러 말만 믿지 않고 시승하잖아요. 로봇 수십억어치엔 시승이 없었습니다."

**용어 번역표** (발표 전체 강제 적용):

| 전문용어 | 일반 청중용 |
|---|---|
| DES/SimPy | 수백 번 돌려보는 빠른 계산 |
| Isaac Sim SIL | 실제 물리까지 재현한 3D 가상 창고 |
| VDA 5050 | BMW가 주도해 만든, 실제 로봇 회사들이 쓰는 국제 표준 그대로 |
| P10/P50 | 장사가 안 풀리는 최악의 달에도 보장되는 숫자 |
| 구성 최적화 루프 | 사람이 3~4개 안을 볼 때 컴퓨터가 밤새 500가지를 다 해봄 |
| 가상 시운전 | 매장 오픈 전 리허설 |
| 3PL | 다른 회사들의 물류를 대신 맡아주는 회사 |

**데모 구성**: 클라이맥스 = 3D 가상 시운전 화면(로봇 10여 대 VDA 5050 오더 주행, 교차로 양보, 장애물 투척 시 정지) + 실시간 처리량 대시보드 → 엔딩 = 제안서 한 장 "이 모든 시뮬레이션이 입찰 마감 전에 이 문서 한 장이 됩니다."

**예상 질문 8종 대비**: ① 한 줄로 뭐예요 → 위 정의. ② 로봇을 만든 거예요? → "로봇이 아니라 로봇 도입 결정을 돕는 시스템." ③ 벤더한테 물어보면 되잖아요 → 도입 서사에서 선제 해소(벤더 견적의 시차·이해충돌). ④ 실제랑 같다는 걸 어떻게 믿어요 → 데모(눈) + "실제 제품과 같은 속도·규칙·통신 방식" + 어펜딕스 V&V. ⑤ 엑셀로 하면 안 돼요 → "엑셀은 로봇들이 교차로에서 막히는 걸 계산 못 합니다. 막힘이 견적을 바꿉니다." ⑥ AI 들어가요 → 8절 표준 답변. ⑦ 누가 돈 내요 → "입찰이 잦은 중견 3PL입니다. 이들에겐 검토 툴이 아니라 수주 도구라서, 건당이 아니라 구독으로 팔립니다." ⑧ 게임이랑 뭐가 달라요 → "보기엔 게임 같죠. 이 로봇들은 실제 제품과 같은 규칙으로 움직여서, 여기서 나온 숫자를 계약서에 쓸 수 있습니다."

전문가용 자료(V&V 절차, VDA 5050 아키텍처, 정합성 지표, 솔버 벤치마크)는 어펜딕스 슬라이드로 백스테이지 배치.

---

## 11. 실행 우선순위 요약

1. 스캔(SLAM) → 지도 → 그래프 에디터 → DES → 리포트의 엔드투엔드 최우선 완성 (Isaac Sim 없이 동작)
2. DES 반복 실행 + P10/P50 + 토네이도 차트 + 증설 로드맵 (공수 소, 3PL 핵심 수요 직결 — 즉시)
3. LLM 리포트 내레이션 모듈 — 출력 포맷: 화주 제안서 삽입용 섹션 (공수 소)
4. 구성 최적화 루프: 그리드 스윕 → 베이지안 최적화 (헤드라인 기여)
5. Isaac Sim: 파라메트릭 씬 + 멀티로봇 SIL — 캘리브레이션 캠페인(대표 시나리오 2~3개) + 데모 영상
6. VDA 5050 factsheet 포맷 차량 파라미터 입력 지원 (일반화 근거, 공수 소)
7. 어펜딕스 실험: AMCL 1대 검증, Nav2 비교, PP/CBS/ECBS/PIBT/LaCAM 스케일 벤치마크
8. 발표 자산: 일반 청중용 본편(3PL 서사) + 전문가용 어펜딕스 이원화

**로드맵 명기 항목**: 도면 파싱(v2), GTP 그리드 모듈, 서로게이트 모델 기반 즉답 예측, RL 배차 학습(DES gym).

---

## 부록: 근거 출처 목록 (링크 포함)

### A. 표준/방법론
| 출처 | 용도 | 링크 |
|---|---|---|
| VDA 5050 공식 사양 (GitHub, VDA/VDMA, KIT-IFL 관리) | 중앙 관제 주행 모델, factsheet, 프로토콜 | https://github.com/VDA5050/VDA5050 |
| VDA 5050 v2.1.0 영문 사양 (factsheet 장 포함) | 차량 파라미터 입력 포맷 근거 | https://github.com/VDA5050/VDA5050/blob/2.1.0/VDA5050_EN.md |
| VDA 5050 릴리스 이력 (v3.0, 2026-03) | 표준 최신 동향 | https://github.com/VDA5050/VDA5050/releases |
| VDI 3633 (시뮬레이션 V&V) | 방법론 준거 | VDI 공식(유료 규격): www.vdi.de 에서 규격번호 검색 |
| ISO 22400 / ISO 3691-4 | KPI 정의 / 무인 산업차량 안전 | ISO 공식(유료 규격): www.iso.org 에서 규격번호 검색 |

### B. 시장 조사 (2026-08 웹 검색 검증)
| 주장 | 출처 | 링크 |
|---|---|---|
| 회수기간 18~24개월 단축, 미드마켓 3PL 확산 | Dataintelo 시장 보고서 | https://dataintelo.com/report/amr-simulation-platforms-market |
| 벤더 시뮬레이션 툴 제공 관행, VDA 5050 보편화 | Robotomated (2026) | https://robotomated.com/learn/warehouse/amr-fleet-management-software |
| 인테그레이터의 시뮬레이션 영업 활용 | AGV Network | https://www.agvnetwork.com/agv-simulation-benefits |
| 플릿 사이징에서 시뮬레이션 표준 단계 | TGW Logistics | https://www.tgw-group.com/us/news/detail/agv-amr-robots-guide/ |
| FlexSim 시나리오 매니저(수동 비교 워크플로) | Autodesk FlexSim 공식 | https://www.flexsim.com/warehousing-simulation/ |
| 수동 3-시나리오 비교 실사례 | FlexSim 사례 연구 | https://www.flexsim.com/material-handling/optimizing-warehouse-throughput-while-physical-distancing/ |
| FlexSim GA 옵티마이저의 위치(부가 기능) | Autodesk 커뮤니티 | https://forums.autodesk.com/t5/flexsim-forum/warehouse-optimization/m-p/13496577 |
| GreyOrange Foundry 병렬 대량 탐색 (2026) | Logistics Business | https://logisticsbusiness.com/materials-handling/amr-agv/simulator-predicts-warehouse-performance/ |
| 도입 절차의 site survey·디지털트윈 검증 단계 | SmartLoadingHub | https://www.smartloadinghub.com/insights/agv-amr/sizing-amr-performance-cost-modern-warehouses/ |

### C. 학술 문헌
| 문헌 | 용도 | 링크 |
|---|---|---|
| Okumura, "LaCAM", AAAI 2023 | 확장성 기준 솔버 | https://ojs.aaai.org/index.php/AAAI/article/view/26377 |
| Okumura, "Improving LaCAM (LaCAM*)", 2023 | anytime 최적 수렴 | https://arxiv.org/abs/2305.03632 |
| Sharon et al., "Conflict-Based Search", AIJ 2015 | 최적성 기준 솔버 | ScienceDirect에서 제목 검색 (DOI 확인 후 부착 권장) |
| 관제 SW 공유 플릿 사이징 프레임워크 (2025) | 아키텍처 선행연구 | https://www.researchgate.net/publication/390515227_Decision-Making_Framework_For_AMR_Fleet_Size_In_Manufacturing_Environments |
| DES 기반 AMR 플릿 dimensioning DSS (2025) | 관련연구 | https://www.researchgate.net/publication/397657329_Simulation-Driven_Approach_for_Dimensioning_AMR_Fleets_in_Distribution_Centre_Logistics |

### D. 경진대회/차량
| 항목 | 용도 | 링크 |
|---|---|---|
| League of the Robot Runners (Amazon Robotics 후원) | lifelong MAPF 실전 검증 무대 | https://github.com/MAPF-Competition |
| LoRR 2023 우승팀(Pikachu) 공개 솔루션 | PIBT/LaCAM 계열 실전성 근거 | https://github.com/DiligentPanda/MAPF-LRR2023 |
| idealworks (iw.hub 제조사, BMW 스핀오프) | 기준 차량 | https://idealworks.com |

### 인용 시 주의
- B군(시장 조사)은 상업 매체·블로그 포함이므로 기획서 본문 인용 시 원문 재확인 후 접속일 병기 권장.
- "인용 전 재확인 권장"으로 표시된 항목(idealworks 제품 사양, BMW-NVIDIA 협업, CBS DOI, VDI/ISO 유료 규격)은 최신 공식 페이지 링크로 교체 후 사용.
