# AMR 경로계획 기반이론 정리 & 핵심 문헌 리스트

> AMR 견적 시뮬레이션 서비스 프로젝트의 이론적 기반 문서.
> "대수–처리량 곡선이 왜 꺾이는가"를 설명하는 이론 체계와, 학습·인용에 쓸 원문 링크를 정리한다.

---

## 0. 왜 경로계획이 견적의 핵심인가

- 대수–처리량 곡선의 한계효용 체감은 **경로 경합(congestion)** — 통로 정체, 교차로 대기, 데드락 — 때문에 발생
- 경합을 모델링하지 못하면 견적이 **낙관 편향** → "투명한 견적" 주장과 정면 충돌
- 즉 경로계획 이론은 장식이 아니라 **곡선의 모양을 결정하는 물리학**

### 용어 3계층 (혼용 금지)
| 계층 | 문제 | 프로젝트에서의 위치 |
|---|---|---|
| 1. Task Allocation | 어느 로봇이 어느 작업을 | 디스패처 파트 (규칙/휴리스틱/RL) |
| 2. Path Planning / **MAPF** | 충돌 없는 경로 집합 | **본 문서의 본체**, 견적 엔진 |
| 3. Motion Planning / Control | 속도·가감속 프로파일 추종 | Nav2 담당, 에뮬레이션 검증 계층 |

---

## 1. 핵심 기반이론: MAPF (Multi-Agent Path Finding)

그리드/그래프 위에서 N개 에이전트의 무충돌 경로 집합을 찾되, sum-of-costs 또는 makespan을 최소화하는 문제. 창고 AMR이 대표 응용이며 Amazon 물류로봇 연구가 분야를 견인.

### 알고리즘 스펙트럼 (학습 순서대로)

1. **A\*** — 단일 에이전트 최단경로. 모든 것의 출발점
2. **시공간 예약 테이블 (Space-Time Reservation)** — 시간축 포함 3차원 그리드에 경로를 "예약". 모든 방법의 기저 프리미티브. 원류는 Silver의 Cooperative A\*/WHCA\* (2005)
3. **Prioritized Planning** — 로봇에 우선순위를 매기고 앞 로봇 경로를 시공간 장애물로 취급해 순차 계획. 최적성 없음, 빠름, 실무 최다 사용. **v1 구현 타겟**
4. **CBS (Conflict-Based Search)** — 최적 알고리즘의 대표. 충돌 발생 시 제약을 추가해 Conflict Tree를 분기하는 2단 구조. 로봇 수 증가 시 폭발
5. **ECBS / EECBS** — 유계 준최적(bounded-suboptimal) 변형. 실용선
6. **Lifelong MAPF** — 표준 MAPF의 일회성 가정을 깨고, 도착 즉시 새 작업이 재발급되는 **연속 운영** 문제. 창고의 실제 조건
7. **RHCR (Rolling-Horizon Collision Resolution)** — lifelong 문제를 "앞 W스텝만 충돌 해소 + 주기적 재계획"으로 분해. 대규모 창고 플릿의 사실상 표준 프레임이자 **본 프로젝트의 직접 기반 문헌**

---

## 2. 결합 문제: TAPF

할당(1층)과 경로(2층)를 분리해서 풀면 준최적 — "가장 가까운 로봇"이 경합 구간을 지나야 한다면 실제로는 먼 로봇이 빠를 수 있음. 이 결합 최적화가 TAPF(Combined Target Assignment and Path Finding)로 정립.

- v1 절충 설계: 분리해서 풀되, **경합 페널티를 포함한 경로 비용을 할당 비용행렬에 피드백**
- RL 디스패처 상태에 경합 정보를 포함시키는 것도 같은 맥락

---

## 3. 데드락 이론 (AGV 고전 문헌)

MAPF가 충돌(collision)을 다룬다면, 좁은 통로·단방향 구간의 교착(deadlock)은 별도 이론. 1990년대 AGV 시스템 문헌 + OS 자원 할당 이론이 원류.

- **Zone Control**: 구역당 1대만 허용 (반도체 팹 OHT 방식). 단순·보수적
- **Banker's Algorithm 계열**: 구역 요청 시 안전 상태 검사 후 승인
- 프레임: **예방(prevention) / 회피(avoidance) / 탐지·복구(detection & recovery)** 3분류
- 시뮬레이션 지표 "데드락 발생 횟수"의 이론적 근거

---

## 4. SimPy 계층의 집계 이론: 대기행렬 & 흐름

**결정 사항**: 360회 스윕에서 매 틱 CBS는 불가 → SimPy 계층은 두 선택지
- **(a) 경량 MAPF**: Prioritized Planning + 예약 테이블 — DES 내에서 충분히 빠르고 경합이 창발. **기본 채택**
- **(b) 흐름 근사**: 통로를 용량 있는 SimPy `Resource`로, 정체를 대기행렬로 근사

(a)를 채택하되 **(b)의 언어로 결과를 해석**:
- **대기행렬 이론**: M/M/c, Jackson network
- **Little's Law** (L = λW): 시뮬레이션 산출 리드타임의 검산 도구 → VDI 3633 verification 활동에 연결
- **교통류의 fundamental diagram** (흐름–밀도 관계): "밀도가 임계치를 넘으면 처리량이 오히려 감소" = 대수–처리량 포화의 이론적 설명

---

## 5. 핵심 문헌 리스트 (링크 포함)

### 필독 (프로젝트 직결)

**① CBS — 이 분야의 관문 논문**
- Sharon, Stern, Felner, Sturtevant. "Conflict-Based Search for Optimal Multi-Agent Pathfinding." *Artificial Intelligence*, vol. 219, pp. 40–66, 2015. DOI: 10.1016/j.artint.2014.11.006
- 원문(무료): https://digitalcommons.du.edu/computer_science_faculty/7/
- 초기 AAAI 버전: https://ojs.aaai.org/index.php/AAAI/article/view/8140

**② RHCR / Lifelong MAPF — 본 프로젝트의 직접 기반**
- Li, Tinka, Kiesel, Durham, Kumar, Koenig. "Lifelong Multi-Agent Path Finding in Large-Scale Warehouses." AAAI 2021, pp. 11272–11281.
- arXiv: https://arxiv.org/abs/2005.07371
- AAAI 공식: https://ojs.aaai.org/index.php/AAAI/article/view/17344
- 1,000대 규모 창고 시뮬레이션에서 검증 — 스윕 규모 상한의 참고점

**③ MAPF 정의·변형·벤치마크 — 용어와 벤치마크의 표준**
- Stern et al. "Multi-Agent Pathfinding: Definitions, Variants, and Benchmarks." SoCS 2019, pp. 151–158.
- arXiv: https://arxiv.org/abs/1906.08291
- 공식: https://ojs.aaai.org/index.php/SOCS/article/view/18510
- 자체 구현을 공개 벤치마크로 검증 → 또 하나의 V&V 스토리

### 확장 (면접 깊이용)

**④ TAPF — 할당·경로 결합 최적화**
- Ma, Koenig. "Optimal Target Assignment and Path Finding for Teams of Agents." AAMAS 2016, pp. 1144–1152.
- arXiv: https://arxiv.org/abs/1612.05693
- CBM(Conflict-Based Min-Cost-Flow): 하위층 min-cost max-flow 할당 + 상위층 CBS 충돌 해소

**⑤ EECBS — 유계 준최적 실용선**
- Li, Ruml, Koenig. "EECBS: A Bounded-Suboptimal Search for Multi-Agent Path Finding." AAAI 2021.
- arXiv: https://arxiv.org/abs/2010.01367

**⑥ Lifelong Pickup & Delivery — 창고 작업 모델의 원형**
- Ma, Li, Kumar, Koenig. "Lifelong Multi-Agent Path Finding for Online Pickup and Delivery Tasks." AAMAS 2017.
- arXiv: https://arxiv.org/abs/1705.10868

### 링크 미확정 (제목·키워드로 검색)

- **Silver (2005)** "Cooperative Pathfinding." AIIDE 2005 — WHCA\*, 시공간 예약의 원류. 검색: `Silver cooperative pathfinding WHCA`
- **ECBS**: Barer, Sharon, Stern, Felner. "Suboptimal Variants of the Conflict-Based Search Algorithm..." SoCS 2014
- **PRIMAL2**: 학습 기반 lifelong MAPF (분산 RL 정책) — RL 디스패처 확장 시 참고. 검색: `PRIMAL2 lifelong MAPF`
- **AGV 데드락**: 검색 키워드 `AGV deadlock avoidance zone control`, `Reveliotis AGV deadlock`
- 커뮤니티 허브: **mapf.info** — 논문·벤치마크·코드 집대성

---

## 6. 학습 로드맵 (이론 → 구현 직결 순)

| 순서 | 항목 | 소요 | 산출물 |
|---|---|---|---|
| 1 | A\* + 시공간 예약 테이블 | 주말 1회 | 단일/2대 데모 구현 |
| 2 | Prioritized Planning | 1주 | **v1 SimPy 내장 플래너** |
| 3 | CBS 논문(①) 정독 | 1주 | 이론 베이스, 면접 대비 |
| 4 | RHCR(②) + 벤치마크(③) | 1~2주 | windowed 재계획 구조 반영 |
| 5 | Little's Law + M/M/c | 반나절 | 결과 검산·해석 언어 |
| 6 | (선택) TAPF(④), 데드락 이론 | 여유 시 | 면접 심화 |

---

## 7. 프로젝트 서사와의 연결

- 현상 수준: "AMR을 늘렸더니 처리량이 포화됐다"
- 이론 수준: **"이것은 MAPF 문헌에서 정립된 경합 구조이며, 우리 모델의 포화 거동은 lifelong MAPF 연구(RHCR) 및 교통류 이론과 정합한다"**
- 검증 수준: 공개 MAPF 벤치마크 + Little's Law 검산 + Isaac Sim 교차검증 = VDI 3633 V&V 체계의 구체적 실행

다음 설계 과제: SimPy 이산사건 세계에서 Prioritized Planning + 예약 테이블을 어떻게 구현할지 — 연속 시간 vs 틱 기반의 절충 설계.
