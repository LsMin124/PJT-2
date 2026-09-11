# AMR 견적 서비스 — 전체 흐름: 버튼에서 로봇까지 (2026-09-11)

> 프론트의 **견적 요청** 버튼 한 번이 어떤 데이터를 만들어, 어떤 경로로, 어떤 프로세스를 거쳐, 결국 Isaac Sim 안의 로봇 바퀴를 돌리기까지를 순서대로 설명한다.
>
> **대상 독자**: 이 프로젝트를 처음 보는 사람. 용어와 배경부터 설명한다.
> **기준**: 팀 레포 `S15P21A106` 브랜치 `develop` @ `cc3fa62`. 전부 코드를 읽어 확인한 값이다.
> **원천 계약**: `SIM_RUN_CONTRACT v3.1` · `SIM_VERIFY_CONTRACT v0.3` · `MQTT_CONTRACT v1.2`

---

## 0. 이 문서를 읽는 법

이 시스템은 **프로세스 7종이 서버 2대에 흩어져, 브로커 2개로 통신한다.** 코드를 아무 데나 열면 "이게 누구한테서 온 메시지지?"에서 막힌다. 그래서 순서를 이렇게 잡았다.

| 절 | 내용 | 이미 아는 사람은 |
|---|---|---|
| 1 | 제품이 뭘 하는 물건인지 | 건너뛰기 |
| 2 | **용어 사전** — AMR·DES·PIBT·θ·VDA 5050·Sparkplug 등 | 건너뛰기 |
| 3 | 시스템 지도 — 프로세스가 어디서 돌고 왜 브로커가 둘인지 | 필독 |
| 4 | **흐름 01~10** — 본론 | 필독 |
| 5 | 지금 끊겨 있는 6곳 | 필독 |
| 6 | 부록 — 상수표·파일 색인 | 참조용 |

---

## 1. 제품이 하는 일

물류센터(창고)를 가진 회사가 **"우리 창고에 자율주행 로봇(AMR)을 몇 대나 넣어야 하나?"** 를 묻는다. 로봇 업체에 물으면 자기네 물건을 많이 팔고 싶은 쪽이 답을 준다. 이 서비스는 그걸 **중립적인 시뮬레이션으로** 답한다.

동작은 2단계다.

**1단계 — 견적 (DES)**
창고 조건(작업자 수, 입·출고 물량, 운영 시간, 예산)을 받아, **로봇 3대·4대·5대 … 15대** 를 각각 따로 시뮬레이션한다. 대수마다 "하루에 몇 건 처리하나 / 로봇이 얼마나 노나 / 얼마 드나"가 나온다. 그 곡선에서 목표를 만족하는 **최소 대수**를 권장한다.
이건 순수 계산이라 빠르다 — 13번 돌려도 2분 안이다.

**2단계 — 물리 검증 (SIL)**
1단계는 "로봇이 1초에 한 칸 간다" 같은 추상 모델이다. 실제로는 가속·감속·회전·충돌 회피 때문에 더 느리다. 그래서 권장 대수를 **NVIDIA Isaac Sim** 안에 실제 물리 엔진으로 띄워, 진짜 로봇이 그 창고에서 그 물량을 감당하는지 10분간 돌려본다.

> 이 2단계 구조가 실제 로봇 벤더의 도입 프로세스(현장 매핑 → 플릿 설계 → 가상 시운전 → 견적)를 재현한 것이다. 자세한 기획 배경은 `AMR_기획_기준선_v3.md` 참조.

---

## 2. 용어 사전

처음 보면 막히는 것부터.

### 2.1 도메인 용어

| 용어 | 뜻 |
|---|---|
| **AMR** | Autonomous Mobile Robot. 자율주행 물류 로봇. 이 프로젝트는 idealworks 사의 **iw.hub** 모델을 쓴다. 선반형이라 위에 토트를 싣는다. |
| **토트(tote)** | 물건을 담는 플라스틱 상자. 로봇이 옮기는 최소 단위다. `tote_units=8` = 토트 하나에 상품 8개. `capacity=6` = 로봇이 한 번 왕복할 때 최대 토트 6개(3단 선반 × 2). |
| **출고 / 입고** | 출고 = 랙에서 꺼내 포장대로 (주문 처리). 입고 = 입고 버퍼에서 랙으로 (재고 보충). |
| **랙(rack)** | 물건이 꽂혀 있는 선반. `A2`, `B7` 같은 ID를 갖는다. |
| **aisle_buf** | 통로 버퍼. 로봇이 랙 앞에서 물건을 받는 정차 지점. 이 창고엔 22곳 있다. |
| **packing** | 포장대. 로봇이 출고 토트를 내려놓는 곳. 5곳. |
| **충전 존(charge zone)** | 로봇이 쉬거나 충전하는 구역. 이 창고의 east 존 1개뿐이다. |
| **처리량(throughput)** | 시간당 처리한 주문/태스크 수. |
| **가동률(utilization)** | 로봇이 실제로 일한 시간 비율. 놀고 있으면 낮다. |

### 2.2 시뮬레이션 용어

| 용어 | 뜻 |
|---|---|
| **DES** | Discrete Event Simulation. 이산 사건 시뮬레이션. 물리를 안 쓰고 "1틱에 한 칸 이동" 같은 규칙으로 계산한다. 빠르지만 낙관적이다. |
| **SIL** | Software-In-the-Loop. 실제 제어 소프트웨어를 물리 시뮬레이터에 연결해 돌리는 것. 여기선 Isaac Sim + ROS 2 에이전트 조합이다. |
| **틱(tick)** | DES 의 시간 단위. 이 프로젝트에서 **1틱 = 1초** (`DT = 1.0`). 로봇이 1틱에 한 칸(1 m) 간다. |
| **칸(cell)** | 창고 바닥을 1 m 격자로 자른 한 칸. 맵 전체는 107 × 116 칸이다. |
| **PIBT** | Priority Inheritance with Backtracking. 다중 로봇 경로계획 알고리즘. 로봇들이 서로 막히지 않게 매 틱 우선순위를 정해 한 칸씩 움직인다. |
| **헤딩 모델** | 로봇이 방향(heading)을 갖는 모델. 90° 돌려면 1틱을 쓴다. 반대는 "칸 모델"(점로봇, 방향 없음). 이 프로젝트 기본은 헤딩 모델이다. |
| **θ (theta)** | 통행 비용을 조절하는 **11개 숫자**. "교차로는 비싸게", "통로 역주행은 비싸게" 같은 규칙의 강도를 담는다. Optuna 로 최적값을 찾아 JSON 파일로 저장해 둔다. |
| **STALL** | 정체. 1500틱 동안 아무 태스크도 완료되지 않으면 판정한다. |
| **makespan** | 전체 작업을 끝내는 데 걸린 시간. |

### 2.3 통신 용어

| 용어 | 뜻 |
|---|---|
| **AMQP / RabbitMQ** | 작업 큐 프로토콜/브로커. "이 일 좀 해줘"를 넣어두면 워커가 하나씩 꺼내 처리한다. **신뢰성 중시**, 메시지가 사라지지 않는다. |
| **MQTT / EMQX** | 경량 발행-구독 프로토콜/브로커. **실시간성 중시**. 센서·로봇처럼 자주 작은 메시지를 뿌리는 데 쓴다. |
| **VDA 5050** | 독일 자동차공업협회가 만든 **AMR 표준 통신 규약**. "로봇에게 어디로 가라"를 벤더 중립적으로 표현한다. 토픽은 `uagv/v2/{제조사}/{시리얼}/order` 같은 모양이다. |
| **Sparkplug B** | MQTT 위에 얹는 **산업용 상태 관리 규약**. 노드가 살았는지 죽었는지(NBIRTH/NDEATH), 값이 바뀌었는지(NDATA)를 protobuf 로 주고받는다. 연결이 끊기면 브로커가 대신 유언(LWT)을 뿌려준다. |
| **SSE** | Server-Sent Events. 서버가 브라우저로 단방향 실시간 푸시를 보내는 HTTP 기술. WebSocket 보다 단순하다. 이 프로젝트는 진행 상황 표시에 쓴다. |
| **ROS 2** | 로봇 운영체제. 노드들이 토픽(`/cmd_vel`, `/odom`)으로 통신한다. |
| **`/cmd_vel`** | ROS 표준 속도 명령 토픽. `linear.x`(전진 속도) + `angular.z`(회전 속도)를 담는다. 결국 **모든 명령은 여기로 수렴한다.** |

### 2.4 이 프로젝트 고유 이름

| 이름 | 정체 |
|---|---|
| **sim_engine** | DES 커널. PIBT + 배차 + 배터리. 견적과 물리 검증이 **같은 커널**을 쓴다. |
| **estimate_worker** | 견적 워커. RabbitMQ 에서 요청을 꺼내 DES 를 돌린다. |
| **fms-core** | Fleet Management System 코어. DES 커널을 감싸고, 실 로봇에게 VDA 5050 명령을 내린다. |
| **host-orch** | 오케스트레이터. RabbitMQ 와 EMQX 를 잇는 **유일한 다리**. |
| **sim-runner-gpu01** | GPU 서버에 상주하는 데몬. 명령을 받으면 Isaac 과 로봇 에이전트를 띄운다. |
| **t4_agent** | ROS 2 패키지. VDA 5050 명령을 `/cmd_vel` 로 번역한다. |

---

## 3. 시스템 지도

### 3.1 프로세스가 어디서 도는가

```
┌─ 브라우저 ────────────┐
│  React 19 + Vite      │
│  01 창고  02 대수      │
│  03 런    04 검증      │
│  05 견적               │
└───────┬───────────────┘
        │ HTTPS + SSE
        ▼
┌─ EC2 (j15a106.p.ssafy.io) ───────────────────────┐
│  호스트 nginx :443    ← 유일한 공개 진입          │
│  a106-frontend        정적 파일만                 │
│  a106-backend :8080   Spring Boot 3.3.5 / Java 21 │
│  a106-db              PostgreSQL 16               │
│  a106-rabbitmq :5672  ★ 잡 큐 브로커              │
│  견적 워커 × 2         DES 실행                    │
│  EMQX :1883           ★ 실시간 브로커              │
│  host-orch            두 브로커의 다리             │
│  fms-core             VDA 5050 마스터              │
│  a106-minio :9002     영상 저장소 (올리는 쪽 없음) │
└───────┬───────────────────────────────────────────┘
        │ MQTT over tailnet
        ▼
┌─ GPU 박스 (RTX 5080, 64GB) ──────┐
│  sim-runner-gpu01   상주 데몬     │
│  Isaac Sim          물리 시뮬     │
│  t4_agent × N       ROS 2 노드    │
└───────────────────────────────────┘
```

배포 방식:
- **Jenkins 자동**: 백엔드 · 프론트 · 견적 워커 · EMQX (`release` 브랜치 머지 시)
- **수동**: 호스트 nginx · host-orch · fms-core · GPU 박스 전체

### 3.2 왜 브로커가 둘인가

이게 이 시스템에서 가장 헷갈리는 지점이다. **성격이 완전히 다른 두 종류의 통신**이 있기 때문이다.

| | RabbitMQ (AMQP) | EMQX (MQTT) |
|---|---|---|
| 무엇을 나르나 | "로봇 5대로 시뮬 돌려줘" 같은 **작업 요청** | "지금 x=12.3, y=45.6 에 있다" 같은 **상태 보고** |
| 빈도 | 견적 1회당 13건 | 로봇 1대당 초당 수 건 |
| 잃어버리면 | **큰일** — 사용자가 결과를 못 받는다 | 괜찮다 — 0.5초 뒤 또 온다 |
| 보장 | durable 큐, ack 확인, 재전달 | QoS 0 (fire-and-forget) 위주 |
| 쓰는 곳 | 웹 ↔ FMS | FMS ↔ 로봇 |

**두 브로커는 서로를 전혀 모른다.** `host-orch` 하나만 양쪽에 동시에 붙어 있고, 그래서 이 프로세스가 죽으면 물리 검증은 통째로 멈춘다.

### 3.3 큐와 토픽 목록

**RabbitMQ 큐 4개** (전부 durable)

| 큐 | 방향 | 내용 |
|---|---|---|
| `sim.request` | BE → 견적 워커 | 대수 하나짜리 DES 실행 요청 |
| `sim.result` | 견적 워커 → BE | KPI 14키 또는 실패 |
| `sim.verify.request` | BE → host-orch | 물리 검증 런 요청 |
| `sim.verify.result` | host-orch → BE | 검증 KPI + 영상 URL(예정) |

**EMQX 토픽** (요약)

| 토픽 | 규약 | 방향 |
|---|---|---|
| `spBv1.0/santa/NCMD/{노드}` | Sparkplug | host-orch → 노드 (명령) |
| `spBv1.0/santa/{NBIRTH,NDATA,NDEATH}/{노드}` | Sparkplug | 노드 → host-orch (상태) |
| `spBv1.0/STATE/host-orch` | Sparkplug | host-orch 생사 (retained) |
| `uagv/v2/idealworks/{시리얼}/order` | VDA 5050 | fms-core → 로봇 |
| `uagv/v2/idealworks/{시리얼}/state` | VDA 5050 | 로봇 → 전원 |
| `biz/v1/event` | 자체 | fms-core → host-orch (KPI 등) |

EMQX 는 **계정별 ACL** 이 걸려 있다. 로봇 `amr01` 은 자기 토픽에만 쓸 수 있고, 남의 토픽에 쓰면 브로커가 연결을 끊어버린다(`deny_action = disconnect`).

---

## 4. 흐름 — 버튼에서 로봇까지

각 단계는 **무엇이 / 왜 / 데이터 / 주의**로 적었다.

---

### 01. 사용자가 창고 조건을 입력한다

**무엇이.** 화면은 라우터 없이 한 페이지를 스크롤하는 **5스텝** 구조다.

| 스텝 | 이름 | 하는 일 |
|---|---|---|
| 01 | WAREHOUSE | 창고 운영 조건 입력 |
| 02 | FLEET | 로봇 대수 범위 선택 → **요청 버튼** |
| 03 | RUNS | 진행 상황 실시간 표시 |
| 04 | VERIFY | 물리 검증 영상 |
| 05 | QUOTE | 최종 견적 리포트 |

입력 항목 9개:

| 입력 | 기본값 | 단위 | 의미 |
|---|---|---|---|
| 작업자 수 | 4 | 명 | 창고 인력 |
| 입고 평균 | 480 | 건/일 | 하루 입고 건수 |
| 출고 평균 | 500 | 건/일 | 하루 출고 주문 수 |
| 입고 창 | 01:00–09:00 | HH:mm | 입고가 들어오는 시간대 |
| 출고 창 | 13:00–18:00 | HH:mm | 주문이 몰리는 시간대 |
| 목표 처리율 | 95 | % | 주문의 몇 %를 처리해야 하나 |
| 예산 | 30,000 | 만원 | 도입 예산 상한 |
| 대수 범위 | 3–15 | 대 | 시뮬레이션할 구간 (상한 28) |

AMR 모델은 화면이 뜰 때 `GET /api/robots` 로 목록을 받아 첫 항목(`iw.hub`)을 자동 선택한다.

**왜.** 이 9개가 DES 시뮬레이션의 입력 전부다. 창고 도면은 이미 고정돼 있고(실측 맵 1종), 사용자가 바꾸는 건 "얼마나 바쁜 창고인가"와 "얼마까지 쓸 수 있나"뿐이다.

**주의.**
- 상태관리 라이브러리가 없다. `App.jsx` 의 `useState` 7개 + `sessionStorage['estimateId']` 가 전부다.
- **죽은 버튼 2개**: `주문 로그 CSV 업로드`, `견적서 PDF 저장` — `onClick` 핸들러가 아예 없다. 눌러도 아무 일도 안 난다.

---

### 02. `견적 요청 — 13 RUNS ↗` 를 누른다

**무엇이.** 파이프라인을 시작하는 버튼은 **이것 하나뿐**이다. 누르면 HTTP 요청이 **순차로 2건** 나간다. 전송 네이밍은 전부 snake_case 다 (프론트가 수동 매핑, 백엔드는 `@JsonNaming(SnakeCaseStrategy)`).

#### 요청 1/2 — `POST /api/warehouses` → 201

```json
{
  "worker_count": 4,
  "inbound_start": "01:00",  "inbound_end": "09:00",
  "outbound_start": "13:00", "outbound_end": "18:00",
  "inbound_avg": 480,  "outbound_avg": 500,
  "target_throughput": 95,
  "budget_limit": 300000000
}
```

`budget_limit` 은 화면의 "만원"에 10,000을 곱한 **원 단위**다. 변환은 프론트 `App.jsx:60` 한 곳에서만 일어난다.

**보내지 않는 5개 키**는 백엔드 기본값이 채운다:

| 키 | 기본값 | 비고 |
|---|---|---|
| `name` | `"품고"` | 창고 이름 |
| `width` / `length` | 110.0 / 90.0 | m. 실측 맵 크기 |
| `outbound_rate` | 0.0 | 미사용 |
| `target_utilization` | 80.0 | 목표 가동률 % |

저장할 때 백엔드가 `area = width × length` 를 계산한다. 시간 값은 컬럼이 `TIMESTAMP` 라서 `LocalTime` 을 **2026-01-01 에 앵커**해 저장한다(날짜는 의미 없음).

> ⚠️ **누를 때마다 창고 행이 새로 생긴다.** 재사용·조회 로직이 없어서 같은 조건으로 세 번 누르면 `warehouses` 테이블에 행이 세 개 쌓인다.

#### 요청 2/2 — `POST /api/estimates` → 202

```json
{"warehouse_id": 12, "amr_model": "iw.hub", "fleet_min": 3, "fleet_max": 15}
```

목표 처리율·가동률·예산은 **보내지 않는다.** 이미 1단계에서 창고에 들어갔으므로, 백엔드가 창고 행에서 읽어 쓴다.

**백엔드가 순서대로 하는 일** (`EstimateService.create()`, 전체가 하나의 `@Transactional`):

1. 창고 조회 — 없으면 `404 EST404_1`
2. 목표 3개 확정 — 요청값 우선, null 이면 창고값
3. `Estimate` 행 저장 — `status = RUNNING`, 권장대수·무릎·사유는 전부 null
4. **대수 목록 산출** — 아래 설명
5. `SimScenario` 행 13개 저장 — `status = QUEUED`, 비용은 이 시점에 확정
6. **RabbitMQ 에 13건 발행**
7. `{estimate_id, max_amr_count, run_ids[]}` 응답

**대수 산출 규칙** — 분기가 둘이다.

| 분기 | 조건 | 계산 |
|---|---|---|
| A | `fleet_min`·`fleet_max` 중 하나라도 없음 | `1 ~ (예산 ÷ 15,500,000 + 1)`, 상한 28 |
| B | 둘 다 있음 | `fleet_min ~ min(fleet_max, 28)` |

프론트가 항상 둘 다 보내므로 **현재 UI 로는 분기 A 에 도달할 수 없다.** 즉 **예산은 대수 상한에 영향을 주지 않고**, 나중에 `BUDGET_EXCEEDED` 판정에만 쓰인다.

**비용 계산**:
```
충전기 수 = min(ceil(대수 / 4), 6)
비용      = (AMR 1,500만 + 설치 50만) × 대수 + 충전기단가 × 충전기수
          = 15,500,000 × 대수          ← 충전기 단가가 현재 0이라
```

**상한 28의 근거**: 설정 파일엔 `estimate.max-amr-count: 28` 로 박혀 있는데, 진짜 근거는 **충전 존 용량**이다. 창고 east 존의 통행 가능 칸 106개에 체비쇼프 간격 2로 홈을 깔면 정확히 28개가 나온다. 29대째를 시도하면 시뮬이 `assert "충전 존 용량 부족"` 으로 죽는다.

---

### 03. RabbitMQ 에 `sim.request` 13건이 쌓인다

**무엇이.** 기본 익스체인지 `""` + 라우팅키 = 큐 이름으로 직배달한다(exchange·binding 선언 없음). durable 큐, Jackson snake_case 직렬화.

```json
{
  "run_id": "est_77_n3",
  "seed": 101,
  "horizon_s": 21600,
  "shifts": "in,in,out,out,out,out",
  "inbound_gap_s": 60.0,
  "outbound_gap_s": 36.0,
  "warehouse": {
    "worker_count": 4,
    "inbound_start": "01:00",  "inbound_end": "09:00",
    "outbound_start": "13:00", "outbound_end": "18:00",
    "inbound_avg": 480, "outbound_avg": 500
  },
  "amr": {
    "count": 3,
    "speed_empty": null, "speed_loaded": null, "consumption": null,
    "battery": null, "charge_time": null, "charge_threshold": null, "capacity": null
  },
  "inbound": [],
  "outbound": []
}
```

**각 필드가 어디서 왔나**:

| 필드 | 출처 | 설명 |
|---|---|---|
| `run_id` | 백엔드 생성 | `est_{견적id}_n{대수}` — 전 시스템에서 이 런을 가리키는 유일한 이름 |
| `seed` | `FIXED_SEED = 101` 상수 | 재현성. 같은 입력 → 같은 결과 |
| `horizon_s` | 설정 `estimate.horizon: 6h` | 시뮬 지평선 21,600초 |
| `shifts` | 설정 | 지평선을 6등분해 앞 2칸은 입고, 뒤 4칸은 출고 |
| `inbound_gap_s` | 계산 | 입고창 28,800초 ÷ 480건 = **60.0** (평균 도착 간격) |
| `outbound_gap_s` | 계산 | 출고창 18,000초 ÷ 500건 = **36.0** |
| `amr.count` | 이 run 의 대수 | **13번 발행되며 3→15 로 바뀌는 유일한 값** |
| `amr.*` 나머지 | 전부 null | 의도된 것 — 시뮬이 자체 기본값을 쓴다 |
| `inbound`/`outbound` | 항상 빈 배열 | 주문 목록을 안 보낸다 → 워커가 직접 만든다 |

**왜 13건인가.** 대수별로 독립 시뮬이라 병렬로 나눠 돌릴 수 있다. 한 건에 다 넣으면 워커 하나가 순차로 돌아야 한다.

**주의 — 발행이 DB 커밋보다 먼저다.**
`@Transactional` 안에서 큐 발행이 6단계, 커밋은 메서드가 끝날 때다. 워커가 아주 빠르면 커밋 전에 결과를 보낼 수 있고, 그러면 백엔드가 `run_id` 로 행을 못 찾아 **조용히 버린다**. 그 건은 30분 뒤 `sweepTimeouts()` 스케줄러가 `TIMEOUT` 으로 회수한다.
반대로 발행이 실패하면 `503 SIM503` 이 올라가 트랜잭션이 통째로 롤백된다 — 견적도 run 행도 남지 않는다.

---

### 04. 견적 워커가 DES 를 돌린다

**무엇이.** EC2 에 워커 컨테이너 **2개**가 떠 있다. 둘 다 같은 `sim.request` 큐를 `prefetch_count=1` 로 구독하므로, 브로커가 라운드로빈으로 한 건씩 나눠준다.

워커당 1코어를 강제한다 — `OMP_NUM_THREADS=1` 등 + compose `cpus: "1.0"`. 시뮬이 결정론이라 어느 워커가 돌려도 KPI 는 같다.

| 워커 수 | 13 runs 소요 | 배속 |
|---|---|---|
| 1 | 407.6 s | 1.0× |
| 5 | 121.7 s | **3.3×** |

5배가 안 나오는 이유는 대수별 run 시간 편차(3~37초)의 꼬리다. 현재 `replicas` 는 **2** — EC2 가 4코어라 백엔드·DB·브로커 몫 2개를 남긴 값이다.

#### ① 주문을 워커가 직접 만든다

요청의 `outbound[]` 가 비어 있으므로 워커가 3단계로 생성한다:

1. **생성** — 프로파일(`order_rules.json`) + 상품 마스터로 하루치 `outbound_avg`(500)건을 만든다. `release="wave"` 방식이라 시간대별로 몰린다.
2. **재배치** — 하루치 시각 구간을 `outbound_start~end`(13:00–18:00)로 **선형 비례 매핑**한다.
3. **자르기** — horizon 밖 주문을 버리고 시각을 0 기준으로 옮긴다.

결과를 `/tmp/orders_XXXX.json` 에 써서 `--orders-file` 로 넘기고, 끝나면 `finally` 에서 지운다.

각 주문은 이렇게 생겼다:
```json
{"order_id": "...", "store_id": "...", "ordered_at_s": 1420,
 "lines": [{"product_id": "...", "rack_id": "A3", "qty": 12}, ...]}
```

시뮬은 `rack_id` 를 `rack_buffers.json` 표로 **정차 지점(`aisle_buf[i]`)** 으로 바꾸고, 버퍼별 `qty` 합을 `tote_units`(8)로 나눠 **토트 수**를 계산한다. 토트 합이 6을 넘는 주문은 시뮬 밖(팔레트 흐름)이라 제외된다.

#### ② 시뮬을 서브프로세스로 부른다

```bash
python sim_v2_tasks.py est_77_n3 3 0 42 1 \
  --flow=main --theta=/app/infra/out/theta_wallA_r24_g15.json \
  --horizon=21600 --shifts=in,in,out,out,out,out \
  --order-gap=36 --inbound-gap=60 \
  --orders-file=/tmp/orders_XXXX.json \
  --svc-per-tote=5 --capacity=1 --tote-units=8
```

**왜 in-process 가 아니라 서브프로세스인가**: `sim_v2_tasks.py` 는 최상위에서 `argv` 를 읽는 스크립트이고, 극단적인 θ 값이 들어오면 정체가 나 영원히 안 끝날 수 있다. 그래서 **wall-clock 으로 프로세스째 끊을 수단**이 필요하다. 타임아웃 600초.

**워커가 넘기지 *않는* 플래그가 곧 조건이다.** `--cell`, `--fixed-homes`, `--battery`, `--no-lanes`, `--no-yield`, `--map-dir` 이 전부 빠지므로 기본값으로 돈다:

| 조건 | 값 | 의미 |
|---|---|---|
| 이동 모델 | 헤딩 PIBT | 2칸 차체, 회전 1틱 |
| 홈/복귀 | 충전 존 | 일 없으면 east 존으로 |
| 차선 | ON, gain 2.0 | 역주행에 비용 2배 |
| 양보 | ON | 정면 조우 시 한쪽이 피함 |
| aisle_block | 1 | 랙 사이 좁은 통로 로봇 금지 |
| 배터리 | **OFF** | → `charge_downtime_s` 는 항상 0 |

#### ③ 시뮬이 읽는 재료

| 파일 | 무엇이 되나 |
|---|---|
| `occupancy_grid.npy` | 0.1 m 격자를 1 m 로 max-pool(보수적) → 통행 가능 **(107, 116) 중 8,640칸** |
| `stations.json` | aisle_buf 22 · packing 5 · charger 6 · input/output 각 2 … 의 실좌표 |
| `charge_zone.json` | east 존 사각형 1개 → 격자로 바꾸면 free 106칸 |
| `rack_buffers.json` | `rack_id("A3") → aisle_buf[i]` **정확 일치** 표. 이게 없으면 명시 주문 모드가 죽는다 |
| `theta_wallA_r24_g15.json` | θ 11개. 로봇 24대·간격 15 조건에서 Optuna 150 trial 로 뽑은 값 |

#### ④ θ 가 하는 일

θ 11개가 만드는 것은 **`edge_cost` 배열 (H, W, 4)** 이다 — "칸 (r,c) 에서 방향 d 로 나가는 비용".

조립 규칙:
1. 전부 1.0 에서 시작
2. 다음 4그룹 중 **먼저 걸리는 하나만** 곱한다 (우선순위 순):
   `station_spur` → `loop` → `junction` → `aisle_interior`
3. `main_aisle_gain` 은 큰 통로 간선에 **항상 따로** 곱한다
4. **θ 밖 운영값** `lane_gain`, `pocket_gain` (둘 다 2.0) 을 곱한다
5. `clip(0.5, 5.0)`

`wait_cost` (H, W) 는 전 칸 동일값(제자리 대기 비용)이다.

**`lane_dir` 은 θ 가 아니다.** 버퍼 주머니 줄을 기준으로 ±1 칸이 **출발 차선**, ±3 칸이 **복귀 차선**이다. 역주행 간선에 2배 벌금을 물리고, 양보할 때 대피 후보 칸 정렬에도 쓰인다.

| # | θ 필드 | 범위 | 의미 |
|---|---|---|---|
| 1 | `aisle_alt_gain` | 1.0–3.5 | 통로 역주행 벌금 |
| 2 | `aisle_alt_boost` | 0.5–1.0 | 통로 순방향 할인 |
| 3 | `loop_gain` | 1.0–3.0 | 순환로 역주행 벌금 |
| 4 | `main_aisle_gain` | 0.5–1.2 | 큰 통로 주행 배율 |
| 5 | `junction_penalty` | 1.0–3.0 | 교차로 진입 벌금 |
| 6 | `station_in` | 0.5–1.0 | 스테이션 접근 할인 |
| 7 | `station_out` | 1.0–3.0 | 스테이션 이탈 역주행 벌금 |
| 8 | `wait_cost` | 0.5–3.0 | 제자리 대기 비용 |
| 9 | `aisle_period` | 1·2·3 | 몇 통로마다 방향 반전 |
| 10 | `aisle_phase` | 0·1 | 첫 통로 북/남 |
| 11 | `loop_dir` | 0·1 | 순환 방향 CW/CCW |

> aisle 계열 4개는 `aisle_block=1` 조건에서 적용될 간선이 0개라 중립값으로 고정돼 있다.

#### ⑤ 커널 한 틱이 하는 일

```
arrivals(t)  →  이 틱에 도착한 주문을 태스크로 접수

dispatch(t)  :  1. 서비스(도킹) 끝난 로봇 처리
                   픽업 완료 → 하역지로 / 하역 완료 → 태스크 완료 + 후속 태스크 즉시 접수
                2. 배차 — 대기 태스크를 FIFO 로 돌며 맨해튼 최근접 로봇 배정
                   capacity > 1 이면 같은 출발지 태스크를 동승(riders)으로 흡수
                3. 남은 놀고 있는 로봇을 충전 존으로

move(t)      :  4. 헤딩 PIBT 한 스텝 (우선순위 → 거리장 → 양보 → 이동)
                5. 존 재타겟 (목표 칸을 남이 뺏었으면 다시 고름)
                6. 도착 판정 → 도킹 서비스 시작
                7. STALL 워치독
```

**서비스 시간** = `4 + 토트수 × 5` 틱, **픽업과 하역 각각**. 토트 6개를 실으면 한쪽에 34틱이다.
**회전** = 1틱 (Isaac 실측 90° 0.76초).
**틱 상한** = 200,000.

**STALL 판정**: 1500틱 동안 완료 태스크가 안 늘면 정체로 본다. 단 고정 시간 모드(지금 경로)에서는 **경고만 하고 계속 돈다** — 종료 코드는 0이고 `stall_warnings` 카운트만 오른다.

---

### 05. KPI 14키가 `sim.result` 로 돌아온다

**무엇이.** 시뮬이 stdout 에 `[SUMMARY] {…}` 한 줄을 뱉으면, 워커가 그 줄만 찾아 파싱하고 14키로 변환해 발행한다.

```json
{
  "run_id": "est_77_n3",
  "status": "done",
  "error": null,
  "kpi": {
    "orders_total": 116,            "orders_completed": 104,
    "inbound_total": 33,            "inbound_completed": 30,
    "makespan_s": 3600,
    "lead_time_mean_s": 190.0,      "lead_time_p90_s": 290.0,
    "queue_time_mean_s": 9.1,
    "throughput_orders_per_h": 104.0,
    "throughput_tasks_per_h": 202.0,
    "amr_utilization": 0.63,
    "conflict_wait_ratio": 0.1,
    "distance_total_m": 41200,
    "charge_downtime_s": 0
  }
}
```

**각 키의 뜻**:

| 키 | 뜻 |
|---|---|
| `orders_total` / `orders_completed` | 생성된 주문 수 / 지평선 안에 끝난 주문 수 |
| `inbound_total` / `inbound_completed` | 입고 건수 / 완료 |
| `makespan_s` | 지평선 모드에선 = horizon |
| `lead_time_mean_s` / `_p90_s` | 주문이 들어와서 끝나기까지 평균 / 상위 10% |
| `queue_time_mean_s` | 태스크가 생성돼서 배차되기까지 평균 |
| `throughput_orders_per_h` | 시간당 주문 처리량 |
| `throughput_tasks_per_h` | 시간당 태스크(=토트 왕복) 처리량 |
| `amr_utilization` | 로봇 가동률 0~1 |
| `conflict_wait_ratio` | 충돌 회피로 멈춰 있던 비율 |
| `distance_total_m` | 전 로봇 총 주행 거리 |
| `charge_downtime_s` | 충전으로 쉰 시간 — **배터리 OFF 라 항상 0** |

**수식**:
```
sim_hours              = T_END × DT / 3600
throughput_tasks_per_h = 완료태스크수 / sim_hours
amr_utilization        = (주행 + 대기 + 서비스 + 회전) / (대수 × T_END)
conflict_wait_ratio    = 대기 / max(주행 + 2×대기, 1)
distance_total_m       = 주행틱 × 1.0 m
```

> **비용은 FMS 가 계산하지 않는다.** KPI 14키에 비용 항목이 없다. 원가·예산 판정은 전부 백엔드 `CostCalculator` 몫이다.

**실패 처리**:
```json
{"run_id": "est_77_n3", "status": "failed",
 "error": "amr.count 는 1~28 정수: 29", "kpi": null}
```

> ⚠️ **DLQ 도 nack 도 없다.** 검증 실패·시뮬 실패·타임아웃은 전부 `status:"failed"` 메시지로 **정상 발행하고 ack** 한다. 다만 `OSError` 같은 예상 밖 예외는 핸들러 밖으로 새어 프로세스가 죽고, 컨테이너 재기동 + 브로커 재전달로 처리된다.

---

### 06. 결과가 화면으로 돌아온다 (03 RUNS → 05 QUOTE)

**무엇이.** 프론트는 버튼을 누른 직후 `EventSource('/api/estimates/{id}')` 를 연다. **같은 URL 이 `Accept` 헤더 협상으로 JSON 과 SSE 로 갈린다** — 별도 경로가 아니다. `EventSource` 는 자동으로 `Accept: text/event-stream` 을 붙이므로 SSE 핸들러로 간다.

**백엔드 체인**:
```
SimResultListener (@RabbitListener "sim.result")
  └ EstimateService.applyResult()                    [@Transactional]
      ├ findByRunId → markDone / markFailed          ← run_id UNIQUE 가 중복 방어 1차선
      ├ publishEvent(RunUpdated)
      └ finalizeIfComplete()                         ← 미완료 run 있으면 여기서 return
           └ EstimateEvaluator.evaluate()            ← 판정
                └ publishEvent(EstimateFinished)
                         ↓ @TransactionalEventListener(AFTER_COMMIT)
                  EstimateEventStream → SSE 송신
```

`AFTER_COMMIT` 을 쓰는 이유: 커밋 전에 SSE 를 쏘면 브라우저가 다시 조회했을 때 아직 DB 에 없을 수 있다.

**SSE 이벤트 3종**:

| 이벤트 | 언제 | 페이로드 |
|---|---|---|
| `snapshot` | 구독 즉시 1회 | 견적 전체 + `runs[]` 배열 |
| `run` | run 하나 끝날 때마다 (**13회**) | `{run_id, amr_count, status, throughput, utilization, outbound, total_cost, error}` |
| `estimate.done` | 13개 전부 끝난 뒤 1회 | `{estimate_id, status, recommended_count, knee_count, reason_code}` |

하트비트는 15초마다 주석(`: ping`)을 보낸다. 에미터 타임아웃 35분. 프론트는 연결이 끊기면 3초 뒤 **딱 한 번만** 재시도한다.

**권장 대수 판정** (`EstimateEvaluator`):

| 순서 | 조건 | 결과 |
|---|---|---|
| 1 | run 이 하나도 없음 | `ALL_FAILED` |
| 2 | run 2개 이상이고 **전부 처리율 ≥ 100%** | `LOAD_TOO_LOW` — 변별력 없음(창고가 너무 한가함) |
| 3 | `처리율 ≥ 목표` **and** `가동률 ≥ 목표` **and** `비용 ≤ 예산` | 만족하는 **최소 대수** = 권장 |
| 4 | 없으면 | `BUDGET_EXCEEDED` / `UTILIZATION_SHORT` / `THROUGHPUT_SHORT` |

**무릎(knee)** 은 "더 넣어도 별로 안 늘어나기 시작하는 지점"이다. 첫 구간의 처리량 증가 기울기를 기준으로, 그 **20% 밑**으로 떨어지는 첫 대수를 고른다.

> ⚠️ **배치 발행도 조기 종료도 없다.** 13건을 한 번에 다 던지고, 전부 끝난 뒤에만 판정한다. 목표를 이미 만족한 대수를 찾았어도 나머지를 계속 돌린다. `5_Docs/Web/견적_스윕_얼리스탑_설계.md` 가 "(설계 검토, 미구현)" 으로 남아 있다.

**여기까지가 1단계다.** 사용자는 05 QUOTE 에서 곡선과 권장 대수를 본다.

---

### 07. `물리 검증 시작` 을 누른다

**무엇이.** 권장 대수가 나와야 이 버튼이 화면에 나타난다(`recommendedCount == null` 이면 렌더 자체를 안 함). 바디 없는 `POST /api/estimates/{id}/verify` → 202.

**왜 2단계가 필요한가.** 1단계 DES 는 "1틱에 한 칸"이라는 추상 모델이다. 실제 로봇은 가속·감속·회전·센서 지연 때문에 한 칸에 **2.2~2.8초** 걸린다. 즉 DES 의 처리량은 **낙관적**이다. 그 낙관 정도를 실측하는 게 2단계다.

RabbitMQ `sim.verify.request`:
```json
{
  "run_id": "verify_77",
  "n_robots": 5,
  "window_s": 600,
  "source_run_id": "est_77_n5",
  "estimate_id": 77,
  "seed": 101
}
```

| 필드 | 의미 |
|---|---|
| `run_id` | `verify_{견적id}` — 견적당 **고정**. 이게 **멱등 키**다 |
| `n_robots` | 권장 대수 |
| `window_s` | 검증 창 길이 600초(10분) |
| `source_run_id` | 어느 견적 run 과 비교할지 — 추적용 꼬리표 |

`run_id` 가 고정인 이유: `host-orch` 가 최근 50건을 기억한다. 같은 `run_id` 가 재전달되면 **GPU 런을 다시 돌리지 않고 결과만 재발행**한다. 10분짜리 GPU 작업을 중복 실행하면 낭비이기 때문이다.

---

### 08. host-orch 가 GPU 를 깨우고 두 노드를 맞춘다

**무엇이.** `host-orch` 는 RabbitMQ 와 EMQX 양쪽에 붙은 **유일한 프로세스**이자 Sparkplug **Primary Host** 다. 잡은 한 번에 하나만 받는다(`prefetch 1`, 러너 1대 전제).

**왜 "둘"을 맞춰야 하나.** 검증 런에는 두 주체가 필요하다:
- **`sim-runner-gpu01`** (GPU 박스) — Isaac 물리 시뮬과 로봇 에이전트를 띄운다
- **`fms-core`** (EC2) — DES 커널로 "어느 로봇이 어디로 갈지"를 결정해 명령을 내린다

둘 다 준비돼야 의미 있는 측정이 시작된다. 하나만 뜬 상태의 시간은 측정에 넣으면 안 된다.

**상태 기계**:

```
STARTING
  ├ 15초마다 NCMD/sim-runner-gpu01 로 { Run Control/Start = <JSON> } 재전송
  │    └ GPU 박스에서:
  │         Isaac 기동 → 로봇 에이전트 ×N 기동
  │         전원 connection=ONLINE 이고 첫 위치(agvPosition)를 보내면
  │         → DBIRTH ×N 발행 → Runner/State = RUNNING
  │
  ├ 러너가 RUNNING 인 걸 확인한 뒤 → NCMD/fms-core 로 같은 Start JSON
  │    └ EC2 에서:
  │         전원 ONLINE + 포즈 수신 → DES 플래너 생성
  │         → 각 로봇에 정렬용 order 2건 (updateId 0, 1)
  │         → 전원이 n1 에 도착 → Fms/State = RUNNING
  │
  └ ★ 둘 다 RUNNING 으로 관측된 tick = started_at ← 검증 창 시작점

RUNNING
  └ window_s(600초) 경과 → 양쪽에 Stop

STOPPING
  └ 러너 IDLE + fms IDLE 확인 → 결과 발행 → ack
```

**타임아웃**:

| 이름 | 값 | 무엇을 기다리나 |
|---|---|---|
| start | 180 s | 러너가 우리 `run_id` 를 잡을 때까지 |
| ready (real) | 420 s | Isaac 뜨고 전 로봇 포즈 올 때까지 |
| stop | 60 s | Stop 후 정리될 때까지 (초과 시 1회 재전송, 또 초과면 실패) |
| heartbeat | 5 s × 3 | 15초 무소식이면 연결 끊김 판정 |

**GPU 박스에서 실제로 뜨는 명령**:
```bash
# Isaac
bash -c "cd ~/isaacsim && ./python.sh 2_Simulation/apps/warehouse_sim.py"

# 로봇 에이전트 × N
bash -c "source /opt/ros/humble/setup.bash && source .../t4_agent/install/setup.bash && \
  exec ros2 launch amr_agent agent.launch.py \
    serial:=amr01 manufacturer:=idealworks model:=iw_hub \
    mqtt_host:=<EMQX> mqtt_username:=amr01 mqtt_password:=<비밀> \
    pose_source:=odom use_sim_time:=true"
```

넘기는 환경변수: `ROS_DOMAIN_ID` · `WSIM_N`(로봇 수) · `SIM_RUN_ID` · `SIM_SCENARIO_PATH` · `SIM_SCENARIO_SHA256` · `WSIM_SPAWN`(스폰 좌표).

Isaac 은 `WSIM_N ≥ 2` 면 자동으로 **네임스페이스 모드**로 들어가 `/amr01/cmd_vel`, `/amr01/odom`, `/amr01/scan` 을 쓴다. `/clock`(시뮬 시간)은 첫 로봇만 발행한다.

**Sparkplug 메트릭** — `host-orch` 가 관측하는 것:

| 메트릭 | 값 | 쓰임 |
|---|---|---|
| `Runner/State` | IDLE·STARTING·RUNNING·STOPPING·ERROR | 상태 판정 |
| `Runner/RunId` | 현재 런 이름 | 우리 런인지 확인 |
| `Runner/RTF` | Real-Time Factor | 실시간 대비 배속. 1.0 이면 실시간 |
| `Runner/RamMB` · `VramMB` | 메모리 최대치 | 자원 리포트 |
| `Runner/IsaacInstances` | 인스턴스 수 | |
| `Fms/State` · `Fms/RunId` · `Fms/Tick` | fms-core 쪽 동일 | |

**연결이 끊기면**: MQTT 의 **LWT(Last Will and Testament)** 로 브로커가 대신 `NDEATH` 를 뿌려준다. `host-orch` 는 `bdSeq` 번호가 마지막 `NBIRTH` 와 일치할 때만 진짜 죽음으로 인정한다(늦게 온 옛 유언 무시).

**Stop 시 종료 사다리** — 순서가 중요하다:
```
1. DDEATH ×N 즉시 발행          (로봇이 죽었다고 먼저 알림)
2. 에이전트에 SIGINT
3. 5초 후 SIGTERM (프로세스 그룹)
4. 3초 후 SIGKILL
5. 에이전트가 전부 죽은 걸 확인한 뒤에야
6. Isaac SIGINT → 20초 → SIGTERM → 3초 → SIGKILL
```
에이전트를 먼저 죽이는 이유: Isaac 이 먼저 죽으면 에이전트가 `/odom` 을 못 받아 에러를 뿜으며 남는다.

---

### 09. 틱 배리어로 로봇을 끌고 간다

**무엇이.** `fms-core` 는 견적과 **완전히 같은 DES 커널**을 교체형 Planner 뒤에 두고, 실 로봇을 그 상태에 맞춘다. **커널이 진실이고 셸이 따라가는** 구조다.

**왜 이렇게 하나.** 견적 KPI 와 검증 KPI 를 비교하려면 두 쪽이 같은 결정을 내려야 한다. 커널을 하나로 두면 "결정은 같고 물리만 다르다"가 보장돼, 차이가 전부 물리 때문임을 알 수 있다.

**한 틱**:
```python
if (서비스 처리됨) and (fleet.pending() 비었음) and (now - tick_started >= tick_s):
    cmds = planner.step(t)                    # DES 커널 한 스텝
    for cmd in cmds:
        link.next_order(x, y, yaw, reverse)   # uagv/v2/idealworks/amrNN/order 발행
    t += 1
    drain_events() → biz/v1/event             # 태스크 배정/완료 이벤트
```

#### 틱 배리어란

**"전 로봇이 이번 칸에 도착할 때까지 다음 틱을 시작하지 않는다"** 는 규칙이다. 구현은 단순하다 — `fleet.pending()` 이 빌 때까지 `_advance()` 를 호출하지 않는 것이 전부다.

| 상황 | 동작 |
|---|---|
| order 발행 | `pending = k+1` 로 표시하고 `sent_at` 기록 |
| 로봇 state 의 `lastNodeId == "n{pending}"` | `settle()` → pending 해제 |
| state 2건이 지나도 미반영 | **같은 orderUpdateId 로 재발행** (유실 대비) |
| state 10건이 지나도 미반영 | 런 실패 — 로봇이 명령을 무시하고 있다 |
| `ack_timeout_s` 30초 초과 | 런 실패 — 재발행해도 이 타이머는 리셋되지 않는다 |
| WAIT(제자리) 로봇 | Command 자체가 생략 → pending 이 안 생겨 기다리지 않음 |

`--tick-s 1.0` 은 **틱의 최소 벽시계 길이**다(상한이 아니다). 로봇 도착이 늦으면 틱이 그만큼 늘어난다. 그래서 **벽시계 600초 창이 DES 로는 118틱 = 118초어치**밖에 안 된다.

#### VDA 5050 order 가 `/cmd_vel` 이 되기까지

```
EMQX ──▶ bridge_node ──▶ executor_node ──▶ primitive_node ──▶ /cmd_vel ──▶ Isaac
   (MQTT)      (ROS)     (order 분해)      (액션 서버)      (Twist)
   ◀──────────────────── state.lastNodeId ◀────────── 도착 보고
```

| 노드 | 역할 |
|---|---|
| `bridge_node` | MQTT ↔ ROS 번역. 접속 시 `connection=ONLINE`(retained) + `factsheet`(로봇 사양) 발행 |
| `executor_node` | order 를 **프리미티브(회전/주행)** 로 분해하고, 도착하면 `state` 발행 |
| `primitive_node` | 액션 서버. 20 Hz 로 `/cmd_vel` 을 뿜어 목표에 맞춘다 |

**order 분해 규칙**:
```
if 목표까지 거리 > 0.05 m:                     # 이동이 있는 노드
    path_dir = 목표 방향
    reverse  = edge 의 orientation 이 π 인가
    face     = reverse ? path_dir + π : path_dir
    if |face - 현재 헤딩| > 0.12 rad:  TURN(face) 추가
    DRIVE(목표, reverse) 추가
if node.theta 가 있고 주행 후 헤딩과 다르면:  TURN(node.theta) 추가
```

| FMS 액션 | VDA 5050 표현 | 프리미티브 | `/cmd_vel` |
|---|---|---|---|
| 전진 1칸 | 1 m 앞 노드, edge 에 `orientation` 없음 | (헤딩 어긋나면 TURN) → DRIVE | `linear.x > 0` + 크로스트랙 보정 |
| **후진 1칸** | edge `orientation = π`, `TANGENTIAL` | DRIVE(reverse=true), 보통 TURN 없음 | `linear.x < 0`, 헤딩 유지 |
| 90° 회전 | 같은 좌표 + 새 `theta`, 길이 0 | TURN 1건 | `linear.x=0`, `angular.z` 램프 |
| 대기 | 새 order 없음 | 없음 | 정지 유지 |
| 틱 ack | — | `node_reached()` | `state.lastNodeId = n_k` 즉시 발행 |

> **후진이 왜 필요한가**: iw.hub 는 구동축이 앞에 있고 차체가 뒤로 1.03 m 매달려 있다. 좁은 통로에서 돌아 나오는 것보다 후진이 빠르고 안전하다. DES 커널도 이 모델(`REAR_CELLS = 1`)로 계획한다.

파라미터: `v_max 0.8 m/s`, `w_max 1.2 rad/s`, 정지 허용치 `tol_xy 0.05 m` / `tol_theta 0.03 rad`.

**중복 order 방어**: 같은 `orderId` + 같은 `orderUpdateId` 가 오면 오류 없이 무시하고 `state` 만 다시 낸다(FMS 재발행 대응). 낮은 `orderUpdateId` 가 오면 `orderUpdateError`.

---

### 10. 창이 닫히고 결과가 웹으로 돌아온다

**무엇이.** 600초가 지나면 `host-orch` 가 양쪽에 Stop 을 보낸다.

`fms-core` 종료 절차:
1. 모든 로봇에 `instantActions cancelOrder` 발행 (진행 중 명령 취소)
2. `planner.kpi(t)` 로 KPI 14키 계산
3. `biz/v1/event` (QoS 1) 로 `kpi_summary` 1건 발행
4. `Fms/State = IDLE` 또는 `ERROR`

`host-orch` 가 그 이벤트를 받아 결과 큐로 내보낸다:

```json
{
  "run_id": "verify_77",
  "status": "done",
  "error": null,
  "verify": {
    "runner": "sim-runner-gpu01",
    "n_robots": 5, "robots_online": 5,
    "window_s": 600, "elapsed_s": 600.4,
    "started_at": "2026-09-11T01:20:11.482Z",
    "ended_at":   "2026-09-11T01:30:11.902Z",
    "rtf_mean": 1.02,
    "ram_mb_max": 5120, "vram_mb_max": 9800,
    "isaac_instances": 1,
    "scenario_sha256": null, "scenario_ack": false,
    "reason": "window elapsed",
    "fms": {
      "node": "fms-core",
      "ticks": 118,
      "status": "done", "error": null,
      "planner": "kernel",
      "counters": {"open_tasks":3,"done_tasks":11,"orders_done":9,"orders_total":12},
      "warnings": null
    }
  },
  "kpi": { … 14키 … },
  "artifacts": null,
  "source_run_id": "est_77_n5",
  "estimate_id": 77
}
```

**가장 중요한 숫자는 `verify.fms.ticks` 다.** 그 창에서 실제로 돈 커널 틱 수다. 위 예시에서 **벽시계 600초 = DES 118초**다. 즉 실 로봇은 DES 보다 약 **5배 느리다**(이 런 기준). 견적 처리량을 그만큼 할인해서 봐야 한다는 뜻이다.

**`warnings` 가 null 이 아니면** 그 런은 견적과 다른 조건에서 돌았다는 신호다(예: θ 파일이 없어 중립 θ 로 돌았다). 그 경우 **KPI 를 견적 KPI 와 비교하면 안 된다.**

**`verify.reason` 값** (왜 창이 닫혔나):

| reason | 최종 | 의미 |
|---|---|---|
| `window elapsed` | done | 정상 — 600초 다 채움 |
| `start timeout` | failed | 180초 안에 러너가 안 뜸 |
| `runner NDEATH` | failed | GPU 쪽 프로세스가 죽음 |
| `heartbeat lost` | failed | 15초 무소식 |
| `fms-core rejected Start` | failed | 시나리오가 잘못됨 |
| `stop timeout` | failed | 정리가 60초 안에 안 끝남 |

**백엔드 수신**: `VerifyResultListener` → `applyVerifyResult` → `artifacts.video` 가 있으면 `estimates.verify_video_url` 에 저장 → SSE `verify.done` 발송 → 프론트 `04 VERIFY` 섹션이 영상과 함께 나타난다.

프론트는 **영상 URL 이 있을 때만** 04 섹션을 렌더한다(`verify.status !== 'done' || !verify.videoUrl` 이면 `null` 반환).

---

## 5. 지금 실제로 끊겨 있는 6곳

**01~06(견적)은 끝까지 돈다.** **07~10(물리 검증)은 완주하지 못한다.** 이유는 아래와 같다.

### 🔴 1. EMQX 가 모든 publish 를 거부하고 있다

ACL 행렬 테스트 50건 중 **publish 28건이 allow·deny 가리지 않고 전부 실패**하고, subscribe 는 전부 통과한다. 브로커는 살아 있고(ping 4.2 ms) 인증도 통과한다.

패턴이 진단적이다:

| 케이스 | 판정 기준 | 관측 |
|---|---|---|
| allow-pub | 관측자가 메시지를 받아야 함 | 거부돼 아무것도 안 옴 → 실패 |
| deny-pub | 브로커가 연결을 끊어야 함 | 안 끊음 → 실패 |
| subscribe | SUBACK + 연결 유지 | 정상 → 통과 |

**가설**: 배포된 EMQX 의 ACL 이 레포의 `acl.conf` 와 어긋났고, `deny_action` 도 `disconnect` 가 아니다. 레포 쪽 설정은 정상이므로 **배포 과정의 문제**로 보인다.

**영향**: 사실이면 08~10 구간 자체가 돌지 않는다. **최우선 확인 대상.**

확인 방법: EC2 에 접속해 컨테이너의 실제 `acl.conf`·`emqx.conf` 를 레포본과 diff.

### 🔴 2. 검증 영상을 만드는 쪽이 없다

`sim.verify.result` 의 `artifacts` 필드가 **`null` 로 하드코딩**돼 있다. 녹화 코드도, MinIO 업로드 코드도 GPU 박스·host-orch 어디에도 없다.

| 있는 것 | 없는 것 |
|---|---|
| MinIO 컨테이너 (포트 9002) | 버킷 생성·정책 |
| 백엔드 `artifacts.video` 수신부 | 업로드하는 쪽 |
| `estimates.verify_video_url` 컬럼 | 녹화 자체 |
| 프론트 `04 VERIFY` 렌더링 | |

**즉 받는 쪽은 완성돼 있고 만드는 쪽만 없다.** 그래서 04 섹션은 영원히 안 보인다.

참고: MJPEG 라이브 스트림(`sil_isaac/viewer/http_stream.py`)은 있지만 파일로 저장하지 않고, T3 레거시의 ffmpeg 녹화 도구는 이 파이프라인과 연결돼 있지 않다.

### 🔴 3. 백엔드가 `scenario` 를 보내지 않는다

검증 요청이 **6키만** 보낸다. 계약이 정의한 `scenario` 객체(주문 목록, 적재 조건, 창 오프셋)가 통째로 빠진다.

결과: `fms-core` 가 `orders=[]` 를 **"명시 주문 목록이 비어 있음"** 으로 해석해 **주문 0건짜리 검증 런**이 된다. 로봇은 뜨지만 시킬 일이 없어 제자리에 서 있는다.

### 🟡 4. `seed 101` 이 전달되지 않는다

최상위 `seed` 는 파싱만 되고 Start 명령 JSON 에 실리지 않는다. `fms-core` 는 `scenario.seed` 기본값 **42** 로 돈다. 견적과 다른 난수열이라 주문 순서가 달라진다.

### 🟡 5. 요청 파싱 실패 결과를 백엔드가 버린다

잘못된 요청에 대한 결과 JSON 에는 `estimate_id` 키 **자체가 없다**. 백엔드는 `estimate_id == null` 로 보고 조용히 폐기한다. 사용자는 실패했다는 사실조차 알 수 없다.

### 🟡 6. `window_start_s` 가 놀고 있다

주문 밀도가 앞쪽에 몰려 있다 — 실측으로 **첫 10분에 2건, 10~20분에 124건**. 그래서 창을 앞에서 자르면 로봇이 놀기만 해 검증이 무의미하다. `window_start_s` 는 주문 시각을 앞당겨 안정 구간을 잘라오는 장치인데, 3번 때문에 실사용이 0이다.

### 정리

**2번과 3번이 짝이다.** 영상 생산자와 시나리오 전달, 이 둘만 채우면 물리 검증이 화면까지 이어진다. 단 1번(EMQX)이 먼저 해결돼야 한다.

---

## 6. 부록

### 6.1 자주 찾게 되는 상수

**DES 커널**

| 항목 | 값 | 근거 |
|---|---|---|
| DT (1틱) | 1.0 s | 칸 1 m ÷ 속도 1 m/s |
| 90° 회전 | 1 틱 | Isaac 실측 0.76 s (2026-09-03 확정) |
| 차체 뒤 여유 | 1 칸 (1.03 m) | Isaac 실측 |
| 서비스 시간 | 4 + 토트수 × 5 틱 | 픽업·하역 각각 |
| 토트 상한 / 왕복 | 6 | 선반 3단 × 2 |
| 토트당 상품 | 8 | 신발 상자 기준 |
| STALL 기준 | 1500 틱 | |
| 틱 상한 | 200,000 | |
| 맵 크기 | 107 × 116 칸 | free 8,640칸 |
| 최대 로봇 | 28 | 충전 존 용량 |

**파이프라인**

| 항목 | 값 |
|---|---|
| horizon | 6 h = 21,600 s |
| 견적 seed | 101 (고정) |
| 워커 타임아웃 | 600 s |
| run 타임아웃 | 30 min |
| SSE 하트비트 | 15 s |
| 검증 창 | 10 min = 600 s |
| start / stop 타임아웃 | 180 s / 60 s |
| ready 타임아웃 (real) | 420 s |
| AMR 단가 | 15,000,000 원 |
| 설치비 / 대 | 500,000 원 |
| 충전기 | AMR 4대당 1, 최대 6 |
| 워커 replicas | 2 |

### 6.2 파일 색인 — "이건 어디 있나"

| 찾는 것 | 위치 (팀 레포 `S15P21A106` 기준) |
|---|---|
| 프론트 화면·API 호출 | `4_Web/SantaFactoryFE/src/{App.jsx, api/estimates.js, components/}` |
| 백엔드 엔드포인트 | `4_Web/SantaFactoryBE/.../controller/EstimateController.java` |
| 견적 로직·판정 | `4_Web/SantaFactoryBE/.../service/estimate/` |
| 큐 발행 | `4_Web/SantaFactoryBE/.../client/SimulationClient.java` |
| DB 스키마 | `4_Web/SantaFactoryBE/src/main/resources/db/migration/` |
| 견적 워커 | `3_FMS/infra/worker/estimate_worker.py` |
| DES 진입점 | `3_FMS/sim_engine/sim_v2_tasks.py` |
| DES 커널 | `3_FMS/sim_engine/kernel/core.py` |
| KPI 계산 | `3_FMS/sim_engine/cli/report.py` |
| θ 적용 | `3_FMS/sim_engine/from_theta.py` |
| 맵 로딩 | `3_FMS/sim_engine/map_loader.py` |
| 주문 스트림 | `3_FMS/sim_engine/order_stream.py` |
| 오케스트레이터 | `3_FMS/host_orch/orchestrator.py` |
| VDA 마스터 | `3_FMS/fms_core/conductor.py`, `vda.py` |
| EMQX 설정·ACL | `3_FMS/infra/emqx/` |
| GPU 데몬 | `2_Simulation/sim_runner/` |
| ROS 에이전트 | `2_Simulation/t4_agent/src/amr_agent/` |
| Isaac 진입점 | `2_Simulation/apps/warehouse_sim.py` |
| 계약 문서 | `3_FMS/contracts/` |

### 6.3 계약 문서

| 문서 | 버전 | 정의하는 것 |
|---|---|---|
| `SIM_RUN_CONTRACT.md` | v3.1 | `sim_v2_tasks.py` CLI 인자·종료코드 |
| `SIM_VERIFY_CONTRACT.md` | v0.3-draft | `sim.verify.request/result` 스키마 |
| `MQTT_CONTRACT.md` | v1.2 (헤더는 v1.0 표기 — stale) | EMQX 토픽·ACL·Sparkplug 메트릭 |
| `guidance_types.py` | θ 스키마 v2 | θ 11필드와 범위 |

### 6.4 직접 확인하는 법

```bash
# 견적 워커를 브로커 없이 1회 실행
cd 3_FMS && python3 infra/worker/estimate_worker.py --once 요청.json

# DES 직접 실행
cd 3_FMS/sim_engine && python3 sim_v2_tasks.py test 5 0 42 1 --horizon=3600

# 전체 테스트 (GPU 불필요)
cd 3_FMS && python3 -m pytest fms_core/tests host_orch/tests sim_engine/tests -q
cd 2_Simulation && python3 -m pytest -q

# EMQX ACL 검증 (살아있는 브로커에 실제로 발행한다)
cd 3_FMS && python3 -m pytest contracts/tests/test_acl_matrix.py -q
```

---

**웹 버전**: https://claude.ai/code/artifact/1c916679-c6fe-4901-808e-208f1e53f9c3
**관련 문서**: `AMR_기획_기준선_v3.md` (기획·제품 정의) · `AMR_경로계획_기반이론_정리.md` (PIBT 이론) · `AMR_구현_결정_인벤토리.md` (결정 이력)
