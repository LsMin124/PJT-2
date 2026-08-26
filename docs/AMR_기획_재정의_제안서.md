# AMR 견적 시뮬레이션 — 기획 재정의 제안서

> 작성 2026-08-20 · 상태: **팀 결정 대기** · 근거 링크는 본문 각 절
> 결론 한 줄: **하나의 사이징 엔진으로 두 개의 산출물을 판다 — 판매자에겐 제안서 생성기(주), 구매자에겐 품의·보조금용 비교 검증 리포트(부).**

---

## 1. 왜 재정의가 필요한가 — 기존 기획의 구조 결함 3가지

기존 기획(구매자 대상 중립 견적 SaaS)은 검증 과정에서 아래 세 결함이 확인됐다. 셋 다 기획 완성도의 문제가 아니라 **서 있는 자리의 문제**다.

| # | 결함 | 내용 |
|---|---|---|
| 1 | **지불 유인 없음** | v1 타겟(3~15대 점진 도입 화주)에게는 단일 벤더 턴키가 합리적이고, 벤더가 견적을 무료로 제공한다. 제3자 견적에 돈 낼 이유가 없다 |
| 2 | **데이터 접근 불가** | 벤더의 주행·배차 알고리즘과 실측 데이터는 비공개. 중립 위치에서는 시뮬레이션을 보정할 정당한 데이터 경로가 없다 |
| 3 | **검증 불가한 정확도 주장** | 우리 시뮬 결과를 "고객이 실제로 사게 될 시스템"과 대조할 수단이 없다 — 견적의 신뢰 주장이 구조적으로 약하다 |

---

## 2. 검토했으나 기각한 대안들

| 대안 | 기각 사유 |
|---|---|
| 중립 견적 SaaS 유지 | 결함 1~3 해소 불가. 노벨티 방어는 가능하나 지불자가 없음 |
| FMS 제품화 피벗 | 단일 벤더 도입은 사실상 100% 벤더 번들 FMS 사용. 서드파티 FMS는 이기종 혼합 플릿(대형 제조, 자동차 중심)의 좁은 니치 — v1 타겟과 불일치 |
| "참조 FMS" 명명 | 논리는 정합하나(벤더 재현 주장 회피) 구매자에게 "가상 운영체계 기준 견적"의 실용 가치가 약함 — 궁여지책 |
| 시장 서사 포기(기술 쇼케이스) | 프로젝트는 성립하나 "시장 논리에 맞는 기획" 요구의 포기 — 최후 폴백으로만 보류 |

이기종 FMS 니치가 실재한다는 근거 자체는 있다(참고: [OTTO의 VDA 5050 혼합 플릿 인증](https://ottomotors.com/company/newsroom/press-releases/otto-adds-vda-5050-certifications-to-support-mixed-fleet-deployments/), [MiR의 서드파티 FMS 어댑터](https://www.sdcexec.com/warehousing/robotics/news/22935658/mobile-industrial-robots-mir-amr-adapter-for-thirdparty-fleet-management-systems), [SYNAOS 벤더 중립 FMS](https://www.synaos.com/en/platform/mrfm)). 다만 니치의 위치가 우리 타겟과 다르므로 v2 백로그로 둔다.

---

## 3. 제안 — 하나의 엔진, 두 개의 산출물

### 산출물 A (주): 판매자용 제안서 생성기

**고객**: 자체 시뮬레이션 팀이 없는 AMR 벤더·총판·SI의 영업/설계 담당자.
(참고: 확정 명세서의 UseCase Actor에 "AMR 영업/설계 담당자"가 이미 포함되어 있다 — 페르소나 신설이 아니라 **가장 강한 하나로 좁히는 것**)

| 논리 | 내용 |
|---|---|
| 지불 유인 | 사이징·제안서가 곧 수주. 영업 도구는 파는 사람이 산다 |
| 데이터 경로 | 고객(벤더)이 자기 로봇의 실측으로 모델을 보정 — 결함 2·3이 **정당하게** 소멸 |
| 역량 공백 | 시뮬레이션 팀은 Geek+·KUKA급만 보유([시장 1~3위는 전부 GTP 턴키 대기업](https://www.geekplus.com/resources/news/global-warehouse-automation-surges-as-geekplus-extends-no.1-amr-leadership-for-seven-consecutive-years)). 국내 중소 벤더·SI 롱테일은 감으로 견적 — *가설, §6 인터뷰로 검증* |

### 산출물 B (부): 구매자용 비교 검증 리포트

**정체**: 견적 생성기가 아니라 **제안 정규화 리포트** — 받아 온 벤더 제안서들을 동일 가정 위에 재계산해 비교 가능하게 만들고, 낙관적 가정을 플래그한다. 감정평가서처럼 **거래 순간에 건당 과금**.

**지불 순간이 특정된다는 것이 핵심이다:**

| 순간 | 근거 |
|---|---|
| 품의·투자심의 | budgetary quote는 비구속 개산이며 내부 승인·예산 배정용이라는 것이 조달 관행으로 정착 ([LightSource 용어집](https://lightsource.ai/glossary/budgetary-quote), [Naboo 가이드](https://www.naboo.app/en-us/blog/budgetary-quote-guide)). 담당자에게 필요한 건 비교하고 싶은 마음이 아니라 **비교했다는 문서** |
| 복수 견적 요구 | 공공 수의계약은 2인 이상 견적서 제출이 법령상 원칙 ([생활법령정보](https://easylaw.go.kr/CSP/CnpClsMain.laf?popMenu=ov&csmSeq=519&ccfNo=3&cciNo=2&cnpClsNo=1)), 비교견적 관행은 만연 수준 ([관행 분석 기사](https://mix.campaigns.do/g/fdsc/news/638/1886)) |
| 보조금 신청 | 스마트공장 컨설팅은 정부 50%+자부담 50%의 **유상 제도** ([KCL 컨설팅사업](https://kcl.re.kr/site/homepage/menu/viewMenu.do?menuid=001014002004), [2025 통합공고](https://www.bizinfo.go.kr/web/lay1/bbs/S1T122C128/AS/74/view.do?pblancId=PBLN_000000000101903)), 사업계획서 작성 대행은 상품으로 거래 중 ([크몽 사례](https://kmong.com/gig/395672)) |

**명시적으로 제외하는 고객**: 예산 내 목표 달성이면 비교하지 않는 만족화 고객, 단일 벤더 확정 고객. 이들은 타겟이 아니다.

### 한 줄 서사

> **"견적은 공짜다. 그러나 비교 가능한 근거는 공짜가 아니다."**

### 세그먼트 제안 (2026-08-24 추가): 첫 고객은 3PL·풀필먼트

§1의 결함 3개를 모두 완화하는 유일한 세그먼트라 **첫 고객군으로 제안**한다. 재정의안 채택 시 함께 결정할 것(§7-4).

| 기존 결함 | 3PL에서의 완화 |
|---|---|
| 지불 유인 (일회성 구매자 → 구독 불성립) | 신규 화주 계약·신규 사이트마다 "이 물량이면 몇 대?"가 **반복** 발생 — 구독이 성립하는 유일한 구매자군 |
| 데이터 접근 | WMS로 주문 프로파일·물동량을 이미 표준 형태로 보유 — 인테이크 상위 티어를 낼 수 있는 고객 |
| 검증 불가 | 다수 사이트 운영 → 예측 vs 실적 비교 데이터가 축적될 수 있는 유일한 구조 |

**이중 역할이 핵심이다.** 3PL은 로봇에 대해선 구매자(산출물 B 소비)지만, 화주 RFP에 응찰할 땐 "이 물량을 이 인력·장비로 처리한다"는 제안서를 내는 판매자다(산출물 A의 구조 재사용). 한 세그먼트가 두 산출물을 동시에 소비한다. RaaS 정합도 있다 — 화주 계약이 1~3년이라 CAPEX 회수가 안 맞아 RaaS를 선호하고, 그만큼 사이징 질문이 가볍고 자주 발생한다(카탈로그의 Locus·트위니 나르고가 정확히 이 세그먼트용).

**협곡 주의(바벨 구조)**: 대형 3PL(CJ대한통운 TES·DHL)은 자체 엔지니어링 조직으로 내재화, 영세 업체는 로봇 검토 자체를 안 한다. 스윗스팟은 그 사이 — **성장 중인 중견 이커머스 풀필먼트**(연 단위 화주 계약 갱신 + RaaS 검토 단계). "중소 3PL의 일상 사이징 질문이 인력이 아니라 로봇 대수인가"는 *추정*이므로 §6 인터뷰 ③으로 검증한다.

---

## 4. 근거 지도 — 주장별 증거 등급

### ✅ 확인됨

| 주장 | 출처 |
|---|---|
| 러프 견적(budgetary quote)은 무료·비구속, 정밀 견적과 2단계 분리 | [LightSource](https://lightsource.ai/glossary/budgetary-quote) · [Naboo](https://www.naboo.app/en-us/blog/budgetary-quote-guide) |
| 벤더 제안서는 표준화 없이는 비교 불가(apples-to-apples 정규화 필요) | [Brown — RFP 가격 비교 템플릿](https://www.brownintegratedlogistics.com/resources/warehouse-rfp-template) · [제조 설비 RFP 가이드](https://ifactoryapp.com/greenfield-consulting/manufacturing-rfp-template-equipment-automation-vendor-selection) |
| 그 정규화·선정을 대행하는 유상 컨설팅이 실재 | [SCT Advisory — 선정 대행](https://sct-advisory.com/supply-chain-services/technology-consulting/system-selections/warehousing-selections/wms-program-planning-rfp-oversight/) |
| 구매자가 제3자 설계·자문에 지불하는 선례 | [MWPVL — 물류설비 설계 자문](https://www.mwpvl.com/html/material_handling.html) · [MWPVL — 시설 설계](https://www.mwpvl.com/html/facility_design.html) |
| 보조금 생태계의 유상 컨설팅·문서 시장 | [KCL](https://kcl.re.kr/site/homepage/menu/viewMenu.do?menuid=001014002004) · [bizinfo](https://www.bizinfo.go.kr/web/lay1/bbs/S1T122C128/AS/74/view.do?pblancId=PBLN_000000000101948) · [크몽](https://kmong.com/gig/395672) |
| 복수(비교) 견적 요구 — 공공은 법령, 민간은 관행 | [생활법령정보](https://easylaw.go.kr/CSP/CnpClsMain.laf?popMenu=ov&csmSeq=519&ccfNo=3&cciNo=2&cnpClsNo=1) · [관행 기사](https://mix.campaigns.do/g/fdsc/news/638/1886) |
| 3PL이 피킹 보조 AMR의 최대 반복 수요처 — 세계 최대 3PL DHL이 Locus 로봇을 500→2,000→5,000대로 RaaS 확장 계약, 35+ 사이트 누적 5억 픽 | [PR Newswire 공식 발표(2023)](https://www.prnewswire.com/news-releases/dhl-supply-chain-expands-global-partnership-with-locus-robotics-to-deploy-5-000-amrs-across-multiple-sites-301820650.html) · [DHL 공식](https://www.dhl.com/global-en/delivered/innovation/dhl-and-locusbots-hit-500-million-picks.html) |

### 🌐 해외 근거 — "국내 관행 특수론" 반박용

| 주장 | 출처 |
|---|---|
| **"영업용 시뮬레이션"은 해외에서 이미 상용 카테고리다** | Visual Components가 아예 [Sales Acceleration 솔루션](https://www.visualcomponents.com/solutions/sales-acceleration/)으로 판매 — RFQ/RFP 대응·제안서 콘텐츠 자동화. 스웨덴 자동화 업체 [Sejfo가 CAD·PPT를 시뮬레이션으로 교체해 수주율 개선](https://www.visualcomponents.com/case-studies/sejfo-transforms-automation-sales-with-visual-components-simulation/) 사례 공개 |
| 창고 설비 업계도 동일 — "판매를 위한 시뮬레이션" 제품 계보 존재 | Rockwell [Emulate3D/Demo3D](https://www.demo3d.com/) — "자동화 물류 시스템의 시연·이해·**판매**를 위한 소프트웨어", "주요 창고 자동화 공급사·통합업체 대부분이 사용", 2019년 Rockwell 인수 ([Material Handling 제품군](https://www.emulate3d.com/material-handling/)) |
| 구매자 측 무(無)벤더편향 유상 자문 시장은 해외가 더 크다 | [St. Onge Company](https://stonge.com/) — 1983년 설립, "no vendor-bias" 독립 물류 엔지니어링(설계·사양·구현 관리, Fortune 500 다수) · [enVista MH 컨설팅](https://envistacorp.com/automation/material-handling-consulting/) · MWPVL(§4 기존) |
| 보조금-컨설팅 결합 제도는 한국 특수가 아니라 국제 공통 패턴 | 독일 [Digital Jetzt](https://digital-skills-jobs.europa.eu/en/opportunities/funding/digital-jetzt-programme) — 중소기업 디지털화 보조, [컨설팅 비용 50~80% 보조](https://blog.provenexpert.com/en/digital-funding-opportunities-in-germany) · 미국 [NIST MEP](https://www.nist.gov/mep) — 연방·주 매칭 기금으로 중소 제조사 컨설팅 지원 |

**함의 3가지:**
1. **카테고리 창조가 아니라 검증된 카테고리의 재편이다** — "시뮬레이션으로 판다"는 명제는 해외에서 이미 상업적으로 증명됨. 기획 리스크가 낮아진다.
2. **경쟁 구도는 3층이다 — Demo3D·Visual Components는 경쟁자가 아니라 카테고리 검증자다.** 둘 다 시뮬레이션 전문 인력 + 고가 데스크톱 라이선스가 전제라, 우리 타겟(시뮬 팀 없는 벤더·SI)은 애초에 운용할 수 없다. 층을 나누면: ① **현재 경쟁자 = 엑셀·경험칙·PPT(비소비)** — 타겟이 지금 견적을 만드는 방식 ② **상위 시장 강자 = Demo3D·VC** — 비경쟁, 수요의 증명 ③ **잠재 위협 = 그들의 SaaS 하향 진입** — 방어선은 AMR 수직 특화(MAPF·플릿 통계·SIL)와 선점 속도. 벤더 고객의 경제적 대안은 "시뮬 엔지니어 채용 + 라이선스" 또는 "건당 컨설팅 외주"이며, 이것이 가격 설계의 기준선. 자기 위치 한 줄: **상위 도구가 증명한 수요를, 그 도구를 못 쓰는 롱테일에 SaaS로 전달한다(low-end disruption)**.
3. **시장이 국내 한정이 아니다** — 산출물 A·B 모두 해외 선례 위에 있으므로, TAM 서사를 국내 벤더 수에 가둘 필요가 없다(단 §6의 보수 원칙 유지).

### ⚠️ 간접 근거 (인터뷰로 확정할 것)

- "벤더는 이길 만한 딜에만 정밀 제안 엔지니어링을 투자한다" — 2단계 견적 구조와 공수 구조상 자명하나 직접 출처 없음
- "국내 중소 벤더·SI는 시뮬레이션 역량이 없다" — 조직 규모상 개연적이나 직접 확인 필요

### ❌ 근거 없음 (기획에 사용 금지)

- ~~"대형 프로젝트는 유상 개념설계(FEED)로 전환되기도 한다"~~ — 출처 확인 실패, **철회**
- 국내 AMR 연간 도입 건수(TAM 분모) — 공표 통계 없음, 보수 가정으로만 표기

---

## 5. 피벗 비용 — 기존 자산이 전부 재배치된다

| 기존 자산 | 새 역할 |
|---|---|
| 시뮬 엔진·2계층·스택(RabbitMQ·Kafka·MQTT·k3s) | 그대로 |
| AMR_SPEC 사용자 등록(명세서 ERD) | 벤더가 자기 기종을 등록하는 그 기능 |
| 카테고리 게이트(배제 사유) | 벤더의 **리드 스크리닝** ("이 고객은 우리 기종에 안 맞음") |
| T1 러프 견적 퍼널 | 벤더 웹사이트 임베드용 **리드 수집 위젯** |
| Isaac SIL + 오차 공개 | 벤더가 고객 설득에 쓰는 **증거 생성기** |
| 투명성 장치(가정 명세·95% CI·민감도) | 제안서 신뢰도 재료 + 산출물 B의 정규화 기준 |
| AI 4역할(역방향 최적화·액티브 러닝·SHAP·배차 옵션 축) | 그대로 — ④는 "우리 SW 옵션이 로봇 N대 값을 아낀다"는 영업 논리로 승격 |
| 카탈로그 5종·파라미터 시트 | 벤더 데이터 입력 템플릿의 기준 서식 |

주차별 마일스톤 변경: **사실상 0** (화면의 주어와 카피 교체가 주 작업).

---

## 6. 정직한 한계와 남은 검증

- **TAM은 수수하다.** 이중 산출물을 합쳐도 니치 B2B 도구다. 상방 주장은 세 개만 허용: ① 데이터 플라이휠(견적 사례 축적 = 벤더 독립적 지식 자산) ② 엔진의 도메인 이식성(병원·공항·주차 로봇 등 사이징 질문이 있는 곳) ③ 해외 확장 여지(§4 해외 근거 — 카테고리 선례 실재). 이를 넘는 주장은 다시 억지가 된다.
- **상위 시장 강자의 하향 진입이 장기 리스크다.** Demo3D·Visual Components는 전문 인력 전제라 현재는 비경쟁(§4 함의 2 — 카테고리 검증자)이지만, 카테고리가 증명되면 가장 유력한 후발 진입자다. 방어선은 AMR 수직 특화와 선점 속도이며, 이는 "빨리 만들어 빨리 검증한다"는 8주 실행 계획과 이해가 일치한다.
- **프로젝트 목적 함수 기준으로는 상위권 기획이다.** 평가·면접에서 요구되는 것은 유니콘 서사가 아니라 지불자·페인·데이터 경로가 설명되는 기획이며, 본 제안은 셋 다 답이 있다. "이 사업의 크기를 정확히 안다"를 약점이 아니라 앞세울 것.
- **검증 액션 3건**: ① 벤더/SI 영업 1곳 인터뷰 — 정밀 제안 투자 기준과 시뮬 역량 실태 ② 구매 경험자 1명 인터뷰 — 품의서에 제3자 비교 자료가 실제로 쓰이는지 ③ 중견 3PL·풀필먼트 운영 담당 1명 인터뷰 — 신규 화주 계약 시 처리 능력 산정을 어떻게 하는지, 로봇 vs 인력 검토 실태(§3 세그먼트 제안의 *추정* 확정용). SSAFY 네트워크로 접근 가능.

---

## 7. 팀 결정 요청

1. 제품 재정의 승인 여부: **"AMR 사이징 엔진 — 판매자 제안서 생성기(주) + 구매자 비교 검증 리포트(부)"**
2. 승인 시 개정 범위: 개괄 §1~§3(제품 정의·시장 근거·여정에 5번째 피벗 기록) → 서비스 소개·주제 여정 문서 → 데모·랜딩의 주어/카피 교체
3. 검증 인터뷰 담당자 지정 (§6의 3건)
4. 첫 고객 세그먼트 채택 여부: **3PL·풀필먼트(중견 이커머스)** — §3 세그먼트 제안 참조. 채택 시 데모 서사·인테이크 템플릿을 WMS 주문 프로파일 기준으로 맞춘다(엔진 변경 없음)

> 본 제안서의 검증 경위(노벨티 → 벤더 알고리즘 → FMS 시장 → 사업성 → 견적 관행)는 그 자체가 주제 여정 문서의 다섯 번째 피벗 기록의 재료다 — 발표 Q&A에서 나올 공격을 전부 미리 맞아 본 과정이므로.
