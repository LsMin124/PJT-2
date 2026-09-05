# PJT-2 — AMR 도입 견적 서비스 · 시뮬레이션 파트 (SSAFY 팀 루돌프)

문서 사이트(GitHub Pages)와 참고 자료, SIL(가상 시운전) 구현 트랙의 개인 저장소.
팀 공용 레포는 GitLab `S15P21A106`이며, 시뮬 코드의 정본은 그쪽 `2_Simulation/`이다.

| 경로 | 내용 |
|---|---|
| `index.html` | 문서 사이트 인덱스 (Pages 엔트리) |
| `pages/plan` `research` `tech` `archive` `working` `product` `ref` | 분류별 문서 페이지 (기획 / 조사 / 기술 / 부록 / 작업 / 제품 / 용어사전) |
| `docs/` | 참고 파일 — **사실의 단일 원천 `AMR_기획_기준선_v3.md`**(2026-09-03), 통합 아키텍처 v3.10 `FMS-WMS_통합_아키텍처.html`·요약본, 기획안·조사 md, PDF·zip(비추적) |
| `sil/` | Isaac Sim 6.0.1 · ROS 2 Humble 구현 트랙 — T1~T3 완료, T4 에이전트 뼈대 완료, 캘리브레이션·에셋 조사 도구. 상세는 `sil/README.md` |

- 새 문서는 해당 분류 폴더에 추가하고 `index.html`에 등록
- 문서 간 사실 충돌 시 `docs/AMR_기획_기준선_v3.md`가 우선(전체개괄·보충자료 v2를 대체)
- `sil/`의 공유 코드(t1·t3·t4)는 팀 레포 `2_Simulation/`이 정본이고 여기는 작업 사본 — 두 사본은 경로 문자열(`sil/` ↔ `2_Simulation/`)만 다르다
