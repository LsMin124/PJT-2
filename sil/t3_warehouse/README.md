# T3 — DXF 공장 씬 (씬 빌더)

"그리드가 곧 씬"이 원칙: map_gen(v6.3)의 occupancy grid와 랙 배치 좌표를 그대로
Isaac USD 씬으로 세운다. DES와 SIL이 단일 소스를 공유하는 구조의 물리 실체.

## 파일

| 파일 | 역할 |
|---|---|
| `build_scene.py` | 씬 빌더 — 그리드→벽·컨베이어 박스, rack_rows→NVIDIA 랙 부품 조립, omap V&V, 스크린샷 |
| `warehouse_scene.usd` | 산출 씬 (재생성물 — untracked) |
| `out/scene_*.png` | 검수 스크린샷 (탑뷰·조감·통로 시점) |
| `out/omap_occ.npy` | 씬→점유맵 역생성 결과 (V&V) |

## 실행

```bash
# 선행: sil/t3_warehouse_map/ 에서 map_gen.py 실행 (npy 생성)
cd ~/isaacsim && ./python.sh <repo>/sil/t3_warehouse/build_scene.py
```

## 구성 요소

- **벽·기둥** (그리드 셀값 1): 그리디 메싱으로 병합한 박스, 높이 3.0m
- **컨베이어** (셀값 5): 병합 박스 높이 0.9m — 라이다 평면(0.7m) 위라 가시 (T2 교훈)
- **랙**: full_warehouse와 동일한 NVIDIA 부품 실측 조립 —
  `SM_RackFrame_03`(0.127×1.0×3.0m) 5개/유닛 + `SM_RackShelf_01`(4.0×1.08m 데크) 4베이×3데크.
  유닛 16.13m = 4베이×4.0m + 프레임 마진 0.13 (팀 실측값과 일치 확인)
- **V&V**: `isaacsim.asset.gen.omap`으로 씬→점유맵을 역생성해 원 그리드와 diff —
  "씬이 그리드와 일치한다"를 자동 검증 (벽·컨베이어 재현율, 랙 풋프린트 점유율, 오검출 수)

## 남은 것 (T3 본편)

iw.hub + 라이다 투입(warehouse_sim.py), 스테이션 왕복 order 연쇄, 리프트 도킹 1장면,
사이클 타임 분해 + 도킹 시간 분포 실측(DES 환류).
