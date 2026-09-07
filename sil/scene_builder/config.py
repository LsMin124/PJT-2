"""경로·에셋 URL·치수 상수 — 원본 두 파일의 상수 블록을 값 그대로 모음 (다른 모듈은 여기서만 import)."""
import os

# ── 경로 — 원본 build_scene.py 가 있던 sil/t3_warehouse/ 를 HERE 로 유지해 상대경로·산출물 위치를 보존 ──
HERE = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "t3_warehouse")
MAP_DIR = os.path.join(HERE, "..", "t3_warehouse_map", "map")
OUT_DIR = os.path.join(HERE, "out")
USD_PATH = os.path.join(HERE, "warehouse_scene.usd")     # 산출 씬 (원본 usd_path)

ASSETS = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0"
SHELF_USD = ASSETS + "/Isaac/Environments/Simple_Warehouse/Props/SM_RackShelf_01.usd"
FRAME_USD = ASSETS + "/Isaac/Environments/Simple_Warehouse/Props/SM_RackFrame_03.usd"

MAT_DIR = ASSETS + "/Isaac/Environments/Simple_Warehouse/Materials"
TEX = MAT_DIR + "/Textures"

CELL = 0.1                      # m/셀 (맵 스크립트와 동일)
WALL_H = 9.0                    # 처마 기둥 상단 +9.000 (DXF 치수선 실측 — 시각용, 라이다 무관)
CENTER_COL_Z = 11.0             # 중앙(릿지 지지) 기둥 상단 +11.000 (실측)
CONV_H = 0.9                    # 라이다 평면(0.7m) 위 — 가시(콜라이더)
TABLE_H = 0.9                   # 작업대 높이 — 라이다 평면 위 (0.6m 큐브 관통 사고 교훈)

# 컨베이어 비주얼 — ConveyorBelt_A08 직선 섹션 (컴포즈 실측 2.719x1.15x1.17, 피벗 동측 끝)
CONV_USD = ASSETS + "/Isaac/Props/Conveyors/ConveyorBelt_A08.usd"
CONV_SEC_L = 2.719
# (x0, y0, 길이, 세로 여부) — 맵 v5.6 CONVEYORS/CONVEYORS_V와 동일 좌표
CONVEYORS_VIS = [(14.6, 38.9, 16.0, False), (14.6, 70.9, 16.0, False),
                 (94.6, 43.1, 15.0, False), (94.6, 75.1, 15.0, False),
                 (101.0, 34.4, 7.4, False),                 # 패킹→출고 연결 (동진)
                 (107.5, 35.3, 7.8, True)]                  # 패킹→출고 연결 (북상)
UNIT_L = 3.0                    # 렉 유닛 길이 (y) — v6.0 구역제(반쪽 3랙×3m).
DECK_SCALE = UNIT_L / 4.0       # SM_RackShelf 베이는 4.0m — 장축 0.75 스케일로 정합
RACK_D = 1.08                   # 데크 실측 깊이 (그리드 선언 1.2 — 풋프린트 내 배치)
RACK_W = 2.4                    # 더블로우 그리드 폭 (맵 RACK_UNIT_D 1.2 x 2)
DECK_Z = (1.35, 2.7)            # 빔 2단 + 바닥 = 3단 파렛트 랙 — 파렛트 유닛로드
                                # (0.96m: 파렛트0.21+박스0.5+토퍼0.25)가 단높이에
                                # 들어가는 실규격. 0.75m 간격 4단은 박스가 위층
                                # 빔·상판을 관통(실측 — "랙을 뚫어버렸네")

# 화물 드레싱 (v5.8) — 구형 창고 캐릭터: 바닥 파렛트 블록(셀값 6) + 랙 박스
PROPS = ASSETS + "/Isaac/Environments/Simple_Warehouse/Props/"
PALLET_USD = PROPS + "SM_PaletteA_01.usd"     # 컴포즈 실측 1.21x1.00, h0.21
BOX_A = PROPS + "SM_CardBoxA_01.usd"          # 0.70x0.50x0.50
BOX_B = PROPS + "SM_CardBoxB_01.usd"          # 0.50x0.50x0.50
BOX_C = PROPS + "SM_CardBoxC_01.usd"          # 0.50x0.50x0.25
PALLET_L, PALLET_D = 1.27, 1.06               # 배치 피치 (장변 1.21, 단변 1.00)

# 사무실 (맵 v5.7 재구축 구역과 동일 좌표) — 벽 3.0m + 가구(시각 전용)
OFFICES = [(14.6, 25.7, 29.5, 37.5), (14.6, 77.0, 29.5, 89.1)]
OFFICE_H = 3.0
OPROPS = ASSETS + "/Isaac/Environments/Office/Props/"
# (에셋, x, y, z, rot_z) — 컴포즈 bbox 실측: TableWorkingDouble 1.7x1.7 h0.75(바닥 피벗),
# ChairOffice_A 시트가 +x향, TableB 0.8x2.8(장축 y), Sofa 0.82x2.04(장축 y)
DESKS = [(23.0, 28.0), (26.4, 28.0), (23.0, 31.6), (26.4, 31.6), (16.9, 34.5), (20.3, 34.5),
         (23.0, 80.0), (26.4, 80.0), (23.0, 83.4), (26.4, 83.4), (16.9, 79.5), (20.3, 79.5)]
MEETINGS = [(17.6, 28.4), (17.6, 86.9)]       # 회의실 테이블(의자 4는 코드로)
PLANTS = [(15.4, 36.6), (28.6, 26.4), (15.4, 77.9), (28.6, 88.3)]
# 벽 집기 — 현장 사무실 드레싱 (문 개구부 회피: 동벽 문 y31.2~32.6 / 82.4~83.8,
# 북·남벽 문 x25.9~27.3). 마커보드·파일캐비닛+프린터·책장·소화전함·바인더
OFFICE_WALL = [
    ("SM_MarkerBoard.usd", 14.85, 28.4, 0.95, 0.0),      # 남 회의실 서벽
    ("SM_MarkerBoard.usd", 14.85, 34.6, 0.95, 0.0),      # 남 사무 구역 서벽
    ("SM_FileCabinet_01.usd", 26.9, 26.2, 0.0, 90.0),    # 남벽 캐비닛 뱅크 — 서랍면이
    ("SM_FileCabinet_02.usd", 27.4, 26.2, 0.0, 90.0),    # 좁은 면(+x)이라 90°가 실내향(실측)
    ("SM_FileCabinet_01.usd", 27.9, 26.2, 0.0, 90.0),
    ("SM_Printer.usd", 27.4, 26.2, 1.34, 0.0),
    ("SM_RingBinderStackA.usd", 26.85, 26.15, 1.34, 10.0),
    ("SM_BookcaseA.usd", 29.15, 34.5, 0.0, 180.0),       # 동벽 (문 31.2~32.6 회피)
    ("SM_FireCabinetA.usd", 29.4, 30.2, 0.55, 180.0),
    ("SM_MarkerBoard.usd", 14.85, 86.9, 0.95, 0.0),      # 북 회의실 서벽
    ("SM_MarkerBoard.usd", 14.85, 79.5, 0.95, 0.0),      # 북 사무 구역 서벽
    ("SM_FileCabinet_01.usd", 26.9, 88.75, 0.0, -90.0),  # 북벽 캐비닛 뱅크 (서랍면 남향)
    ("SM_FileCabinet_02.usd", 27.4, 88.75, 0.0, -90.0),
    ("SM_FileCabinet_01.usd", 27.9, 88.75, 0.0, -90.0),
    ("SM_Printer.usd", 27.4, 88.75, 1.34, 180.0),
    ("SM_RingBinderStackA.usd", 27.85, 88.8, 1.34, 170.0),
    ("SM_BookcaseA.usd", 29.15, 80.5, 0.0, 180.0),       # 동벽 (문 82.4~83.8 회피)
    ("SM_FireCabinetA.usd", 29.4, 85.0, 0.55, 180.0),
]

# 철골 외피 — 설계도(포털 프레임 6m 모듈)의 윈드 컬럼 + 월 거트 (원본 [1c])
FRAME_XS = [20.1 + 6 * k for k in range(15)]              # 장변(남·북벽) 프레임 축
END_YS = [31.8, 38.0, 51.0, 57.3, 63.6, 83.0]             # 단변(서·동벽), 문 구간 회피
GIRT_Z = (2.7, 5.0, 7.2, 8.6)                             # 8.6 — 처마 9.0 하부 최상단 거트
DOOR_FREE_Y = ((26.0, 38.2), (44.8, 70.2), (76.8, 89.0))  # 서·동벽 문 구간 제외 스팬

# 작업대 비주얼 — packing_table (컴포즈 실측 2.47x0.78 h1.08, 벤치 rect 2.3~3.2x1.3), 원본 [2]
PACK_USD = ASSETS + "/Isaac/Props/PackingTable/packing_table.usd"

# 지게차 — 인바운드 파렛트 밴드(셀값 6, 15.6~17.6 x 47~66) 안 주차. 정적 콜라이더
# 3개(리지드 없음, 컴포즈 실측 1.21x3.49 h2.15)라 st 마스크 안 → V&V 오검출 없음 (원본 [2b])
FORK_POS = (16.6, 63.8)

# 렉 화물 — 피킹 낱박스 종류(에셋, 폭, 높이) · 적재 z = 바닥 + 데크 상판(배치 z +0.03), 원본 [3b]
BOX_KIND = ((BOX_A, 0.70, 0.50), (BOX_B, 0.50, 0.50), (BOX_C, 0.50, 0.25))
LOAD_Z = (0.0,) + tuple(dz + 0.03 for dz in DECK_Z)

# 바닥 마킹 선폭 (원본 [3c])
LW = 0.12

# 충전 스테이션 — 동벽 실내면 x (그리드 실측: 동벽 109.90~110.20), 원본 [3d]
E_WALL = 109.90

# 도어 — 서·동 박공벽 문 4곳 y 스팬 (그리드 실측 y 38.3~44.6 / 70.3~76.7), 원본 [3e]
DOOR_SPANS = ((38.3, 44.6), (70.3, 76.7))

# ── 박공지붕 (원본 roof_structure.py) — 설계도(64m x 96m 철골 창고) 실측 기하 ──
# DXF 치수선 실측 (MAT DUNG KHUNG DAU HOI BTCT TRUC 1-17 / MC KHUNG):
#   - ±0.000 → 측벽 기둥 상단 +9.000, 중앙 기둥 상단 +11.000
#   - 지붕 경사 i=15%, 반스팬 32.0m → 상승 4.8m (릿지 프레임 ~+13.8, 지붕면 ~+14.5)
#   - K1 철골 프레임 6m 간격, 박공 단부벽 기둥 6.4m x 10칸
#   - 릿지 환기 모니터(cua troi) 폭 4.5m, 지붕면 단부 베이 X-브레이싱
# 건물 외곽 (맵 벽 밴드 실측: x 14.0~110.4, y 25.3~89.7)
Y_EAVE0, Y_EAVE1 = 25.3, 89.7          # 처마선(남·북 벽 외면)
RIDGE_Y = (Y_EAVE0 + Y_EAVE1) / 2      # 57.5 — 중앙 기둥열(y≈57.3)이 릿지 지지열
X_ROOF0, X_ROOF1 = 13.7, 110.7         # 지붕 x 범위 (박공벽 밖 0.3m 오버행)
SLOPE = 0.15                           # i=15% (도면 4곳 명기)
EAVE_Z = 9.0                           # 기둥 상단 +9.000
CENTER_Z = 11.0                        # 중앙 기둥 상단 +11.000
CHORD0 = 9.6                           # 상현재 하면 (처마, 트러스 깊이 반영)
SHEET_OFF = 0.2                        # 상현재→지붕면(퍼린 두께)
VENT_HALF = 2.25                       # 환기 모니터 반폭 (4.5m/2)
VENT_X0, VENT_X1 = 20.1, 104.1         # 모니터 연장 = K1 프레임 구간
PURLIN_STEP = 1.4                      # 퍼린 간격 (경사면 투영 y 기준)
