"""사무실 2개소 — 구역 판정(in_office) + 카펫 바닥·책상·회의 테이블·벽 집기·화분 드레싱 (pxr 필요)."""
from pxr import UsdGeom

from .config import DESKS, MEETINGS, OFFICES, OFFICE_WALL, OPROPS, PLANTS
from .gridutil import rnd
from .prims import add_asset, add_quad


def in_office(cx, cy):
    return any(x0 <= cx <= x1 and y0 <= cy <= y1 for x0, y0, x1, y1 in OFFICES)


def build_office_interior(stage):
    """[1b] 사무실 인테리어 — 카펫 바닥 + 가구(시각 전용, 그리드·플래너 무관)."""
    UsdGeom.Xform.Define(stage, "/World/office_furniture")
    for oi, (x0, y0, x1, y1) in enumerate(OFFICES):
        q = add_quad(stage, f"/World/office_furniture/floor_{oi}", x0, y0, x1, y1, 0.006, 4.0)
        UsdGeom.Gprim(q.GetPrim()).CreateDisplayColorAttr([(0.55, 0.56, 0.60)])
    n_furn = 0
    for i, (dx, dy) in enumerate(DESKS):
        # 데스크: 철제 작업 테이블(0.8x1.7 실측, 장축 y → 90° 회전) 등맞댄 2대 —
        # 유리·블랙 톤 TableWorkingDouble이 식당처럼 보인다는 피드백으로 교체
        add_asset(stage, f"/World/office_furniture/desk_{i}a", OPROPS + "SM_TableWorkSecurity.usd",
                  dx, dy - 0.42, rot_z=90.0)
        add_asset(stage, f"/World/office_furniture/desk_{i}b", OPROPS + "SM_TableWorkSecurity.usd",
                  dx, dy + 0.42, rot_z=90.0)
        # 의자: 강관 캔틸레버(SM_Chair, 시트 +x향) — 레드 디자이너 체어 대체
        add_asset(stage, f"/World/office_furniture/chair_{i}a", OPROPS + "SM_Chair.usd",
                  dx, dy - 1.05, rot_z=90.0)
        add_asset(stage, f"/World/office_furniture/chair_{i}b", OPROPS + "SM_Chair.usd",
                  dx, dy + 1.05, rot_z=-90.0)
        add_asset(stage, f"/World/office_furniture/mon_{i}a", OPROPS + "SM_MonitorPC_ON_1.usd",
                  dx, dy - 0.3, z=0.75, rot_z=90.0)
        add_asset(stage, f"/World/office_furniture/mon_{i}b", OPROPS + "SM_MonitorPC_ON_2.usd",
                  dx, dy + 0.3, z=0.75, rot_z=-90.0)
        # 사무 소품 — 파티션(0.64 x2, 책상 중앙 가로지름)·키보드·마우스·PC·서류(결정적)
        add_asset(stage, f"/World/office_furniture/prt_{i}a", OPROPS + "SM_Partition.usd",
                  dx - 0.64, dy, z=0.75)
        add_asset(stage, f"/World/office_furniture/prt_{i}b", OPROPS + "SM_Partition.usd",
                  dx, dy, z=0.75)
        add_asset(stage, f"/World/office_furniture/kb_{i}a", OPROPS + "SM_KeyboardPC.usd",
                  dx, dy - 0.58, z=0.76, rot_z=90.0)
        add_asset(stage, f"/World/office_furniture/kb_{i}b", OPROPS + "SM_KeyboardPC.usd",
                  dx, dy + 0.58, z=0.76, rot_z=-90.0)
        add_asset(stage, f"/World/office_furniture/ms_{i}a", OPROPS + "SM_MousePC.usd",
                  dx + 0.35, dy - 0.55, z=0.76)
        add_asset(stage, f"/World/office_furniture/ms_{i}b", OPROPS + "SM_MousePC.usd",
                  dx - 0.35, dy + 0.55, z=0.76)
        add_asset(stage, f"/World/office_furniture/pc_{i}", OPROPS + "SM_PC.usd",
                  dx + 0.62, dy - 0.38, rot_z=90.0)
        n_furn += 12
        if rnd("opaper", i) < 0.6:
            add_asset(stage, f"/World/office_furniture/pap_{i}", OPROPS + "SM_PaperStack_A.usd",
                      dx - 0.58, dy - 0.5, z=0.76, rot_z=rnd("opr", i) * 40 - 20)
            n_furn += 1
        if rnd("ophone", i) < 0.4:
            add_asset(stage, f"/World/office_furniture/ph_{i}", OPROPS + "SM_Phone.usd",
                      dx - 0.58, dy + 0.5, z=0.76, rot_z=180.0)
            n_furn += 1
    for i, (mx, my) in enumerate(MEETINGS):
        add_asset(stage, f"/World/office_furniture/meet_{i}", OPROPS + "SM_TableB.usd", mx, my)
        for j, (cx, cy, rz) in enumerate([(mx - 1.0, my - 0.7, 0), (mx - 1.0, my + 0.7, 0),
                                          (mx + 1.0, my - 0.7, 180), (mx + 1.0, my + 0.7, 180)]):
            add_asset(stage, f"/World/office_furniture/meet_{i}c{j}", OPROPS + "SM_Chair.usd",
                      cx, cy, rot_z=rz)
        n_furn += 5
    # 벽 집기 — 현장 사무실 드레싱 (문 개구부 회피: 동벽 문 y31.2~32.6 / 82.4~83.8,
    # 북·남벽 문 x25.9~27.3). 마커보드·파일캐비닛+프린터·책장·소화전함·바인더 (좌표: config.OFFICE_WALL)
    for i, (usd, wx, wy, wz, wr) in enumerate(OFFICE_WALL):
        add_asset(stage, f"/World/office_furniture/wall_{i}", OPROPS + usd, wx, wy, z=wz, rot_z=wr)
        n_furn += 1
    for i, (px, py) in enumerate(PLANTS):
        add_asset(stage, f"/World/office_furniture/plant_{i}", OPROPS + "SM_Plant01.usd", px, py)
        n_furn += 1
    print(f"[1b] 사무실 인테리어: 바닥 2 + 가구 {n_furn}점")
