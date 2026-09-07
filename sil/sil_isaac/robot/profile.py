"""RobotProfile — 기종 하나를 시뮬에 올리는 데 필요한 숫자 전부 (omni 불필요).

값의 출처: iw.hub 는 t3_warehouse README 의 에셋 실측(2026-08-31)과 calibration/turn_probe(2026-09-06).
새 기종은 이 dataclass 하나를 채우면 spawn·surgery·lidar·factsheet 가 같은 값을 본다.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Sequence, Tuple

ASSETS = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0"


@dataclass(frozen=True)
class Box:
    """축 정렬 박스 (중심 xyz, 크기 xyz) — 헐 콜라이더용."""
    center: Tuple[float, float, float]
    size: Tuple[float, float, float]


@dataclass(frozen=True)
class Sphere:
    """구 콜라이더 (중심 xyz, 반경)."""
    center: Tuple[float, float, float]
    radius: float


@dataclass(frozen=True)
class RobotProfile:
    name: str
    usd_path: str
    wheel_dof_names: Tuple[str, str]
    wheel_radius: float          # m — 접촉 구체 반경이자 DifferentialController 입력
    track: float                 # m — 좌우 구동륜 간격 (wheel_base 인자)
    max_lin: float = 1.2         # m/s — 안전 클램프 (에셋 한계 아님)
    max_ang: float = 1.5         # rad/s
    spawn_z: float = 0.08        # 축 평형 높이 + 여유
    chassis_link: str = "chassis"  # 리지드바디 링크 이름 — 라이다는 반드시 여기 밑에
    lidar_z: float = 0.45        # 섀시 원점 기준 라이다 높이 (omap 스캔 밴드 0.2~1.2 안)
    # ── 에셋 수술 (iw.hub 전용 항목은 비우면 건너뜀) ──
    locked_prismatic_joints: Tuple[str, ...] = ()
    locked_revolute_joints: Tuple[str, ...] = ()
    low_friction_materials: Tuple[str, ...] = ()
    broken_collisions: Tuple[str, ...] = ()
    hull: Optional[Box] = None
    gliders: Tuple[Sphere, ...] = ()
    glider_friction: float = 0.03
    wheel_links: Tuple[str, ...] = ()          # 실린더 → 구체 접촉으로 바꿀 바퀴 링크
    wheel_collision_child: str = "Cylinder"    # 비활성할 원본 콜라이더 이름
    wheel_material: str = "wheel_material"     # 접촉 구체가 상속할 물리 재질 프림
    # ── factsheet 용 (선택) ──
    length: float = 0.0
    width: float = 0.0
    height: float = 0.0
    front_overhang: float = 0.0
    rear_overhang: float = 0.0
    swing_radius: float = 0.0

    def dof_index(self, side: str) -> int:
        return 0 if side == "left" else 1


IWHUB = RobotProfile(
    name="iw.hub",
    usd_path=ASSETS + "/Isaac/Robots/Idealworks/iwhub/iw_hub.usd",
    wheel_dof_names=("left_wheel_joint", "right_wheel_joint"),
    # 실측 운동학 (에셋 콜리전·비주얼 bbox 실측 — T1의 0.115/0.413은 오류였음:
    # 바퀴 실린더·메시 반경 0.081, 휠 트랙 tr y=±0.29 → 0.58)
    wheel_radius=0.08,
    track=0.58,
    spawn_z=0.08,                 # 축 평형 0.07(지면 -0.01 + r0.08) 위 1cm
    lidar_z=0.45,                 # 차체(0.35) 위, omap 스캔 밴드(0.2~1.2) 안
    # 무구동 lift_joint(프리즘틱) 잠금 — 중력에 서서히 침하해 리프트 콜리전이 바닥에 닿으면
    # 주행이 점진 감속 후 고착(시간 기반 ~40s 동결 실측)되는 것을 차단
    locked_prismatic_joints=("lift_joint",),
    # 캐스터 무력화 — 스월 축과 바퀴가 동축(트레일 0)이라 자기 정렬이 없고, 관절 한계(±2π)에
    # 걸리면 바퀴가 옆으로 꺾인 채 앵커가 되어 구동륜이 공회전 (3회 주행 실측)
    locked_revolute_joints=("left_swivel_joint", "right_swivel_joint", "left_caster_joint", "right_caster_joint"),
    low_friction_materials=("caster_material",),
    # 에셋의 섀시/리프트/스월/캐스터 Collision 은 extent 가 ±50(cm 시절 값)에 boundingCube 근사라
    # ×100 으로 깨져 있음(섀시 박스 142x65x10 m, 캐스터 구체 r7 m) → 전부 비활성
    broken_collisions=("chassis/Collision", "chassis/Collision_01", "lift/Collision",
                       "left_swivel/Collision", "right_swivel/Collision",
                       "left_caster/Collision", "right_caster/Collision"),
    # 섀시 외곽 박스(벽·장애물 접촉용, 바닥과 여유 0.03) — 시각 메시 bbox 실측 근사
    hull=Box(center=(-0.32, 0.0, 0.13), size=(1.40, 0.62, 0.22)),
    # 뒤 글라이더 2개만, 바퀴 바닥보다 3mm 높게(-0.077 vs -0.08) — 4점 코플레이너로 두면
    # 하중이 저마찰 글라이더에만 실려 구동륜 수직력이 0이 된다(실측: 완전 무이동).
    gliders=(Sphere((-0.677, 0.093, -0.022), 0.055), Sphere((-0.677, -0.093, -0.022), 0.055)),
    glider_friction=0.03,
    # 바퀴 실린더 → 해석적 구체 접촉: PhysX 가 실린더를 컨벡스 헐로 근사하면 다각형 바퀴가 되어 덜컹거림
    wheel_links=("left_wheel", "right_wheel"),
    length=1.43, width=0.66, height=0.41, front_overhang=0.40, rear_overhang=1.03, swing_radius=1.09,
)

PROFILES = {"iw.hub": IWHUB, "iwhub": IWHUB}


def get_profile(name: str) -> RobotProfile:
    try:
        return PROFILES[name.lower()]
    except KeyError as e:
        raise KeyError(f"unknown robot profile {name!r}; known: {sorted(PROFILES)}") from e
