"""에셋 런타임 수술 — RobotProfile 의 수술 항목을 스테이지에 적용한다 (원본 warehouse_sim.surgery 분할).

세 단계: (1) 무구동 조인트 잠금, (2) 캐스터 무력화 + 저마찰, (3) 깨진 콜리전 비활성 후 헐·글라이더·바퀴 구체 재구성.
"""
from __future__ import annotations

from .profile import RobotProfile

LIFT_DRIVE_STIFFNESS, LIFT_DRIVE_DAMPING = 1.0e5, 1.0e4
CASTER_DRIVE_STIFFNESS, CASTER_DRIVE_DAMPING = 1.0e4, 1.0e3
CASTER_FRICTION = 0.05


def lock_prismatic_joints(stage, root: str, joints) -> int:
    from pxr import UsdPhysics

    n = 0
    for name in joints:
        prim = stage.GetPrimAtPath(f"{root}/{name}")
        if not prim.IsValid():
            continue
        drv = UsdPhysics.DriveAPI.Apply(prim, "linear")
        drv.CreateTargetPositionAttr(0.0)
        drv.CreateStiffnessAttr(LIFT_DRIVE_STIFFNESS)
        drv.CreateDampingAttr(LIFT_DRIVE_DAMPING)
        n += 1
    return n


def neutralize_casters(stage, root: str, joints, materials) -> None:
    """스월·롤을 위치 드라이브로 잠그고 캐스터 재질을 저마찰로 — 뒤축은 '미끄럼 글라이더'가 된다."""
    from pxr import UsdPhysics

    for name in joints:
        prim = stage.GetPrimAtPath(f"{root}/{name}")
        if not prim.IsValid():
            continue
        d = UsdPhysics.DriveAPI.Apply(prim, "angular")
        d.CreateTargetPositionAttr(0.0)
        d.CreateStiffnessAttr(CASTER_DRIVE_STIFFNESS)
        d.CreateDampingAttr(CASTER_DRIVE_DAMPING)
    for name in materials:
        prim = stage.GetPrimAtPath(f"{root}/{name}")
        if not prim.IsValid():
            continue
        m = UsdPhysics.MaterialAPI.Apply(prim)
        m.CreateStaticFrictionAttr(CASTER_FRICTION)
        m.CreateDynamicFrictionAttr(CASTER_FRICTION)
        m.CreateRestitutionAttr(0.0)


def disable_collisions(stage, root: str, rel_paths) -> int:
    from pxr import UsdPhysics

    n = 0
    for rel in rel_paths:
        prim = stage.GetPrimAtPath(f"{root}/{rel}")
        if prim.IsValid():
            UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(False)
            n += 1
    return n


def _physics_material(stage, path: str, friction: float):
    from pxr import UsdPhysics, UsdShade

    mat = UsdShade.Material.Define(stage, path)
    api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    api.CreateStaticFrictionAttr(friction)
    api.CreateDynamicFrictionAttr(friction)
    api.CreateRestitutionAttr(0.0)
    return mat


def _invisible_collider(prim) -> None:
    from pxr import UsdGeom, UsdPhysics

    UsdPhysics.CollisionAPI.Apply(prim)
    UsdGeom.Imageable(prim).MakeInvisible()


def add_gliders(stage, root: str, profile: RobotProfile) -> None:
    from pxr import Gf, UsdGeom, UsdShade

    if not profile.gliders:
        return
    mat = _physics_material(stage, f"{root}/glider_mat", profile.glider_friction)
    for i, g in enumerate(profile.gliders):
        sp = UsdGeom.Sphere.Define(stage, f"{root}/{profile.chassis_link}/glider_{i}")
        sp.CreateRadiusAttr(g.radius)
        UsdGeom.Xformable(sp.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(*g.center))
        _invisible_collider(sp.GetPrim())
        UsdShade.MaterialBindingAPI.Apply(sp.GetPrim()).Bind(mat, materialPurpose="physics")


def replace_wheel_colliders(stage, root: str, profile: RobotProfile) -> None:
    """바퀴 원본 콜라이더 비활성 → 반경 wheel_radius 해석적 구체 (원점 = 축심). 마찰은 에셋 재질 상속."""
    from pxr import UsdGeom, UsdPhysics, UsdShade

    wmat = UsdShade.Material.Get(stage, f"{root}/{profile.wheel_material}")
    for link in profile.wheel_links:
        cyl = stage.GetPrimAtPath(f"{root}/{link}/{profile.wheel_collision_child}")
        if cyl.IsValid():
            UsdPhysics.CollisionAPI.Apply(cyl).CreateCollisionEnabledAttr(False)
        ws = UsdGeom.Sphere.Define(stage, f"{root}/{link}/contact_sphere")
        ws.CreateRadiusAttr(profile.wheel_radius)
        _invisible_collider(ws.GetPrim())
        if wmat:
            UsdShade.MaterialBindingAPI.Apply(ws.GetPrim()).Bind(wmat, materialPurpose="physics")


def add_hull(stage, root: str, profile: RobotProfile) -> None:
    from pxr import Gf, UsdGeom

    if profile.hull is None:
        return
    cb = UsdGeom.Cube.Define(stage, f"{root}/{profile.chassis_link}/hull")
    cb.GetSizeAttr().Set(1.0)
    xf = UsdGeom.Xformable(cb.GetPrim())
    xf.AddTranslateOp().Set(Gf.Vec3d(*profile.hull.center))
    xf.AddScaleOp().Set(Gf.Vec3f(*profile.hull.size))
    _invisible_collider(cb.GetPrim())


def apply_asset_fixes(stage, root: str, profile: RobotProfile) -> None:
    """프로파일에 적힌 수술을 전부 적용. 항목이 비어 있으면 그 단계는 건너뛴다."""
    lock_prismatic_joints(stage, root, profile.locked_prismatic_joints)
    neutralize_casters(stage, root, profile.locked_revolute_joints, profile.low_friction_materials)
    disable_collisions(stage, root, profile.broken_collisions)
    add_gliders(stage, root, profile)
    replace_wheel_colliders(stage, root, profile)
    add_hull(stage, root, profile)
