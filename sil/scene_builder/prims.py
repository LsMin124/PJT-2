"""USD 프림 유틸 — 콜라이더 박스, UV 타일링 박스 메시, 수평 사각 메시, 참조 에셋 배치 (pxr 필요)."""
from pxr import Gf, Sdf, UsdGeom, UsdPhysics


def add_box(stage, path, x, y, w, h, z0, z1, collide=True):
    """(x,y)~(x+w,y+h), 높이 z0~z1 박스 (+콜라이더)."""
    cube = UsdGeom.Cube.Define(stage, path)
    cube.GetSizeAttr().Set(1.0)
    xf = UsdGeom.Xformable(cube.GetPrim())
    xf.AddTranslateOp().Set(Gf.Vec3d(x + w / 2, y + h / 2, (z0 + z1) / 2))
    xf.AddScaleOp().Set(Gf.Vec3f(w, h, z1 - z0))
    if collide:
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    return cube


def add_box_mesh(stage, path, x, y, w, h, z0, z1, uv_scale=4.0):
    """타일링 UV를 가진 박스 메시(옆 4면+윗면) + 콜라이더 — Cube prim은 UV가 없어
    텍스처 재질이 단색으로 뭉개진다(실측). 벽·기둥용."""
    mesh = UsdGeom.Mesh.Define(stage, path)
    x1, y1 = x + w, y + h
    pts = [(x, y, z0), (x1, y, z0), (x1, y1, z0), (x, y1, z0),
           (x, y, z1), (x1, y, z1), (x1, y1, z1), (x, y1, z1)]
    faces = [(0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7), (4, 5, 6, 7)]
    mesh.CreatePointsAttr(pts)
    mesh.CreateFaceVertexCountsAttr([4] * 5)
    mesh.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
    mesh.CreateExtentAttr([(x, y, z0), (x1, y1, z1)])
    mesh.CreateDoubleSidedAttr(True)
    hz = (z1 - z0) / uv_scale
    st = []
    for d in (w, h, w, h):
        u = d / uv_scale
        st += [(0, 0), (u, 0), (u, hz), (0, hz)]
    st += [(0, 0), (w / uv_scale, 0), (w / uv_scale, h / uv_scale), (0, h / uv_scale)]
    UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying).Set(st)
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    return mesh


def add_quad(stage, path, x0, y0, x1, y1, z, uv_scale, flip=False):
    """수평 사각 메시 + 타일링 UV (uv_scale m당 텍스처 1회) — 바닥·천장 시각용."""
    mesh = UsdGeom.Mesh.Define(stage, path)
    pts = [(x0, y0, z), (x1, y0, z), (x1, y1, z), (x0, y1, z)]
    mesh.CreatePointsAttr(pts)
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 3, 2, 1] if flip else [0, 1, 2, 3])
    mesh.CreateExtentAttr([(x0, y0, z - 0.01), (x1, y1, z + 0.01)])
    mesh.CreateDoubleSidedAttr(True)
    u, v = (x1 - x0) / uv_scale, (y1 - y0) / uv_scale
    st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
    st.Set([(0, 0), (u, 0), (u, v), (0, v)])
    return mesh


def add_asset(stage, path, usd, x, y, z=0.0, rot_z=None, instance=False, scale=None):
    """참조 에셋 배치 — 에셋 루트에 자체 xformOpOrder가 있어도 안전하도록
    래퍼 Xform이 이동·회전을 담당한다 (A08에서 실측한 예외의 일반화).
    instance=True면 USD 인스턴싱(대량 화물용 — 프로토타입 공유)."""
    w = UsdGeom.Xform.Define(stage, path)
    xf = UsdGeom.Xformable(w.GetPrim())
    xf.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
    if rot_z is not None:
        xf.AddRotateZOp().Set(rot_z)
    if scale is not None:
        xf.AddScaleOp().Set(Gf.Vec3f(*scale))
    a = UsdGeom.Xform.Define(stage, path + "/asset")
    a.GetPrim().GetReferences().AddReference(usd)
    if instance:
        a.GetPrim().SetInstanceable(True)
    return w
