"""Kinematic constants from USD: drive-joint anchor positions (track), wheel collider radius, chassis collider extents."""
import sys, math
from pxr import Usd, UsdGeom, UsdPhysics, Gf
def colliders_under(prim):
    out = []
    for p in Usd.PrimRange(prim, Usd.TraverseInstanceProxies()):
        if not p.HasAPI(UsdPhysics.CollisionAPI): continue
        tn = p.GetTypeName(); g = {"type": tn, "name": p.GetName()}
        xf = UsdGeom.Xformable(p).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        sc = [xf.GetRow3(i).GetLength() for i in range(3)]
        if tn == "Cylinder":
            g["r"] = p.GetAttribute("radius").Get(); g["h"] = p.GetAttribute("height").Get(); g["axis"] = p.GetAttribute("axis").Get()
        elif tn == "Sphere":
            g["r"] = p.GetAttribute("radius").Get()
        elif tn == "Cube":
            g["size"] = p.GetAttribute("size").Get()
        else:
            g["approx"] = p.GetAttribute("physics:approximation").Get() if p.GetAttribute("physics:approximation") else None
        r = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default","render","proxy","guide"], useExtentsHint=False).ComputeWorldBound(p).ComputeAlignedRange()
        if not r.IsEmpty(): g["world_size"] = [round(v,3) for v in r.GetSize()]
        g["scale"] = [round(s,3) for s in sc]
        g["enabled"] = p.GetAttribute("physics:collisionEnabled").Get() if p.GetAttribute("physics:collisionEnabled") and p.GetAttribute("physics:collisionEnabled").HasAuthoredValue() else True
        out.append(g)
    return out
def world_pos(prim, local):
    xf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    return xf.Transform(Gf.Vec3d(*local))
for path in sys.argv[1:]:
    st = Usd.Stage.Open(path); print("=====", path.split("Robots/")[-1])
    root = st.GetDefaultPrim()
    drives = []
    for p in st.Traverse():
        if p.IsA(UsdPhysics.RevoluteJoint) and (p.HasAPI(UsdPhysics.DriveAPI, "angular") or __import__("os").environ.get("ALLJ")):
            j = UsdPhysics.Joint(p)
            b0 = j.GetBody0Rel().GetTargets(); b1 = j.GetBody1Rel().GetTargets()
            if not b0 or not b1: continue
            p0 = st.GetPrimAtPath(b0[0]); p1 = st.GetPrimAtPath(b1[0])
            lp0 = j.GetLocalPos0Attr().Get(); lp1 = j.GetLocalPos1Attr().Get()
            w0 = world_pos(p0, lp0); w1 = world_pos(p1, lp1)
            cols = colliders_under(p1)
            drives.append((p.GetName(), p0.GetName(), p1.GetName(), p.GetAttribute("physics:axis").Get(), w0, cols))
    for name, b0, b1, ax, w, cols in drives:
        cs = "; ".join(f"{c['type']}{'' if c['enabled'] else '[off]'} r={c.get('r')} h={c.get('h')} axis={c.get('axis','')} approx={c.get('approx','')} world={c.get('world_size')} scale={c['scale']}" for c in cols[:3])
        print(f"  {name:<24} {b0}->{b1} ax={ax} anchor_world=({w[0]:.3f},{w[1]:.3f},{w[2]:.3f})  colliders: {cs}")
    # pairwise anchor distances among drives named like wheels
    ws = [(n, w) for n, _, _, _, w, _ in drives if "wheel" in n.lower()]
    for i in range(len(ws)):
        for k in range(i+1, len(ws)):
            d = (ws[i][1]-ws[k][1]).GetLength(); print(f"  dist {ws[i][0]} <-> {ws[k][0]} = {d:.4f} m")
    # chassis / base colliders
    for p in st.Traverse():
        if p.HasAPI(UsdPhysics.RigidBodyAPI) and p.GetName().lower() in ("chassis","chassis_link","base_link","body","main_body"):
            cs = colliders_under(p)
            print(f"  {p.GetName()} colliders ({len(cs)}):")
            for c in cs[:8]: print(f"     {c}")
