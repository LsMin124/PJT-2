"""Inventory a robot USD: articulation, joints/drives, colliders, mass, sensors, bbox."""
import sys, os, re, math, json
from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf, Tf

SENSOR_PAT = re.compile(r"lidar|camera|imu|contact|radar|ultrason|sensor|hawk|hesai|velodyne|ouster", re.I)

def attr(prim, name, default=None):
    a = prim.GetAttribute(name)
    if a and a.HasAuthoredValue():
        try: return a.Get()
        except Exception: return default
    return default

def bbox_of(prim, cache):
    r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    if r.IsEmpty(): return None
    mn, mx = r.GetMin(), r.GetMax()
    return [round(v, 4) for v in mn], [round(v, 4) for v in mx]

def local_bbox(prim):
    c = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy", "guide"], useExtentsHint=False)
    r = c.ComputeLocalBound(prim).ComputeAlignedRange()
    if r.IsEmpty(): return None
    return r.GetSize()

def inspect(path):
    st = Usd.Stage.Open(path)
    out = {"file": path, "defaultPrim": st.GetDefaultPrim().GetPath().pathString if st.GetDefaultPrim() else None,
           "upAxis": UsdGeom.GetStageUpAxis(st), "metersPerUnit": UsdGeom.GetStageMetersPerUnit(st)}
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"], useExtentsHint=False)
    types, art_roots, rbs, colliders, masses, joints, sensors, graphs, variants, refs = {}, [], [], {}, [], [], [], [], [], set()
    unresolved = 0
    for prim in Usd.PrimRange(st.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
        tn = prim.GetTypeName() or "(untyped)"
        types[tn] = types.get(tn, 0) + 1
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            art_roots.append(prim.GetPath().pathString)
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            rbs.append(prim.GetPath().pathString)
        if prim.HasAPI(UsdPhysics.MassAPI):
            m = attr(prim, "physics:mass")
            if m: masses.append((prim.GetPath().pathString, float(m)))
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            approx = attr(prim, "physics:approximation", "(exact-mesh)") if tn == "Mesh" else tn
            en = attr(prim, "physics:collisionEnabled", True)
            key = f"{approx}{'' if en else ' [disabled]'}"
            colliders[key] = colliders.get(key, 0) + 1
        if prim.IsA(UsdPhysics.Joint):
            j = UsdPhysics.Joint(prim)
            b0 = [p.pathString.split("/")[-1] for p in j.GetBody0Rel().GetTargets()]
            b1 = [p.pathString.split("/")[-1] for p in j.GetBody1Rel().GetTargets()]
            d = {"path": prim.GetPath().pathString, "type": tn, "body0": b0, "body1": b1,
                 "axis": attr(prim, "physics:axis"), "lower": attr(prim, "physics:lowerLimit"), "upper": attr(prim, "physics:upperLimit"),
                 "enabled": attr(prim, "physics:jointEnabled", True), "excludeArt": attr(prim, "physics:excludeFromArticulation", False)}
            for dt in ("angular", "linear"):
                if prim.HasAPI(UsdPhysics.DriveAPI, dt):
                    d["drive"] = {"kind": dt, "type": attr(prim, f"drive:{dt}:physics:type"),
                                  "stiffness": attr(prim, f"drive:{dt}:physics:stiffness"), "damping": attr(prim, f"drive:{dt}:physics:damping"),
                                  "maxForce": attr(prim, f"drive:{dt}:physics:maxForce"), "targetVel": attr(prim, f"drive:{dt}:physics:targetVelocity")}
            mv = attr(prim, "physxJoint:maxJointVelocity")
            if mv is not None: d["maxJointVelocity"] = mv
            joints.append(d)
        if tn in ("OmniGraph",):
            graphs.append(prim.GetPath().pathString)
        nm = prim.GetName()
        if SENSOR_PAT.search(nm) or SENSOR_PAT.search(tn) or tn in ("Camera", "Lidar", "OmniLidar", "IsaacImuSensor", "IsaacContactSensor", "IsaacRtxLidarSensorAPI"):
            if tn not in ("Mesh", "Material", "Shader", "Scope", "GeomSubset"):
                sens = {"path": prim.GetPath().pathString, "type": tn}
                cfg = attr(prim, "omni:sensor:Core:sensorConfig") or attr(prim, "sensorModelConfig")
                if cfg: sens["config"] = str(cfg)
                for k in ("rotationRate", "horizontalFov", "verticalFov", "maxRange", "minRange"):
                    v = attr(prim, k)
                    if v is not None: sens[k] = v
                sensors.append(sens)
        vs = prim.GetVariantSets()
        for vn in vs.GetNames():
            variants.append({"prim": prim.GetPath().pathString, "set": vn, "sel": vs.GetVariantSet(vn).GetVariantSelection(), "opts": vs.GetVariantSet(vn).GetVariantNames()})
        for stack in prim.GetPrimStack():
            for r in stack.referenceList.GetAddedOrExplicitItems():
                refs.add(r.assetPath)
            for r in stack.payloadList.GetAddedOrExplicitItems():
                refs.add(r.assetPath)
        if prim.GetPath().pathString.count("/") <= 3 and prim.GetTypeName() == "" and not prim.IsInstanceProxy():
            pass
    # wheel radius estimate: for angular-drive revolute joints, local bbox of body1 prim
    root = st.GetDefaultPrim() or st.GetPseudoRoot()
    b = bbox_of(root, cache)
    out["bbox_min_max"] = b
    if b: out["size"] = [round(b[1][i] - b[0][i], 3) for i in range(3)]
    for d in joints:
        if d["type"] == "PhysicsRevoluteJoint" and d.get("drive", {}).get("kind") == "angular" and d["body1"]:
            tgt = UsdPhysics.Joint(st.GetPrimAtPath(d["path"])).GetBody1Rel().GetTargets()[0]
            p = st.GetPrimAtPath(tgt)
            if p:
                s = local_bbox(p)
                if s: d["body1_size"] = [round(v, 4) for v in s]
    out.update({"articulationRoots": art_roots, "rigidBodies": len(rbs), "rigidBodyNames": [r.split("/")[-1] for r in rbs][:40],
                "colliders": colliders, "massTotal": round(sum(m for _, m in masses), 3), "masses": masses[:40],
                "joints": joints, "sensors": sensors, "omniGraphs": graphs, "variants": variants,
                "primTypes": dict(sorted(types.items(), key=lambda kv: -kv[1])[:25]), "refs": sorted(refs)[:40]})
    return out

if __name__ == "__main__":
    for p in sys.argv[1:]:
        print(json.dumps(inspect(p), indent=1, default=str))
