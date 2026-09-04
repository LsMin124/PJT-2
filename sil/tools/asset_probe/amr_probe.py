"""Headless Isaac probe: spawn one candidate AMR on a flat floor, settle, drive straight, rotate; report kinematics + stability."""
import argparse, json, math, sys, time
ap = argparse.ArgumentParser()
ap.add_argument("--robot", required=True); ap.add_argument("--speed", type=float, default=0.5); ap.add_argument("--secs", type=float, default=4.0); ap.add_argument("--z", type=float, default=None); ap.add_argument("--lock", default="", help="comma list of revolute joints to lock (limits 0,0)")
args = ap.parse_args()
import os; R = os.environ.get("AMR_ASSET_DIR", os.path.expanduser("~/isaac_assets/Robots"))
SPECS = {
    "nova_carter": dict(usd=R + "/NVIDIA/NovaCarter/nova_carter.usd", art="", z=0.10, drive=["joint_wheel_left", "joint_wheel_right"], fwd=[1, 1], rot=[1, -1], radius=0.14, variants={"Sensors": "None"}),
    "o3dyn": dict(usd=R + "/Fraunhofer/O3dyn/o3dyn.usd", art="", z=0.10, drive=["wheel_fr_joint", "wheel_fl_joint", "wheel_rr_joint", "wheel_rl_joint"], fwd=[1, 1, 1, 1], rot=[1, -1, 1, -1], radius=0.17),
    "o3dyn_trimmed": dict(usd=R + "/Fraunhofer/O3dyn/o3dyn_trimmed.usd", art="", z=0.10, drive=["wheel_fr_joint", "wheel_fl_joint", "wheel_rr_joint", "wheel_rl_joint"], fwd=[1, 1, 1, 1], rot=[1, -1, 1, -1], radius=0.17),
    "forklift_b": dict(usd=R + "/IsaacSim/ForkliftB/forklift_b.usd", art="", z=0.10, drive=["back_wheel_drive"], fwd=[1], rot=None, radius=0.16, hold={"back_wheel_swivel": 0.0}),
    "iw_hub": dict(usd=R + "/Idealworks/iwhub/iw_hub.usd", art="", z=0.10, drive=["left_wheel_joint", "right_wheel_joint"], fwd=[1, 1], rot=[1, -1], radius=0.08),
}
spec = SPECS[args.robot]
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import numpy as np, omni.timeline, omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, UsdLux, Gf
import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.simulation_manager import SimulationManager

def log(*a): print("[probe]", *a, flush=True)
stage_utils.set_stage_up_axis("Z"); stage_utils.set_stage_units(meters_per_unit=1.0)
stage = omni.usd.get_context().get_stage()
ground = UsdGeom.Cube.Define(stage, "/World/ground"); ground.GetSizeAttr().Set(1.0)
xf = UsdGeom.XformCommonAPI(ground); xf.SetTranslate(Gf.Vec3d(0, 0, -0.1)); xf.SetScale(Gf.Vec3f(200, 200, 0.2))
UsdPhysics.CollisionAPI.Apply(ground.GetPrim())
UsdLux.DistantLight.Define(stage, "/World/light").CreateIntensityAttr(3000)
stage_utils.add_reference_to_stage(usd_path=spec["usd"], path="/World/robot")
rp = stage.GetPrimAtPath("/World/robot")
for k, v in spec.get("variants", {}).items():
    vs = rp.GetVariantSet(k)
    if vs and vs.IsValid(): vs.SetVariantSelection(v); log("variant", k, "=", v)
UsdGeom.XformCommonAPI(rp).SetTranslate(Gf.Vec3d(0, 0, spec["z"] if args.z is None else args.z))
if args.lock:
    for p in Usd.PrimRange(rp):
        if p.GetName() in args.lock.split(","):
            p.GetAttribute("physics:lowerLimit").Set(0.0); p.GetAttribute("physics:upperLimit").Set(0.0); log("locked", p.GetPath())
# find articulation root
art_path = None
for p in Usd.PrimRange(rp):
    if p.HasAPI(UsdPhysics.ArticulationRootAPI): art_path = p.GetPath().pathString; break
log("articulation root:", art_path)
SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")
robot = Articulation(art_path)
tl = omni.timeline.get_timeline_interface(); tl.play()
for _ in range(5): app.update()
names = list(robot.dof_names); log("dofs:", len(names), names)
def pose():
    pos, ori = robot.get_world_poses()
    p = np.asarray(pos.numpy()).reshape(-1)[:3]; q = np.asarray(ori.numpy()).reshape(-1)[:4]
    yaw = math.atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2))
    return p, yaw
res = {"robot": args.robot, "dofs": len(names)}
zs = []
for i in range(120):
    app.update(); zs.append(float(pose()[0][2]))
p0, yaw0 = pose()
try:
    dp = np.asarray(robot.get_dof_positions().numpy()).reshape(-1)
    res["dof_pos_after_settle"] = {n: round(float(v), 3) for n, v in zip(names, dp)}
    _, ori = robot.get_world_poses(); q = np.asarray(ori.numpy()).reshape(-1)[:4]
    roll = math.atan2(2*(q[0]*q[1]+q[2]*q[3]), 1-2*(q[1]**2+q[2]**2)); pitch = math.asin(max(-1,min(1,2*(q[0]*q[2]-q[3]*q[1]))))
    res["roll_pitch_deg"] = [round(math.degrees(roll),2), round(math.degrees(pitch),2)]
except Exception as e: log("diag failed", e)
res["settle_z"] = round(float(zs[-1]), 4); res["settle_bounce"] = round(float(max(zs[30:]) - min(zs[30:])), 4)
res["settle_xy_drift"] = round(float(np.hypot(p0[0], p0[1])), 4)
if not np.all(np.isfinite(p0)) or abs(p0[2]) > 10: res["status"] = "BLOWUP"; print("RESULT", json.dumps(res), flush=True); app.close(); sys.exit(0)
idx = robot.get_dof_indices(spec["drive"])
w = args.speed / spec["radius"]
if spec.get("hold"):
    hidx = robot.get_dof_indices(list(spec["hold"].keys()))
    robot.set_dof_position_targets(positions=np.array([list(spec["hold"].values())], dtype=np.float32), dof_indices=hidx)
robot.set_dof_velocity_targets(velocities=np.array([[w * s for s in spec["fwd"]]], dtype=np.float32), dof_indices=idx)
n = int(args.secs * 60); samples = []
for i in range(n):
    app.update()
    if (i + 1) % 15 == 0:
        p, yaw = pose(); samples.append((round((i + 1) / 60, 3), float(p[0]), float(p[1]), float(p[2]), yaw))
p1, yaw1 = pose()
d = np.hypot(p1[0] - p0[0], p1[1] - p0[1])
travel_dir = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
# speed over last second
(ta, xa, ya, _, _), (tb, xb, yb, _, _) = samples[-5], samples[-1]
v_last = np.hypot(xb - xa, yb - ya) / (tb - ta)
res.update({"cmd_wheel_rad_s": round(w, 3), "cmd_speed": args.speed, "dist_m": round(float(d), 4), "v_last_1s": round(float(v_last), 4),
            "eff_radius": round(float(v_last / w), 4), "yaw_drift_deg": round(math.degrees(math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0))), 2),
            "travel_vs_heading_deg": round(math.degrees(math.atan2(math.sin(travel_dir - yaw0), math.cos(travel_dir - yaw0))), 2),
            "z_change": round(float(p1[2] - p0[2]), 4), "samples": [(t, round(x, 3), round(y, 3), round(z, 3), round(math.degrees(yw), 1)) for t, x, y, z, yw in samples]})
# stop, then rotate in place
robot.set_dof_velocity_targets(velocities=np.zeros((1, len(idx)), dtype=np.float32), dof_indices=idx)
for _ in range(60): app.update()
p2, yaw2 = pose(); res["stop_coast_m"] = round(float(np.hypot(p2[0] - p1[0], p2[1] - p1[1])), 4)
if spec.get("rot"):
    robot.set_dof_velocity_targets(velocities=np.array([[w * s for s in spec["rot"]]], dtype=np.float32), dof_indices=idx)
    for _ in range(120): app.update()
    p3, yaw3 = pose()
    dyaw = math.atan2(math.sin(yaw3 - yaw2), math.cos(yaw3 - yaw2))
    res["rot_2s_deg"] = round(math.degrees(dyaw), 1); res["rot_rate_rad_s"] = round(dyaw / 2.0, 3)
    res["rot_xy_wander_m"] = round(float(np.hypot(p3[0] - p2[0], p3[1] - p2[1])), 4)
    if abs(dyaw) > 1e-3: res["eff_track_m"] = round(abs(2 * w * spec["radius"] / (dyaw / 2.0)), 4)
res["status"] = "OK" if np.all(np.isfinite(p2)) else "NAN"
print("RESULT", json.dumps(res), flush=True)
tl.stop(); app.close()
