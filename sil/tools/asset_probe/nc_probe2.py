"""Nova Carter with NVIDIA's own benchmark recipe: WheeledRobot + GroundPlane, spawn at z=0, drive via apply_wheel_actions."""
import json, math, sys
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import numpy as np, omni.timeline
from isaacsim.core.api import World
from isaacsim.core.api.objects import GroundPlane
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.wheeled_robots.robots import WheeledRobot
import os; R = os.environ.get("AMR_ASSET_DIR", os.path.expanduser("~/isaac_assets/Robots"))
usd = sys.argv[1] if len(sys.argv) > 1 else R + "/NVIDIA/NovaCarter/nova_carter.usd"
world = World(stage_units_in_meters=1.0, physics_dt=1/60, rendering_dt=1/60)
GroundPlane("/World/ground", z_position=0.0)
robot = WheeledRobot(prim_path="/World/robot", wheel_dof_names=["joint_wheel_left", "joint_wheel_right"], create_robot=True, usd_path=usd, position=np.array([0, 0, 0.0]))
try:
    import omni.usd
    st = omni.usd.get_context().get_stage(); rp = st.GetPrimAtPath("/World/robot")
    vs = rp.GetVariantSet("Sensors"); vs.SetVariantSelection("None"); print("[nc2] Sensors=None", flush=True)
except Exception as e: print("[nc2] variant skip", e, flush=True)
world.reset(); robot.initialize()
def pose():
    p, q = robot.get_world_pose(); p = np.asarray(p); q = np.asarray(q)
    yaw = math.atan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]**2+q[3]**2)); pitch = math.asin(max(-1,min(1,2*(q[0]*q[2]-q[3]*q[1]))))
    return p, yaw, pitch
for _ in range(120): world.step(render=False)
p0, yaw0, pitch0 = pose(); print("[nc2] settle", p0.round(4), "pitch_deg", round(math.degrees(pitch0),2), "dof", robot.dof_names, np.asarray(robot.get_joint_positions()).round(3), flush=True)
w = 0.5 / 0.14
robot.apply_wheel_actions(ArticulationAction(joint_positions=None, joint_efforts=None, joint_velocities=np.array([w, w])))
xs = []
for i in range(240):
    world.step(render=False)
    if (i+1) % 60 == 0: p, yaw, pitch = pose(); xs.append((round((i+1)/60,2), *p.round(3).tolist(), round(math.degrees(yaw),1), round(math.degrees(pitch),1)))
p1, yaw1, _ = pose()
d = float(np.hypot(p1[0]-p0[0], p1[1]-p0[1]))
print("RESULT", json.dumps({"usd": usd.split("Robots/")[-1], "dist_m": round(d,4), "yaw_drift_deg": round(math.degrees(yaw1-yaw0),2), "samples": xs}), flush=True)
app.close()
