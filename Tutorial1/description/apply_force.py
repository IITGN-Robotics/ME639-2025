import time
import numpy as np
import mujoco
from mujoco.viewer import launch_passive

XML_PATH = "/home/debojit/ME639/Practice1/description/scene.xml"

model = mujoco.MjModel.from_xml_path(XML_PATH)
data  = mujoco.MjData(model)

# Initial freejoint pose (qpos[0:3]=xyz, qpos[3:7]=wxyz)
data.qpos[0:3] = np.array([0.0, 0.0, 2.5])
data.qpos[3:7] = np.array([1.0, 0.0, 0.0, 0.0])
mujoco.mj_forward(model, data)

# Force window and vector
FORCE_START, FORCE_END = 3.0, 4.0      # seconds
F_world = np.array([-4.0, 0.0, 0.0])  # N, world frame (increase if friction is high)

# Get site & body ids
site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "force_site")
body_id = model.site_bodyid[site_id]

dt = model.opt.timestep

with launch_passive(model, data) as viewer:
    # draw initial frame
    viewer.sync()

    prev = time.perf_counter()
    accum = 0.0

    while viewer.is_running():
        now = time.perf_counter()
        accum += now - prev
        prev = now

        while accum >= dt:
            # clear external wrenches each step
            data.xfrc_applied[:] = 0.0

            t = data.time
            if FORCE_START <= t < FORCE_END:
                # world positions of site and body origin (≈ CoM if not overridden)
                p_site_w = data.site_xpos[site_id].copy()
                p_body_w = data.xipos[body_id].copy()
                r = p_site_w - p_body_w

                # torque = r x F (both in world frame)
                tau_world = np.cross(r, F_world)

                # apply wrench at the body equivalent to a force at the site
                data.xfrc_applied[body_id, 0:3] = F_world      # Fx, Fy, Fz
                data.xfrc_applied[body_id, 3:6] = tau_world    # Tx, Ty, Tz

            mujoco.mj_step(model, data)
            accum -= dt

        viewer.sync()
