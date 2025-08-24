import time
import numpy as np
import mujoco
from mujoco.viewer import launch_passive

XML_PATH = "/home/debojit/ME639/Tutorial1/description/simple_scene.xml"

model = mujoco.MjModel.from_xml_path(XML_PATH)
# (Optional) slow down or speed up the sim by changing the timestep:
# model.opt.timestep = 0.005  # 200 Hz
data  = mujoco.MjData(model)

# Set initial freejoint pose (qpos[0:3]=xyz, qpos[3:7]=wxyz)
data.qpos[0:3] = np.array([0.0, 0.0, 2.5])
data.qpos[3:7] = np.array([1.0, 0.0, 0.0, 0.0])
mujoco.mj_forward(model, data)

# Force window and vector (bump magnitude if friction is high)
FORCE_START, FORCE_END = 3.0, 4.0      # seconds
FORCE = np.array([-6.0, 0.0, 0.0])    # N, world frame
TORQUE = np.array([0.0, 0.0, 0.0])     # Nm, world frame

# Body index for the cuboid (resolve via site)
site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "force_site")
body_id = model.site_bodyid[site_id]

dt = model.opt.timestep

with launch_passive(model, data) as viewer:
    # Render one initial frame before stepping so you see the starting pose
    viewer.sync()
    print(f"Start: t={data.time:.3f}s (no steps yet)")

    # Real-time pacing
    prev = time.perf_counter()
    accum = 0.0

    while viewer.is_running():
        now = time.perf_counter()
        accum += now - prev
        prev = now

        # Step at (approximately) real-time rate
        while accum >= dt:
            # clear external wrenches each step
            data.xfrc_applied[:] = 0.0

            t = data.time
            if FORCE_START <= t < FORCE_END:
                data.xfrc_applied[body_id, 0:3] = FORCE
                data.xfrc_applied[body_id, 3:6] = TORQUE

            mujoco.mj_step(model, data)
            accum -= dt

        viewer.sync()
