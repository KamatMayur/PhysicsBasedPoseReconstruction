from PBPR_RL.utils.cmu_amc_converter import get_qpos, qvel_from_qpos, xpos_from_qpos, compute_com
import mujoco
import mujoco.viewer
import time


import numpy as np

import numpy as np

model = mujoco.MjModel.from_xml_path(r"PBPR_RL\environments\envs\assets\humanoid_CMU.xml")
data = mujoco.MjData(model)

poses = get_qpos(r"C:\Users\mayur\Documents\GitHub\PhysicsBasedPoseReconstruction\PBPR_RL\environments\envs\assets\cmu_mocap\walk_1.amc")[0::4]
qvels = qvel_from_qpos(poses, 1.0/120)
xpos = xpos_from_qpos(poses)
com = compute_com(xpos)
xpos = xpos[:, 1:]

actual_xpos = []
with mujoco.viewer.launch_passive(model, data) as viewer:
    
    while viewer.is_running():
        for pose in poses:
            data.qpos = pose
            mujoco.mj_step(model, data)
            time.sleep(1.0/30)
            viewer.sync()
        break


