import mujoco
import mujoco.viewer
import time
from utils import cmu_amc_converter

import numpy as np

import numpy as np

model = mujoco.MjModel.from_xml_path(r"PBPR_RL\environments\envs\assets\humanoid_CMU.xml")
data = mujoco.MjData(model)
expert = cmu_amc_converter.get_qpos( r"C:\Users\mayur\Documents\GitHub\PhysicsBasedPoseReconstruction\PBPR_RL\environments\envs\assets\cmu_mocap\walk_1.amc")
for i in range(expert.shape[0]):
    expert[i][2] -=0.09


        
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        for i in range(expert.shape[0]):
            data.qpos = expert[i]
            mujoco.mj_step(model, data)
            time.sleep(1.0/120)
            viewer.sync()