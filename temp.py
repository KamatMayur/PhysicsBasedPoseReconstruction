
import mujoco
import mujoco.viewer
import time
from PBPR_RL.utils import cmu_amc_converter

import numpy as np

import numpy as np

model = mujoco.MjModel.from_xml_path(r"PBPR_RL\environments\envs\assets\humanoid_CMU.xml")
data = mujoco.MjData(model)
expert = cmu_amc_converter.get_qpos( r"C:\Users\mayur\Documents\GitHub\PhysicsBasedPoseReconstruction\PBPR_RL\environments\envs\assets\cmu_mocap\walk_1.amc")

with mujoco.viewer.launch_passive(model, data) as viewer:
    
    while viewer.is_running():
        data.xfrc_applied[mujoco.mj_name2id(model, 1, 'root')][:3] = [0, 0, 501.0]
        mujoco.mj_step(model, data)
        #time.sleep(1.0/10)
        viewer.sync()
        
with mujoco.viewer.launch_passive(model, data) as viewer:
    
    while viewer.is_running():
        for i in range(expert.shape[0]):
            data.qpos = expert[i]
            mujoco.mj_step(model, data)
            time.sleep(1.0/120)
            viewer.sync()
        