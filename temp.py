from PBPR_RL.utils.cmu_amc_converter import get_qpos
import mujoco
from mujoco import viewer
import time

model = mujoco.MjModel.from_xml_path(r"PBPR_RL\environments\envs\assets\humanoid_CMU.xml")
data = mujoco.MjData(model)


print(model.opt.timestep)

with viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        # for pos in get_qpos(r"PBPR_RL\environments\envs\assets\cmu_mocap\walk_1.amc"):
        mujoco.mj_step(model, data)
            # data.qpos = pos
            # time.sleep(0.00833)
        viewer.sync()    



# hmm = get_qpos(r"PBPR_RL\environments\envs\assets\cmu_mocap\walk_1.amc")
# print(hmm[409])