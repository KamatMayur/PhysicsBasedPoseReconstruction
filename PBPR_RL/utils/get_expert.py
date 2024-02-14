import mujoco
import numpy as np

class Expert:

    def __init__(self, e_qpos, model, data, end_eff=True):
        self.data = data
        self.model = model
        self.e_qvel = []
        self.e_qpos = []
        self.e_com = []
        self.e_xpos = []
        self.e_xquat = []
        self.e_eexpos = []
        self.num_frames = e_qpos.shape[0]
        for i in range(self.num_frames):
            self.data.qpos = e_qpos[i]
            mujoco.mj_step(self.model, self.data)
            
            ee_names = ["lfoot", "rfoot", "head", "lhand", "rhand"]
            eexpos = []
            for ee in ee_names:
                ee_id = mujoco.mj_name2id(self.model, 1, ee)
                eexpos.append(self.data.xpos[ee_id].copy())
            
            self.e_xpos.append(self.data.xpos[3:].copy())
            self.e_com.append(self.data.subtree_com[1].copy())
            self.e_qvel.append(self.data.qvel.copy())
            self.e_qpos.append(self.data.qpos.copy())
            self.e_eexpos.append(eexpos)
            self.e_xquat.append(self.data.xquat[4:].copy())
         
        self.e_xpos = np.array(self.e_xpos)
        self.e_com = np.array(self.e_com )
        self.e_qvel = np.array(self.e_qvel)
        self.e_qpos = np.array(self.e_qpos)
        self.e_eexpos = np.array(self.e_eexpos)



    def get_ee_xpos(self, frame):
        return self.e_eexpos[frame]
    
    def get_xquat(self, frame):
        return self.e_xquat[frame]

    def get_xpos(self, frame):
        return self.e_xpos[frame]
    
    def get_qpos(self, frame):
        return self.e_qpos[frame]

    def get_qvel(self, frame):
        return self.e_qvel[frame]

    def get_com(self, frame):
        return self.e_com[frame]

        
# from cmu_amc_converter import get_qpos
# m = mujoco.MjModel.from_xml_path(r'C:\Users\mayur\Documents\GitHub\PhysicsBasedPoseReconstruction\PBPR_RL\environments\envs\assets\humanoid_CMU.xml')
# d = mujoco.MjData(m)
# expert = Expert(e_qpos=get_qpos(r"C:\Users\mayur\Documents\GitHub\PhysicsBasedPoseReconstruction\PBPR_RL\environments\envs\assets\cmu_mocap\walk_1.amc"),
#                 model=m,
#                 data=d)



# print(expert.get_xquat(3).shape)