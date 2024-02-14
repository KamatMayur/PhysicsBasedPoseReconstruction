__credits__ = ["Kallinteris-Andreas"]

from typing import Dict, Tuple, Union

import numpy as np
import mujoco
from gymnasium import utils
from gymnasium.envs.mujoco import MujocoEnv
from gymnasium.spaces import Box, Tuple
from utils.cmu_amc_converter import get_qpos
from utils.get_expert import Expert
from utils.maths import multi_quat_diff, multi_quat_diff, multi_quat_norm, rotation_from_quaternion

DEFAULT_CAMERA_CONFIG = {
    "trackbodyid": 1,
    "distance": 4.0,
    "lookat": np.array((0.0, 0.0, 0.8925)),
    "elevation": -20.0,
}


class ImitationEnv(MujocoEnv, utils.EzPickle):

    metadata = {
        "render_modes": [
            "human",
            "rgb_array",
            "depth_array",
        ],
    }

    def __init__(
        self,
        xml_file: str = r"C:\Users\mayur\Documents\GitHub\PhysicsBasedPoseReconstruction\PBPR_RL\environments\envs\assets\humanoid_CMU.xml",
        amc_file: str = r"C:\Users\mayur\Documents\GitHub\PhysicsBasedPoseReconstruction\PBPR_RL\environments\envs\assets\cmu_mocap\walk_1.amc",
        frame_skip: int = 5,
        pose_weight: float = 0.6,
        velocit_weight: float = 0.1,
        end_eff_weight: float = 0.2,
        com_weight: float = 0.1,
        default_camera_config: Dict[str, Union[float, int]] = DEFAULT_CAMERA_CONFIG,
        reset_noise_scale: float = 0.5e-2,
        **kwargs,
    ):
        utils.EzPickle.__init__(
            self,
            xml_file,
            amc_file,
            frame_skip,
            pose_weight,
            velocit_weight,
            end_eff_weight,
            com_weight,
            default_camera_config,
            reset_noise_scale,
            **kwargs,
        )

        self._reset_noise_scale = reset_noise_scale

        MujocoEnv.__init__(
            self,
            xml_file,
            frame_skip,
            observation_space=None,
            default_camera_config=default_camera_config,
            **kwargs,
        )

        self.model.opt.timestep = 1.0/120

        self.metadata = {
            "render_modes": [
                "human",
                "rgb_array",
                "depth_array",
            ],
            "render_fps": int(np.round(1.0 / self.dt)),
        }

        obs_size = self.data.qpos.size + self.data.qvel.size
        self.observation_space = Box(low=-np.inf, high=np.inf, shape=(obs_size,), dtype=np.float64)

        self.observation_structure = {
            "qpos": self.data.qpos.size,
            "qvel": self.data.qvel.size,
        }

        act_action_space = Box(low=-1, high=1, shape=(self.model.nu,), dtype=np.float32)
        xfrc_action_space = Box(low=-400, high=400, shape=(3,), dtype=np.float32)
        self.action_space = Box(low=np.concatenate([act_action_space.low, xfrc_action_space.low]),
                                high=np.concatenate([act_action_space.high, xfrc_action_space.high]),
                                dtype=np.float32)
        self.action_structure = {
            "actuation": self.model.nu,
            "external_force": 3,
        }

        e_qpos = get_qpos(amc_file=amc_file)
        for i in range(e_qpos.shape[0]):
            e_qpos[i][2] -=0.09
        self.expert = Expert(e_qpos, self.model, self.data)  

        self._pose_weight = pose_weight
        self._velocit_weight = velocit_weight
        self._end_eff_weight = end_eff_weight
        self._com_weight = com_weight

    def _get_obs(self):
        position = self.data.qpos.flatten()
        velocity = self.data.qvel.flatten()

        return np.concatenate(
            (
                position,
                velocity,
            )
        )
    
    def do_simulation(self, action, n_frames) -> None:

        ctrl = action[:self.model.nu]
        xfrc = action[self.model.nu:]
        self.data.xfrc_applied[mujoco.mj_name2id(self.model, 1, 'root')][:3] = xfrc
        return super().do_simulation(ctrl, n_frames)
    
    def step(self, action):
        self.prev_bquat = self.data.xquat[4:].copy()

        self.do_simulation(action, self.frame_skip)

        reward, reward_info = self._get_rew(action)
        info = {
            "x_position": self.data.qpos[0],
            "y_position": self.data.qpos[1],
            "z_distance_from_origin": self.data.qpos[2] - self.init_qpos[2],
            **reward_info,
        }
        
        if self.render_mode == "human":
            self.render()

        end_of_clip = True if self.frame_num+1 == self.expert.e_qpos.shape[0] else False
        root_dropped =  True if (self.expert.get_qpos(self.frame_num)[2]) - self.data.qpos[2] >= 0.1 else False
        head_id = mujoco.mj_name2id(self.model, 1, "head")
        cur_headpos = self.data.xpos[head_id][2].copy()
        head_dropped = True if (self.expert.get_ee_xpos(self.frame_num)[2][2]) - cur_headpos >= 0.1 else False

        terminated = False
        if end_of_clip or root_dropped or head_dropped:
            terminated = True

        self.frame_num += 1
        # truncation=False as the time limit is handled by the `TimeLimit` wrapper added during `make`
        return self._get_obs(), reward, terminated, False, info
    
    def get_angvel_fd(self, prev_bquat, cur_bquat):
        dt = self.model.opt.timestep
        q_diff = multi_quat_diff(cur_bquat, prev_bquat)
        body_angvel = rotation_from_quaternion(q_diff)/dt
        return body_angvel


    def _get_rew(self, action):
        k_p = 2.0
        k_v = 0.005
        k_e = 20
        k_c = 1000

        cur_bquat = self.data.xquat[4:]
        e_bquat = self.expert.get_xquat(self.frame_num)
        pose_diff = multi_quat_norm(multi_quat_diff(cur_bquat, e_bquat))
        pose_dist = np.linalg.norm(pose_diff)
        pose_reward = np.exp(-k_p * (pose_dist ** 2))

        cur_bangvel = self.get_angvel_fd(self.prev_bquat, cur_bquat)
        e_bangvel = self.get_angvel_fd(self.expert.get_xquat(min(self.frame_num-1, 0)), self.expert.get_xquat(self.frame_num))
        velocity_dist = np.linalg.norm(cur_bangvel - e_bangvel, ord=2)
        velocity_reward = np.exp(-k_v * (velocity_dist ** 2))

        cur_ee = []
        for ee in  ["lfoot", "rfoot", "head", "lhand", "rhand"]:
            ee_id = mujoco.mj_name2id(self.model, 1, ee)
            cur_ee.append(self.data.xpos[ee_id].copy())
        end_eff_dist = np.linalg.norm(cur_ee - self.expert.get_ee_xpos(self.frame_num))
        end_eff_reward = np.exp(-k_e * (end_eff_dist ** 2))

        com_dist = np.linalg.norm(self.data.subtree_com[1].copy() - self.expert.get_com(self.frame_num))
        com_reward = np.exp(-k_c * (com_dist ** 2))

        reward = self._pose_weight*pose_reward + self._velocit_weight*velocity_reward + self._end_eff_weight*end_eff_reward + self._com_weight*com_reward

        reward_info = {
            "pose_reward": pose_reward,
            "velocity_reward": velocity_reward,
            "end_eff_reward": end_eff_reward,
            "com_reward": com_reward,
            "frame_num" : self.frame_num,
            "sim_time": self.data.time
        }

        return reward, reward_info

    def reset_model(self):
        self.frame_num = 0
        self.init_qpos = self.expert.get_qpos(self.frame_num) 
        self.init_qvel = self.expert.get_qvel(self.frame_num)

        qpos = self.init_qpos
        qvel = self.init_qvel 
        self.set_state(qpos, qvel)

        observation = self._get_obs()
        return observation

    def _get_reset_info(self):
        return {
            "x_position": self.data.qpos[0],
            "y_position": self.data.qpos[1],
            "z_distance_from_origin": self.data.qpos[2] - self.init_qpos[2]
        }