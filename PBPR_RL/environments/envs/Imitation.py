__credits__ = ["Kallinteris-Andreas"]

from typing import Dict, Tuple, Union

import numpy as np

from gymnasium import utils
from gymnasium.envs.mujoco import MujocoEnv
from gymnasium.spaces import Box
from utils.cmu_amc_converter import get_qpos, qvel_from_qpos, xpos_from_qpos, compute_com


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
        dt: float = 1.0/30.0,
        frame_skip: int = 5,
        pose_weight: float = 0.3,
        velocit_weight: float = 0.1,
        end_eff_weight: float = 0.5,
        com_weight: float = 0.1,
        default_camera_config: Dict[str, Union[float, int]] = DEFAULT_CAMERA_CONFIG,
        reset_noise_scale: float = 0.5e-2,
        **kwargs,
    ):
        utils.EzPickle.__init__(
            self,
            xml_file,
            amc_file,
            dt,
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

        self.metadata = {
            "render_modes": [
                "human",
                "rgb_array",
                "depth_array",
            ],
            "render_fps": int(np.round(1.0 / self.dt)),
        }

        obs_size = 2*self.data.qpos.size + self.data.qvel.size

        self.observation_space = Box(
            low=-np.inf, high=np.inf, shape=(obs_size,), dtype=np.float64
        )

        self.observation_structure = {
            "qpos": self.data.qpos.size,
            "qvel": self.data.qvel.size,
            "exp_qpos": self.data.qpos.size,
        }

        self._pose_weight = pose_weight
        self._velocit_weight = velocit_weight
        self._end_eff_weight = end_eff_weight
        self._com_weight = com_weight

        self.expert_qpos = get_qpos(amc_file)[4-1::4]
        self.expert_qvel = qvel_from_qpos(self.expert_qpos, self.dt)
        self.expert_xpos = xpos_from_qpos(self.expert_qpos)
        self.exper_com = compute_com(self.expert_xpos)
        self.expert_xpos = self.expert_xpos[:, 1:]              # removed the com of the ground plane

        self.model.opt.timestep = self.dt

    def _get_obs(self):
        position = self.data.qpos.flatten()
        velocity = self.data.qvel.flatten()
        if self.frame_num == self.expert_qpos.shape[0]:
            exp_position = self.expert_qpos[self.frame_num-1]
        else:
           exp_position = self.expert_qpos[self.frame_num] 


        return np.concatenate(
            (
                position,
                velocity,
                exp_position,
            )
        )

    def step(self, action):
        self.do_simulation(action, self.frame_skip)

        reward, reward_info = self._get_rew(action)
        info = {
            "x_position": self.data.qpos[0],
            "y_position": self.data.qpos[1],
            "z_distance_from_origin": self.data.qpos[2] - self.init_qpos[2],
            "expert_z": self.expert_qpos[self.frame_num-1][2],
            "agent_z": self.data.qpos[2],
            **reward_info,
        }
        
        if self.render_mode == "human":
            self.render()

        end_of_clip = True if self.frame_num+1 == self.expert_qpos.shape[0] else False
        model_fallen =  True if (self.expert_qpos[self.frame_num-1][2]) - self.data.qpos[2] >= 0.2 else False

        terminated = False
        if end_of_clip or model_fallen:
            terminated = True

        self.frame_num += 1
        # truncation=False as the time limit is handled by the `TimeLimit` wrapper added during `make`
        return self._get_obs(), reward, terminated, False, info

    def _get_rew(self, action):
        pose_reward = np.exp(-2*(np.linalg.norm(np.array(self.data.qpos) - np.array(self.expert_qpos[self.frame_num]))))

        velocity_reward = np.exp(-0.005*(np.linalg.norm(np.array(self.data.qvel) - np.array(self.expert_qvel[self.frame_num]))))

        end_eff_id = [5, 10, 16, 22, 29]
        end_eff_reward = np.exp(-5*(np.linalg.norm(np.array(self.data.xpos[end_eff_id, :]) - np.array(self.expert_xpos[self.frame_num][end_eff_id, :]))))

        body_positions = self.data.xpos[1:] # exclude the ground plane
    
        # Get the body masses from the model (assuming it's available globally)
        body_masses = self.model.body_mass[1:]  # Assuming the ground plane is the first body
    
        # Compute the center of mass using the weighted average of body positions
        com = np.sum(body_positions * body_masses[:, np.newaxis], axis=0) / np.sum(body_masses)
    
        com_reward = np.exp(-100*(np.linalg.norm(np.array(com) - np.array(self.exper_com[self.frame_num]))))

        reward = self._pose_weight*pose_reward + self._velocit_weight*velocity_reward + self._end_eff_weight*end_eff_reward + self._com_weight*com_reward

        reward_info = {
            "pose_reward": pose_reward,
            "velocity_reward": velocity_reward,
            "end_eff_reward": end_eff_reward,
            "com_reward": com_reward,
            "frame_num" : self.frame_num
        }

        return reward, reward_info

    def reset_model(self):
        self.frame_num = 0
        noise_low = -self._reset_noise_scale
        noise_high = self._reset_noise_scale
        self.init_qpos = self.expert_qpos[self.frame_num]
        self.init_qvel = self.expert_qvel[self.frame_num]

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