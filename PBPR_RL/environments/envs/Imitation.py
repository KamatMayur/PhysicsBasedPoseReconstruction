__credits__ = ["Kallinteris-Andreas"]

from typing import Dict, Tuple, Union

import numpy as np

from gymnasium import utils
from gymnasium.envs.mujoco import MujocoEnv
from gymnasium.spaces import Box
from utils.cmu_amc_converter import get_qpos


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
        xml_file: str = r"examples\envs\assets\humanoid_CMU.xml",
        amc_file: str = r"PBPR_RL\environments\envs\assets\cmu_mocap\walk_1.amc",
        dt: float = 1.0/120.0,
        frame_skip: int = 1,
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
            default_camera_config,
            reset_noise_scale,
            **kwargs,
        )

        self._reset_noise_scale = reset_noise_scale

        MujocoEnv.__init__(
            self,
            xml_file,
            amc_file,
            dt,
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

        self.expert_poses = get_qpos(amc_file)

        self.model.opt.timestep = self.dt

    def _get_obs(self):
        position = self.data.qpos.flatten()
        velocity = self.data.qvel.flatten()
        exp_position = self.expert_poses[self.frame_num]


        return np.concatenate(
            (
                position,
                velocity,
                exp_position,
            )
        )

    def step(self, action):
        self.do_simulation(action, self.frame_skip)
        self.frame_num += 1

        reward, reward_info = self._get_rew(action)
        info = {
            "x_position": self.data.qpos[0],
            "y_position": self.data.qpos[1],
            "z_distance_from_origin": self.data.qpos[2] - self.init_qpos[2],
            **reward_info,
        }
        
        if self.render_mode == "human":
            self.render()

        if self.frame_num == self.expert_poses.shape[0]:
            terminated = True
        # truncation=False as the time limit is handled by the `TimeLimit` wrapper added during `make`
        return self._get_obs(), reward, terminated, False, info

    def _get_rew(self, action):
        
        similarity = np.linalg.norm(np.array(self.data.qpos) - np.array(self.expert_poses[self.frame_num-1]))
        similarity_rew = (1 / (1 + similarity))/self.model.opt.timestep
        quad_ctrl_cost = self._ctrl_cost_weight * np.square(self.data.ctrl).sum()

        quad_impact_cost = (
            self._impact_cost_weight * np.square(self.data.cfrc_ext).sum()
        )
        min_impact_cost, max_impact_cost = self._impact_cost_range
        quad_impact_cost = np.clip(quad_impact_cost, min_impact_cost, max_impact_cost)

        reward = similarity_rew - quad_ctrl_cost - quad_impact_cost + 1

        reward_info = {
            "reward_similarity": similarity_rew,
            "reward_quadctrl": -quad_ctrl_cost,
            "reward_impact": -quad_impact_cost,
        }

        return reward, reward_info

    def reset_model(self):
        self.frame_num = 0
        noise_low = -self._reset_noise_scale
        noise_high = self._reset_noise_scale
        qpos = self.init_qpos + self.np_random.uniform(
            low=noise_low, high=noise_high, size=self.model.nq
        )
        qvel = self.init_qvel + self.np_random.uniform(
            low=noise_low, high=noise_high, size=self.model.nv
        )
        self.set_state(qpos, qvel)

        observation = self._get_obs()
        return observation

    def _get_reset_info(self):
        return {
            "x_position": self.data.qpos[0],
            "y_position": self.data.qpos[1],
            "z_distance_from_origin": self.data.qpos[2] - self.init_qpos[2]
        }