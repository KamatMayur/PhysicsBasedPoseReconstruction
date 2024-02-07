from gymnasium.envs.registration import register

register(
     id="Imitation-v0",
     entry_point="environments.envs:ImitationEnv",
     max_episode_steps=1000,
)