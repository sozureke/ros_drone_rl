from stable_baselines3 import PPO
from drone_rl_agent.env import DroneLandingEnv
from drone_rl_agent.wrappers import NormalizeObs, ClipReward


def main() -> None:

    env = DroneLandingEnv()
    env = NormalizeObs(env, low=[-10]*6, high=[10]*6)
    env = ClipReward(env, -10, 0)

    model = PPO('MlpPolicy', env, verbose=1, tensorboard_log='tb')
    model.learn(total_timesteps=300_000)
    model.save('ppo_landing')


if __name__ == '__main__':
    main()
