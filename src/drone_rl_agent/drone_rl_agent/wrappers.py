import numpy as np
import gym
from gym import Wrapper


class NormalizeObs(Wrapper):
    def __init__(self, env, low, high):
        super().__init__(env)
        self.low = np.asarray(low, dtype=np.float32)
        self.high = np.asarray(high, dtype=np.float32)

    def reset(self, **kwargs):
        obs, info = self.env.reset(**kwargs)
        return self._normalize(obs), info

    def step(self, action):
        obs, reward, done, truncated, info = self.env.step(action)
        return self._normalize(obs), reward, done, truncated, info

    def _normalize(self, observation):
        return 2.0 * (observation - self.low) / (self.high - self.low) - 1.0


class ClipReward(Wrapper):
    def __init__(self, env, min_r=-10.0, max_r=10.0):
        super().__init__(env)
        self.min_r = min_r
        self.max_r = max_r

    def step(self, action):
        obs, reward, done, truncated, info = self.env.step(action)
        clipped = float(np.clip(reward, self.min_r, self.max_r))
        return obs, clipped, done, truncated, info
