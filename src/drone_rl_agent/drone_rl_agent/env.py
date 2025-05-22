import time
import gym
import numpy as np
import rclpy
from rclpy.node import Node
from gym import spaces

from drone_interfaces.msg import DroneState, PlatformState
from drone_control.command_gateway import CommandGateway


class DroneLandingEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, rate_hz: float = 20.0):
        super().__init__()
        self.rate_hz = rate_hz

        rclpy.init(args=None)
        self.node = Node('rl_env')
        self.gateway = CommandGateway()

        self.drone_state: DroneState | None = None
        self.platform_state: PlatformState | None = None

        self.node.create_subscription(
            DroneState, '/drone/state',
            lambda m: setattr(self, 'drone_state', m), 10)
        self.node.create_subscription(
            PlatformState, '/platform/state',
            lambda m: setattr(self, 'platform_state', m), 10)

        high = np.array([10, 10, 10, 5, 5, 5], dtype=np.float32)
        self.observation_space = spaces.Box(-high, high, dtype=np.float32)
        self.action_space = spaces.Box(-1.0, 1.0, shape=(3,), dtype=np.float32)

    def _step_sim(self, dt):
        start = self.node.get_clock().now().nanoseconds
        while (self.node.get_clock().now().nanoseconds - start) * 1e-9 < dt:
            rclpy.spin_once(self.node, timeout_sec=0.002)

    def _get_obs(self):
        ds, ps = self.drone_state, self.platform_state
        return np.array([
            ds.x - ps.x,
            ds.y - ps.y,
            ds.z - ps.z,
            ds.vx, ds.vy, ds.vz
        ], dtype=np.float32)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.gateway.disarm()
        time.sleep(0.2)
        self.gateway.set_offboard_mode()
        self.gateway.arm()
        self.gateway.publish_position(0.0, 0.0, 2.0)

        while rclpy.ok() and (self.drone_state is None or self.platform_state is None):
            rclpy.spin_once(self.node, timeout_sec=0.05)

        return self._get_obs(), {}


    def step(self, action):
        vx, vy, vz = (action * 1.0).tolist()
        self.gateway.publish_velocity(vx, vy, vz)
        self._step_sim(1.0 / self.rate_hz)

        obs = self._get_obs()
        dx, dy, dz = obs[:3]
        reward = - (abs(dx) + abs(dy) + abs(dz))
        done = abs(dx) < 0.05 and abs(dy) < 0.05 and abs(dz) < 0.05
        info = {}
        return obs, reward, done, False, info

    def close(self):
        rclpy.shutdown()


def run_inference_node():
    from stable_baselines3 import PPO
    from drone_rl_agent.wrappers import NormalizeObs

    env = DroneLandingEnv()
    env = NormalizeObs(env, low=[-10]*6, high=[10]*6)

    model = PPO.load('ppo_landing.zip')
    obs, _ = env.reset()

    while rclpy.ok():
        action, _ = model.predict(obs, deterministic=True)
        obs, *_ = env.step(action)
