# ======================================================================
#  Balancio-Kit (c) 2021 Linar (UdeSA)
#  This code is licensed under MIT license (see LICENSE.txt for details)
# ======================================================================

"""
This module contains different gym wrappers to change the reward function.
"""

from gym import Wrapper
import numpy as np


class VelocityPitchRewardWrapper(Wrapper):
    """ Change the environment reward to contemplate both the pitch and the linear velocity of the robots base.

    Example:
            env = VelocityPitchRewardWrapper(BalancioGymEnv(...))

    @param env: Environment
    """

    def __init__(self, env):
        super().__init__(env)

    def step(self, action):
        observation, reward, done, info = self.env.step(action)
        return observation, self.reward(), done, info

    def reward(self):
        # Weights
        weight_pitch = 1
        weight_vel = 4 / 50

        # Penalize over pitch
        self.env.pitch_angle = self.env.robot.get_pitch()[0]
        # Penalize over velocity
        vel = self.env.robot.get_linear_vel_imu()[1]

        reward = - weight_pitch * np.square(self.env.pitch_angle) - weight_vel * abs(vel)
        return reward
