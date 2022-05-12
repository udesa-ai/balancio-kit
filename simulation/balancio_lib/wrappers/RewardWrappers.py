# ======================================================================
#  Balancio-Kit (c) 2021 Linar (UdeSA)
#  This code is licensed under MIT license (see LICENSE.txt for details)
# ======================================================================

"""
This module contains different gym wrappers to change the reward function.
"""

from gym import Wrapper
import numpy as np
from typing import Callable


class VelocityPitchRewardWrapper(Wrapper):
    """ Change the environment's reward to contemplate both the pitch and the linear velocity of the robots base.

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


class VelocityPitchYawRewardWrapper(Wrapper):
    """ Change the environment's reward to contemplate the pitch, the linear velocity and the yaw of the robots base.

    Example:
            env = VelocityPitchYawRewardWrapper(BalancioGymEnv(...))

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
        weight_yaw = 4 / 5

        # Penalize over pitch
        self.env.pitch_angle = self.env.robot.get_pitch()[0]
        # Penalize over velocity
        vel = self.env.robot.get_linear_vel_imu()[1]
        # Penalize over yaw
        yaw_rot = self.env.robot.get_angular_vel()[2]

        reward = - weight_pitch * np.square(self.env.pitch_angle) - weight_vel * abs(vel) - weight_yaw * np.square(yaw_rot)
        return reward


def get_reward_wrapper(wrapper_key: str) -> Callable:
    """ Get the class of the selected wrapper.

    @param wrapper_key: Reward wrapper type.
    @return reward_wrapper: Callable of the selected wrapper.
    """

    # Dictionary containing the different wrapper classes
    reward_wrappers = {"None": Wrapper,
                       "VelocityPitchRewardWrapper": VelocityPitchRewardWrapper,
                       "VelocityPitchYawRewardWrapper": VelocityPitchYawRewardWrapper}

    try:
        reward_wrapper = reward_wrappers[wrapper_key]
    except KeyError:
        raise ValueError(
            'Invalid Reward Wrapper. Valid reward wrappers: {}'.format(", ".join(reward_wrappers)))

    return reward_wrapper
