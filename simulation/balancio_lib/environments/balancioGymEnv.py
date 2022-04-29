# ======================================================================
#  Balancio-Kit (c) 2021 Linar (UdeSA)
#  This code is licensed under MIT license (see LICENSE.txt for details)
# ======================================================================

"""
This module implements an OpenAI Gym environment of the Balancio-Kit.
"""

import os
import inspect
import gym
import time
from gym import spaces
from gym.utils import seeding
import numpy as np
import pybullet
from ..robot import balancio
from pybullet_utils import bullet_client
import pybullet_data
from pkg_resources import parse_version
from collections import deque
import matplotlib.pyplot as plt
from typing import Union, Tuple

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

RENDER_HEIGHT = 720
RENDER_WIDTH = 960
EPISODE_LENGTH = 5000

# For data plotting (used for debugging)
PLOT_DATA = True


class BalancioGymEnv(gym.Env):
    """ Class that encapsulates the Balancio-Kit environment.
    """
    metadata = {'render.modes': ['human', 'rgb_array'], 'video.frames_per_second': 50}

    def __init__(self,
                 urdf_root: str = pybullet_data.getDataPath(),
                 action_repeat: int = 1,
                 is_discrete: bool = False,
                 renders: bool = False,
                 normalize: bool = True,
                 backlash: bool = True,
                 real_imu: bool = False,
                 seed: Union[str, int] = None,
                 algo_mode: str = 'RL',  # 'RL' or 'PID'
                 memory_buffer: int = 1,
                 only_pitch: bool = True,
                 policy_feedback: bool = False
                 ):
        """Class constructor.

        @param urdf_root: Pybullet installation path.
        @param action_repeat: Number of simulation steps inside a control step.
        @param is_discrete: If the action space is discrete.
        @param renders: Render simulation or not.
        @param normalize: Normalize or not actions and observations.
        @param backlash: Whether to apply backlash to the motors or not.
        @param real_imu: If the pitch is calculated directly (False) or via a complementary filter of a simulated IMU (True).
        @param seed: Gym and Numpy random seed.
        @param algo_mode: Control algorithm running on top of the environment. (RL, PID, etc).
        @param memory_buffer: Number of time-steps stored and returned for each observation component.
        @param only_pitch: If the observation contains only the robot's pitch.
        @param policy_feedback: If the observation contains the motor commands of the previous time-step.
        """
        self._time_step = 1 / 240  # 0.01
        self._urdf_root = urdf_root
        self._action_repeat = action_repeat
        self._observation = []
        self._env_step_counter = 0
        self._renders = renders
        self._normalize = normalize
        self._is_discrete = is_discrete
        if self._renders:
            self._p = bullet_client.BulletClient(connection_mode=pybullet.GUI)
        else:
            self._p = bullet_client.BulletClient()

        self._backlash = backlash
        self._real_imu = real_imu
        self.filter_tau = 0.98
        self.pitch_ri = 0
        self.seed(seed)
        self._algo_mode = algo_mode
        self._memory_buffer = memory_buffer
        self._only_pitch = only_pitch
        self._policy_feedback = policy_feedback

        self.pitch_buffer = deque(np.zeros(self._memory_buffer), self._memory_buffer)
        self.ax_buffer = deque(np.zeros(self._memory_buffer), self._memory_buffer)
        self.ay_buffer = deque(np.zeros(self._memory_buffer), self._memory_buffer)
        self.az_buffer = deque(np.zeros(self._memory_buffer), self._memory_buffer)
        self.gx_buffer = deque(np.zeros(self._memory_buffer), self._memory_buffer)
        self.gy_buffer = deque(np.zeros(self._memory_buffer), self._memory_buffer)
        self.gz_buffer = deque(np.zeros(self._memory_buffer), self._memory_buffer)
        self.pwmL_buffer = deque(np.zeros(self._memory_buffer), self._memory_buffer)
        self.pwmR_buffer = deque(np.zeros(self._memory_buffer), self._memory_buffer)

        # self.reset()
        if self._only_pitch:
            if self._policy_feedback:
                observation_dim = 3 * self._memory_buffer
            else:
                observation_dim = 1 * self._memory_buffer
        else:  # pitch, ax, ay, az, gx, gy, gz
            if self._policy_feedback:
                observation_dim = 9 * self._memory_buffer
            else:
                observation_dim = 7 * self._memory_buffer

        self.observation_dim = observation_dim
        print("Observation dimension: {}".format(observation_dim))

        # Normalization parameters
        # Observation vector: [pitch, ax, ay, az, gx, gy, gz, pwmL, pwmR]  -> Varies depending on observation_dim
        # HARD-CODED MEAN AND ST-DEV
        self.norm_mean = np.array([0, 0, 0, 1.05682576e+00, 0, 0, 0, 0, 0])
        # self.norm_std = np.array([0.06519447, 3.0353326, 1.81746345, 5.0081295, 0.09617138, 0.03410039, 0.26264516, 0.32308017, 0.32999594])
        self.norm_std = np.array(
            [1.5, 3.0353326, 1.81746345, 5.0081295, 0.09617138, 0.03410039, 0.26264516, 0.32308017, 0.32999594])
        if self._only_pitch and self._policy_feedback:
            self.norm_mean = np.array([self.norm_mean[0], self.norm_mean[-2], self.norm_mean[-1]])
            self.norm_std = np.array([self.norm_std[0], self.norm_std[-2], self.norm_std[-1]])
        else:
            self.norm_mean = self.norm_mean[:int(observation_dim / self._memory_buffer)]
            self.norm_std = self.norm_std[:int(observation_dim / self._memory_buffer)]

        # self.norm_mean = np.zeros(int(observation_dim/self._memory_buffer))
        # self.norm_std = np.ones(int(observation_dim/self._memory_buffer))
        self.norm_ctr = 0
        self.norm_var = np.ones(int(observation_dim / self._memory_buffer))
        self.mean_diff = np.zeros(int(observation_dim / self._memory_buffer))

        max_act = 1
        self.act_norm = max_act * np.array([1, 1])

        observation_high = np.ones(observation_dim)
        # if not self._normalize:
        #     observation_high = self.obs_norm

        if is_discrete:
            self.action_space = spaces.Discrete(9)
        else:
            action_dim = 2
            if self._normalize:
                self._action_bound = 1
            else:
                self._action_bound = max_act
            action_high = np.array([self._action_bound] * action_dim)
            self.action_space = spaces.Box(-action_high, action_high, dtype=np.float32)
        self.observation_space = spaces.Box(-observation_high, observation_high, dtype=np.float32)
        self.viewer = None

        self.pitch_angle = 0

        self.last_frame_time = time.time()

        # Data plotting
        if PLOT_DATA and self._real_imu:
            self.fig, self.ax = plt.subplots(1, 1)
            self.ax.set_xlim(0, 100)
            self.ax.set_ylim(-2, 2)
            self.fig_background = self.fig.canvas.copy_from_bbox(self.ax.bbox)
            self.fig_points1 = self.ax.plot([0], [0], '--', animated=True)[0]
            self.fig_points1.set_color('gray')
            self.fig_points1.set_label('Real Pitch')
            self.fig_points2 = self.ax.plot([0], [0], '-', animated=True)[0]
            self.fig_points2.set_label('Complementary Filter Pitch')
            self.ax.legend()
            plt.show(block=False)
            plt.pause(0.01)
            self.real_pitch_buffer = deque(maxlen=100)
            self.filter_pitch_buffer = deque(maxlen=100)

    def reset(self) -> list:
        """Reset environment.
        @return _observation: Initial robot's observation, after resetting the environment.
        """
        self._p.resetSimulation()
        # p.setPhysicsEngineParameter(numSolverIterations=300)
        self._p.setTimeStep(self._time_step)
        # self._p.setPhysicsEngineParameter(numSolverIterations=300, numSubSteps=200)
        self._p.loadURDF(os.path.join(self._urdf_root, "plane.urdf"))
        # stadiumobjects = self._p.loadSDF(os.path.join(self._urdfRoot, "stadium.sdf"))

        self._p.setGravity(0, 0, -0.981)  # Gravity: 0.981 dm/(10.s)^2
        self._robot = balancio.Balancio(self._p, time_step=self._time_step,
                                        backlash=self._backlash)
        self._env_step_counter = 0

        self.pitch_ri = self._robot.orientation_init_pitch
        self._robot.linear_accel_reset()

        self.pitch_buffer = deque(np.zeros(self._memory_buffer), self._memory_buffer)
        self.ax_buffer = deque(np.zeros(self._memory_buffer), self._memory_buffer)
        self.ay_buffer = deque(np.zeros(self._memory_buffer), self._memory_buffer)
        self.az_buffer = deque(np.zeros(self._memory_buffer), self._memory_buffer)
        self.gx_buffer = deque(np.zeros(self._memory_buffer), self._memory_buffer)
        self.gy_buffer = deque(np.zeros(self._memory_buffer), self._memory_buffer)
        self.gz_buffer = deque(np.zeros(self._memory_buffer), self._memory_buffer)
        self.pwmL_buffer = deque(np.zeros(self._memory_buffer), self._memory_buffer)
        self.pwmR_buffer = deque(np.zeros(self._memory_buffer), self._memory_buffer)

        # if self._normalize:
        #     self.normalizer_reset()

        for i in range(5):
            self._p.stepSimulation()
            self._robot.linear_accel_update()
        self._observation = self.get_observation_UPDATE()

        return self._observation

    def __del__(self):
        """Destructor"""
        self._p = 0

    def seed(self, seed_init: Union[int, str] = None):
        """Set Gym and Numpy seed.
        @param seed_init: Seed.
        """
        self.np_random, seed = seeding.np_random(seed_init)
        if seed_init is not None:
            np.random.seed(seed_init)
        return [seed]

    def step(self, action: Union[list, np.ndarray]) -> Tuple[list, float, bool, dict]:
        """Apply one control step to the environment.
        Note that the period between each control step = action repeat * period of each simulation step.

        @param action: Motor commands.
                Note: step receives normalized actions between -1 and 1.
        @return self._observation: Robot's observations.
                Note: The returned observations are normalized if, self._normalize is True.
        @return reward: Reward earned by the agent in the current step.
        @return done: True if the episode has terminated, False if not.
        @return info: {}
        """
        # if (self._renders):
        #   basePos, orn = self._p.getBasePositionAndOrientation(self._racecar.racecarUniqueId)
        #   #self._p.resetDebugVisualizerCamera(1, 30, -40, basePos)
        if self._policy_feedback:
            self.pwmL_buffer.appendleft(action[0])
            self.pwmR_buffer.appendleft(action[1])

        if self._normalize:
            denormalized_action = self.denormalize_action(action)
        else:
            denormalized_action = action

        # self._robot.apply_action(denormalized_action)
        for i in range(self._action_repeat):
            self._robot.apply_action(denormalized_action)
            self._p.stepSimulation()
            if self._renders:
                elapsed_time = time.time() - self.last_frame_time
                if elapsed_time < self._time_step / 10:
                    time.sleep(self._time_step / 10 - elapsed_time)
                self.last_frame_time = time.time()

            self._robot.linear_accel_update()
            if i == self._action_repeat - 1:
                self._observation = self.get_observation_UPDATE()
            if self._termination():
                if self._algo_mode == 'RL':
                    break
                elif self._algo_mode == 'PID':
                    pass
            self._env_step_counter += 1
        reward = self._reward()
        done = self._termination()
        # print("len=%r" % len(self._observation))

        return self._observation, reward, done, {}

    def render(self, mode='human'):
        # TODO: Check this method
        """ Gym's Render method.
        """
        if mode != "rgb_array":
            return np.array([])
        base_pos, orn = self._p.getBasePositionAndOrientation(self._robot.robotUniqueId)
        view_matrix = self._p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=base_pos,
                                                                distance=self._cam_dist,
                                                                yaw=self._cam_yaw,
                                                                pitch=self._cam_pitch,
                                                                roll=0,
                                                                upAxisIndex=2)
        proj_matrix = self._p.computeProjectionMatrixFOV(fov=60,
                                                         aspect=float(RENDER_WIDTH) / RENDER_HEIGHT,
                                                         nearVal=0.1,
                                                         farVal=100.0)
        (_, _, px, _, _) = self._p.getCameraImage(width=RENDER_WIDTH,
                                                  height=RENDER_HEIGHT,
                                                  viewMatrix=view_matrix,
                                                  projectionMatrix=proj_matrix,
                                                  renderer=pybullet.ER_BULLET_HARDWARE_OPENGL)
        rgb_array = np.array(px)
        rgb_array = rgb_array[:, :, :3]
        return rgb_array

    def close(self):
        """Close pybullet client.
        """
        self._p.disconnect()

    def _termination(self) -> bool:
        """Check if episode has ended.
        @return: True if episode has terminated.
        """
        self.pitch_angle = self._robot.get_pitch()[0]
        return self._env_step_counter > EPISODE_LENGTH or abs(self.pitch_angle) > 0.8

    def _reward(self) -> float:
        """Calculate reward in current timestep.
        @return reward: Computed reward.
        """
        self.pitch_angle = self._robot.get_pitch()[0]
        reward = - np.square(self.pitch_angle)
        return reward

    def normalize_action(self, action: Union[list, np.ndarray]) -> Union[list, np.ndarray]:
        """ Normalize action.
        @param action: Action to normalize.
        @return norm_action: Normalized action.
        """
        # Normalized action -> [-1, 1]
        action_normalizer = self.act_norm
        norm_action = np.array(action) / action_normalizer
        return norm_action

    def denormalize_action(self, normalized_action: Union[list, np.ndarray]) -> Union[list, np.ndarray]:
        """ De-normalize action.
        @param normalized_action: Action to de-normalize.
        @return denorm_action: Denormalized action.
        """
        # Normalized action -> [-1, 1]
        action_normalizer = self.act_norm
        denorm_action = np.array(normalized_action) * action_normalizer
        return denorm_action

    def get_observation_UPDATE(self) -> list:
        """ Get the robot's observation.

        (!) Warning: This method should be called once per simulation step.

        Depending on the initialization of the environment, the observation can contain the robot's pitch,
        data from the accelerometer, gyroscope, and/or a feedback from the previous commanded robot.
        E.g.
        Observation vector: [pitch, ax, ay, az, gx, gy, gz, pwmL, pwmR]  -> Varies depending on observation_dim.
        The observed list may contain also data from older time-steps. The amount (n) of time-steps is determined
        by the variable '_memory_buffer'.
        E.g.
        observation vector: [pitch_1, .., pitch_n, ax_1, ..., ax_n, ay_1, ..., ay_n, ...]

        @return full_obs: List containing the different components of the observation.
        """

        sensor_obs = []

        pitch = self._robot.get_pitch()
        sensor_obs.extend(pitch)

        linear_accel = self._robot.get_linear_accel()  # Update it before calling this method!
        angular_vel = self._robot.get_angular_vel()
        if not self._only_pitch:
            sensor_obs.extend(linear_accel)
            sensor_obs.extend(angular_vel)
        if self._policy_feedback:
            sensor_obs.extend([0])  # We do not want to normalize the pwm feedback
            sensor_obs.extend([0])

        if self._real_imu:
            ay = linear_accel[1]
            az = linear_accel[2]
            gx = angular_vel[0]
            accel_pitch = np.arctan2(ay, az)
            self.pitch_ri = self.filter_tau * (self.pitch_ri - gx * self._time_step * self._action_repeat) + (
                    1 - self.filter_tau) * (-accel_pitch)
            sensor_obs[0] = self.pitch_ri

        if PLOT_DATA and self._real_imu:
            self.real_pitch_buffer.append(pitch[0])
            self.filter_pitch_buffer.append(self.pitch_ri)
            # self.filter_pitch_buffer.append(accel_pitch)
            self.fig.canvas.restore_region(self.fig_background)
            self.fig_points1.set_data(np.arange(len(self.real_pitch_buffer)), self.real_pitch_buffer)
            self.fig_points2.set_data(np.arange(len(self.filter_pitch_buffer)), self.filter_pitch_buffer)
            self.ax.draw_artist(self.fig_points1)
            self.ax.draw_artist(self.fig_points2)
            self.fig.canvas.blit(self.ax.bbox)
            self.fig.canvas.flush_events()
            plt.pause(0.001)

        # TODO: Select State Normalizer.
        if self._normalize:
            # self.normalizer_update(np.array(sensor_obs))
            sensor_obs = self.normalizer_normalize(np.array(sensor_obs))

        self.pitch_buffer.appendleft(sensor_obs[0])
        if not self._only_pitch:
            self.ax_buffer.appendleft(sensor_obs[1])
            self.ay_buffer.appendleft(sensor_obs[2])
            self.az_buffer.appendleft(sensor_obs[3])
            self.gx_buffer.appendleft(sensor_obs[4])
            self.gy_buffer.appendleft(sensor_obs[5])
            self.gz_buffer.appendleft(sensor_obs[6])

        full_obs = []
        full_obs.extend(self.pitch_buffer)
        if not self._only_pitch:
            full_obs.extend(self.ax_buffer)
            full_obs.extend(self.ay_buffer)
            full_obs.extend(self.az_buffer)
            full_obs.extend(self.gx_buffer)
            full_obs.extend(self.gy_buffer)
            full_obs.extend(self.gz_buffer)
        if self._policy_feedback:
            full_obs.extend(self.pwmL_buffer)
            full_obs.extend(self.pwmR_buffer)
        return full_obs

    def normalizer_update(self, sensor_obs: np.ndarray):
        """ Update observation normalizer.

        @param sensor_obs: New observation.
        """
        # alfa = 0.05
        self.norm_ctr += 1
        alfa = 1 / self.norm_ctr
        last_mean = self.norm_mean.copy()

        # Running Average
        self.norm_mean = (1 - alfa) * self.norm_mean + alfa * sensor_obs

        # used to compute variance
        self.mean_diff += (sensor_obs - last_mean) * (sensor_obs - self.norm_mean)
        # Variance
        self.norm_var = (alfa * self.mean_diff).clip(min=1e-2)
        # Standard Deviation
        self.norm_std = np.sqrt(self.norm_var)

    def normalizer_reset(self):
        """Reset observation normalizer.
        """
        self.norm_ctr = 0
        self.norm_mean = np.zeros(int(self.observation_dim / self._memory_buffer))
        self.mean_diff = np.zeros(int(self.observation_dim / self._memory_buffer))

    def normalizer_normalize(self, sensor_obs: np.ndarray) -> np.ndarray:
        """Normalize observation.
        @param sensor_obs: Observation to normalize.
        @return: Normalized observation.
        """
        state_mean = self.norm_mean
        state_std = self.norm_std
        return (sensor_obs - state_mean) / state_std

    def normalizer_denormalize(self, normalized_obs: np.ndarray) -> np.ndarray:
        """Denormalize observation.
        @param normalized_obs: Observation to de-normalize.
        @return: De-normalized observation.
        """
        # Normalized observation -> [-1, 1]
        denorm_obs = normalized_obs * self.norm_std + self.norm_mean
        return denorm_obs

    def add_sliders(self):
        """Add pitch and yaw sliders to the Pybullet GUI.
        """
        self.tita_slider = self._p.addUserDebugParameter(paramName='Tita', rangeMin=-0.1, rangeMax=0.1, startValue=0.0)
        self.yaw_slider = self._p.addUserDebugParameter(paramName='Yaw Rate (Dimensionless)', rangeMin=-1, rangeMax=1,
                                                        startValue=0.0)

    def get_slider_tita(self):
        """Get the pitch value from the GUI slider.
        @return: Pitch value.
        """
        return self._p.readUserDebugParameter(self.tita_slider)

    def get_slider_yaw(self):
        """Get the yaw value from the GUI slider.
        @return: Yaw value.
        """
        return self._p.readUserDebugParameter(self.yaw_slider)

    if parse_version(gym.__version__) < parse_version('0.9.6'):
        _render = render
        _reset = reset
        _seed = seed
        _step = step
