
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

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

RENDER_HEIGHT = 720
RENDER_WIDTH = 960
EPISODE_LENGTH = 500

class BalancioGymEnv(gym.Env):
    metadata = {'render.modes': ['human', 'rgb_array'], 'video.frames_per_second': 50}

    def __init__(self,
                 urdf_root=pybullet_data.getDataPath(),
                 action_repeat=1,
                 is_discrete=False,
                 renders=False,
                 normalize=True,
                 backlash=True,
                 seed=None):
        self._time_step = 0.01
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
        self.seed(seed)
        # self.reset()
        observation_dim = 1
        print("Observation dimension: {}".format(observation_dim))

        # Normalization parameters
        self.obs_norm = np.pi * 0.5
        max_act = 1
        self.act_norm = max_act * np.array([1, 1])

        observation_high = np.ones(observation_dim)
        if not self._normalize:
            observation_high = observation_high * self.obs_norm

        if is_discrete:
            self.action_space = spaces.Discrete(9)
        else:
            action_dim = 1
            if self._normalize:
                self._action_bound = 1
            else:
                self._action_bound = max_act
            action_high = np.array([self._action_bound] * action_dim)
            self.action_space = spaces.Box(-action_high, action_high, dtype=np.float32)
        self.observation_space = spaces.Box(-observation_high, observation_high, dtype=np.float32)
        self.viewer = None

        self.pitch_angle = 0

    def reset(self):
        self._p.resetSimulation()
        #p.setPhysicsEngineParameter(numSolverIterations=300)
        self._p.setTimeStep(self._time_step)
        self._p.loadURDF(os.path.join(self._urdf_root, "plane.urdf"))
        # stadiumobjects = self._p.loadSDF(os.path.join(self._urdfRoot, "stadium.sdf"))

        self._p.setGravity(0, 0, -9.81)
        self._robot = balancio.Balancio(self._p, urdf_root_path=self._urdf_root, time_step=self._time_step, backlash=self._backlash)
        self._env_step_counter = 0
        for i in range(5):
            self._p.stepSimulation()
        if self._normalize:
            self._observation = self.get_normalized_observation()
        else:
            self._observation = self._robot.get_observation()
        return np.array(self._observation)

    def __del__(self):
        self._p = 0

    def seed(self, seed_init=None):
        self.np_random, seed = seeding.np_random(seed_init)
        if seed_init is not None:
            np.random.seed(seed_init)
        return [seed]

    def step(self, action):
        """ Note: step receives normalized actions between -1 and 1,
            and returns also normalized observations between -1 and 1."""
        # if (self._renders):
        #   basePos, orn = self._p.getBasePositionAndOrientation(self._racecar.racecarUniqueId)
        #   #self._p.resetDebugVisualizerCamera(1, 30, -40, basePos)
        vec_action = action[0] * np.array([1, 1])

        if self._normalize:
            vec_action = self.denormalize_action(vec_action)

        self._robot.apply_action(vec_action)
        for i in range(self._action_repeat):
            self._p.stepSimulation()
            if self._renders:
                time.sleep(self._time_step)
            if self._normalize:
                self._observation = self.get_normalized_observation()
            else:
                self._observation = self._robot.get_observation()

            if self._termination():
                break
            self._env_step_counter += 1
        reward = self._reward()
        done = self._termination()
        #print("len=%r" % len(self._observation))

        return np.array(self._observation), reward, done, {}

    def render(self, mode='human', close=False):
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
        self._p.disconnect()

    def _termination(self):
        self.pitch_angle = self._robot.get_observation()[0]
        return self._env_step_counter > EPISODE_LENGTH or abs(self.pitch_angle) > 0.8

    def _reward(self):
        self.pitch_angle = self._robot.get_observation()[0]
        reward = - np.square(self.pitch_angle)
        return reward

    def normalize_action(self, action):
        # Normalized action -> [-1, 1]
        action_normalizer = self.act_norm
        norm_action = np.array(action) / action_normalizer
        return norm_action

    def denormalize_action(self, normalized_action):
        # Normalized action -> [-1, 1]
        action_normalizer = self.act_norm
        denorm_action = np.array(normalized_action) * action_normalizer
        return denorm_action

    def get_normalized_observation(self):
        obs = self._robot.get_observation()[0]
        obs_normalizer = self.obs_norm
        norm_obs = obs/obs_normalizer
        return [norm_obs]

    def denormalize_observation(self, normalized_obs):
        # Normalized action -> [-1, 1]
        obs_normalizer = self.obs_norm
        denorm_obs = normalized_obs * obs_normalizer
        return denorm_obs

    def add_sliders(self):
        self.tita_slider = self._p.addUserDebugParameter(paramName='Tita', rangeMin=-0.1, rangeMax=0.1, startValue=0.0)
        self.yaw_slider = self._p.addUserDebugParameter(paramName='Yaw Rate (Dimensionless)', rangeMin=-1, rangeMax=1, startValue=0.0)

    def get_slider_tita(self):
        return self._p.readUserDebugParameter(self.tita_slider)

    def get_slider_yaw(self):
        return self._p.readUserDebugParameter(self.yaw_slider)

    if parse_version(gym.__version__) < parse_version('0.9.6'):
        _render = render
        _reset = reset
        _seed = seed
        _step = step
