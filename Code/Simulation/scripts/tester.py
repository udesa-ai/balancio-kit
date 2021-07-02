import balancioGymEnv_simple
import balancioGymEnv

import stable_baselines
from stable_baselines.common.policies import MlpPolicy
from stable_baselines.common.vec_env import DummyVecEnv, SubprocVecEnv
from stable_baselines import PPO2
from stable_baselines.common.callbacks import EvalCallback, StopTrainingOnRewardThreshold

import tensorflow as tf
import os


env = balancioGymEnv.BalancioGymEnv(renders=True, normalize=True, backlash=True)
env = DummyVecEnv([lambda: env])

model = PPO2.load('Models/test/best_model', env=env)

while True:
    obs = env.reset()
    done = False
    while done == False:
        action, _states = model.predict(obs, deterministic=True)  # deterministic false wtf(?)
        obs, reward, done_a, info = env.step(action)
        done = done_a[0]
        env.render()
