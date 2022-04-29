# ======================================================================
#  Balancio-Kit (c) 2021 Linar (UdeSA)
#  This code is licensed under MIT license (see LICENSE.txt for details)
# ======================================================================
"""
Script for training an RL agent for the Balancio.
"""

# Filter tensorflow version warnings
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)
warnings.simplefilter(action='ignore', category=Warning)
import tensorflow as tf
tf.get_logger().setLevel('INFO')
tf.autograph.set_verbosity(0)
import logging
tf.get_logger().setLevel(logging.ERROR)

from balancio_lib.environments import balancioGymEnv
from stable_baselines.common.policies import MlpPolicy
from stable_baselines.common.vec_env import DummyVecEnv, SubprocVecEnv
from stable_baselines import A2C
from stable_baselines.common.callbacks import EvalCallback
from stable_baselines.common import set_global_seeds
import argparse
from typing import Callable

# Instantiate the parser
parser = argparse.ArgumentParser(description='Script to train RL agent')
parser.add_argument("-mn", "--modelName", action='store', default='model', type=str,
                    help="Name where model and logs will be saved to [Default: model]")
args = parser.parse_args()

# Policy hyperparameters
MODEL_NAME = args.modelName
TIMESTEPS = 500000  # 1000000
EVAL_FREQ = 5000
NUM_CPU = 8  # Number of processes to uses -> More implies more samples per time, but less efficiency.
NET_LAYERS = [32, 32]
MEMORY_BUFFER = 1

# Environment
NORMALIZE = True
BACKLASH = True
SEED = 5
LoopFreq = 100  # Hz
StepPeriod = (1 / 240) * 1 / 10  # s
actions_per_step = int(round((1 / LoopFreq) / StepPeriod))  # For Microcontroller loop frequency compatibility


def make_env(rank: int, seed: int = 0) -> Callable:
    """
    Utility function for multiprocessed env.

    @param rank: Index of the subprocess
    @param seed: The initial seed for RNG
    @return _init:
    """

    def _init():
        env = balancioGymEnv.BalancioGymEnv(action_repeat=actions_per_step, renders=False, normalize=NORMALIZE,
                                            backlash=BACKLASH, seed=seed + rank, memory_buffer=MEMORY_BUFFER)
        env.seed(seed + rank)
        return env

    set_global_seeds(seed)
    return _init


def main():
    """Main training script."""
    # Directories
    training_save_path = os.path.join('../rl_data/models/', MODEL_NAME)
    training_log_path = os.path.join('../rl_data/logs/', MODEL_NAME)
    if not os.path.exists(training_save_path):
        os.makedirs(training_save_path)
    if not os.path.exists(training_log_path):
        os.makedirs(training_log_path)

    # Wrapper of environments used for training.
    train_env = SubprocVecEnv([make_env(rank=i, seed=SEED) for i in range(NUM_CPU)])
    # Environment used for evaluation.
    eval_env = DummyVecEnv([lambda: balancioGymEnv.BalancioGymEnv(action_repeat=actions_per_step, renders=False,
                                                                  normalize=NORMALIZE, backlash=BACKLASH,
                                                                  memory_buffer=MEMORY_BUFFER)])
    eval_callback = EvalCallback(eval_env,
                                 eval_freq=EVAL_FREQ,
                                 best_model_save_path=training_save_path,
                                 verbose=1)
    policy_kwargs = dict(act_fun=tf.nn.relu, net_arch=NET_LAYERS)
    # model = PPO2(MlpPolicy, env, policy_kwargs=policy_kwargs, verbose=1, tensorboard_log=training_log_path)
    model = A2C(MlpPolicy, train_env, gamma=0.9, n_steps=16, ent_coef=3.2155672659533806e-05, max_grad_norm=0.6,
                vf_coef=0.4439617266032203, learning_rate=0.004030614464204483, alpha=0.9,
                policy_kwargs=policy_kwargs, verbose=1, tensorboard_log=training_log_path)
    model.learn(total_timesteps=TIMESTEPS, callback=eval_callback)

    # Trained agent testing.
    test_env = balancioGymEnv.BalancioGymEnv(action_repeat=actions_per_step, renders=True, normalize=NORMALIZE,
                                             backlash=BACKLASH, memory_buffer=MEMORY_BUFFER)
    while True:
        obs = test_env.reset()
        done = False
        while done is False:
            action, _states = model.predict(obs, deterministic=True)
            obs, reward, done, info = test_env.step(action)
            test_env.render()


if __name__ == '__main__':
    main()
