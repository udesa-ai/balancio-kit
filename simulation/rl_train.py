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
from balancio_lib.wrappers import RewardWrappers
from stable_baselines.common.policies import MlpPolicy
from stable_baselines.common.vec_env import DummyVecEnv, SubprocVecEnv
from stable_baselines import A2C
from stable_baselines.common.callbacks import EvalCallback
from stable_baselines.common import set_global_seeds
import argparse
from typing import Callable
import yaml

# Instantiate the parser
parser = argparse.ArgumentParser(description='Script to train RL agent.')
parser.add_argument("-a", "--Algo", action='store', default='A2C', type=str,
                    help="Reinforcement Learning algorithm used during training [Default: 'A2C'].")
parser.add_argument("-en", "--EnvName", action='store', default='p_1', type=str,
                    help="Environment name: 'pif_b' --> p if pitch, i if imu, f if feedback, b buffer length. [Default: 'p_1'].")
parser.add_argument("-rw", "--RewardWrapper", action='store', default='None', type=str,
                    help="Apply a reward wrapper to change the default reward [Optional].")
args = parser.parse_args()


# Policy hyperparameters
MODEL_NAME = args.Algo.upper() + "_" + args.EnvName
TIMESTEPS = 500000  # 1000000
EVAL_FREQ = 5000
NUM_CPU = 8  # Number of processes to uses -> More implies more samples per time, but less efficiency.

# Environment
NORMALIZE = True
BACKLASH = True
SEED = 5
memory_buffer = int(args.EnvName[args.EnvName.find("_")+1::])
only_pitch = not 'i' in args.EnvName
policy_feedback = 'f' in args.EnvName
LoopFreq = 100  # Hz
StepPeriod = (1 / 240) * 1 / 10  # s
actions_per_step = int(round((1 / LoopFreq) / StepPeriod))  # For Microcontroller loop frequency compatibility


def make_env(env_wrapper: Callable, rank: int, seed: int = 0) -> Callable:
    """
    Utility function for multiprocessed env.

    @param env_wrapper: Additional environment wrapper.
    @param rank: Index of the subprocess.
    @param seed: The initial seed for RNG.
    @return _init: Function that creates an environment.
    """

    def _init():
        env = env_wrapper(balancioGymEnv.BalancioGymEnv(action_repeat=actions_per_step, renders=False,
                                                        normalize=NORMALIZE, backlash=BACKLASH,
                                                        seed=seed + rank, memory_buffer=memory_buffer,
                                                        only_pitch=only_pitch,
                                                        policy_feedback=policy_feedback))
        return env

    set_global_seeds(seed)
    return _init


def main():
    """Main training script."""
    # Directories
    training_save_path = os.path.join('../rl_data/models/', MODEL_NAME + '_dev')
    training_log_path = os.path.join('../rl_data/logs/', MODEL_NAME)
    if not os.path.exists(training_save_path):
        os.makedirs(training_save_path)
    if not os.path.exists(training_log_path):
        os.makedirs(training_log_path)

    # Add useful wrappers around the environment
    reward_wrapper = RewardWrappers.get_reward_wrapper(args.RewardWrapper)

    # Wrapper of environments used for training.
    train_env = SubprocVecEnv([make_env(reward_wrapper, rank=i, seed=SEED) for i in range(NUM_CPU)])
    # Environment used for evaluation.
    eval_env = DummyVecEnv([lambda: reward_wrapper(balancioGymEnv.BalancioGymEnv(action_repeat=actions_per_step, renders=False,
                                                                                 normalize=NORMALIZE, backlash=BACKLASH,
                                                                                 memory_buffer=memory_buffer, only_pitch=only_pitch,
                                                                                 policy_feedback=policy_feedback))])
    eval_callback = EvalCallback(eval_env,
                                 eval_freq=int(EVAL_FREQ/NUM_CPU),
                                 best_model_save_path=training_save_path,
                                 verbose=1)

    # Load RL algorithm hyperparameters.
    with open('../rl_data/hyperparameters/{}.yaml'.format(args.Algo.upper()), 'r') as stream:
        try:
            rl_algo_hp = yaml.safe_load(stream)[args.EnvName]
        except yaml.YAMLError as exc:
            print(exc)

    net_layers = 2*[rl_algo_hp["neurons_layer"]]
    policy_kwargs = dict(act_fun=tf.nn.relu, net_arch=net_layers)
    # model = PPO2(MlpPolicy, env, policy_kwargs=policy_kwargs, verbose=1, tensorboard_log=training_log_path)

    if args.Algo.upper() == "A2C":
        model = A2C(MlpPolicy, train_env, gamma=rl_algo_hp["gamma"], n_steps=rl_algo_hp["n_steps"],
                    ent_coef=rl_algo_hp["ent_coef"], max_grad_norm=rl_algo_hp["max_grad_norm"],
                    vf_coef=rl_algo_hp["vf_coef"], learning_rate=rl_algo_hp["learning_rate"], alpha=rl_algo_hp["alpha"],
                    policy_kwargs=policy_kwargs, verbose=1, tensorboard_log=training_log_path)
    else:
        raise Exception("Insert a compatible RL algorithm: A2C, ...")

    model.learn(total_timesteps=TIMESTEPS, callback=eval_callback)

    # Trained agent testing.
    if args.Algo.upper() == "A2C":
        best_model = A2C.load(os.path.join(training_save_path, 'best_model'))
    else:
        raise Exception("Insert a compatible RL algorithm: A2C, ...")

    test_env = reward_wrapper(balancioGymEnv.BalancioGymEnv(action_repeat=actions_per_step, renders=True, normalize=NORMALIZE,
                                             backlash=BACKLASH, memory_buffer=memory_buffer, only_pitch=only_pitch,
                                             policy_feedback=policy_feedback))
    try:
        while True:
            obs = test_env.reset()
            done = False
            while done is False:
                action, _states = best_model.predict(obs, deterministic=True)
                obs, reward, done, info = test_env.step(action)
                test_env.render()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
