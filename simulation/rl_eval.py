# ======================================================================
#  Balancio-Kit (c) 2021 Linar (UdeSA)
#  This code is licensed under MIT license (see LICENSE.txt for details)
# ======================================================================
"""
Script for evaluating an RL agent on the Balancio.
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
from stable_baselines import A2C
import argparse


# TODO: Automate model conversion.

# Instantiate the parser
parser = argparse.ArgumentParser(description='Script to evaluate RL agents')
parser.add_argument("-mn", "--ModelName", action='store', default='A2C_p_1_dev', type=str,
                    help="Name of the model to be evaluated [Default: A2C_p_1_dev]")
parser.add_argument("-a", "--Algo", action='store', default='A2C', type=str,
                    help="Reinforcement Learning algorithm used during training [Default: 'A2C'].")
parser.add_argument("-en", "--EnvName", action='store', default='p_1', type=str,
                    help="Environment name: 'pif_b' --> p if pitch, i if imu, f if feedback, b buffer length. [Default: 'p_1'].")
parser.add_argument("-rw", "--RewardWrapper", action='store', default='None', type=str,
                    help="Apply a reward wrapper to change the default reward [Optional].")
args = parser.parse_args()

# Params
CONVERT2KERAS = False  # Save Stable Baselines model to Keras, and evaluate it.
MODEL_NAME = args.ModelName  # Folder name where best_model.zip is held.
NET_LAYERS = [32, 32]

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

# Directories
training_save_path = os.path.join('../rl_data/models/')


def main():
    if args.Algo.upper() == "A2C":
        model = A2C.load(os.path.join(training_save_path, MODEL_NAME, 'best_model'))
    else:
        raise Exception("Insert a compatible RL algorithm: A2C, ...")

    # Add useful wrappers around the environment
    reward_wrapper = RewardWrappers.get_reward_wrapper(args.RewardWrapper)

    env = reward_wrapper(balancioGymEnv.BalancioGymEnv(action_repeat=actions_per_step, renders=True, normalize=NORMALIZE,
                                                       backlash=BACKLASH, memory_buffer=memory_buffer, only_pitch=only_pitch,
                                                       policy_feedback=policy_feedback))

    # Convert stable-baselines model to keras, for further lite conversion.
    if CONVERT2KERAS:
        keras_model = tf.keras.Sequential()
        keras_model.add(tf.keras.layers.Dense(NET_LAYERS[0], input_dim=1))
        keras_model.add(tf.keras.layers.Activation('relu'))
        keras_model.add(tf.keras.layers.Dense(NET_LAYERS[1]))
        keras_model.add(tf.keras.layers.Activation('relu'))
        keras_model.add(tf.keras.layers.Dense(2))

        params = model.get_parameters()
        keras_model.layers[0].set_weights([params['model/shared_fc0/w:0'], params['model/shared_fc0/b:0']])
        keras_model.layers[2].set_weights([params['model/shared_fc1/w:0'], params['model/shared_fc1/b:0']])
        keras_model.layers[4].set_weights([params['model/pi/w:0'], params['model/pi/b:0']])

        keras_model.save(os.path.join(training_save_path, MODEL_NAME, 'model.h5'))

    while True:
        obs = env.reset()
        done = False
        cumulative_reward = 0
        while done is False:
            if CONVERT2KERAS:
                action = keras_model.predict(obs)[0]
                action.clip(-1, 1)
                sb_action, _states = model.predict(obs, deterministic=True)
                diff = action - sb_action
                print("Diferencia entre predicciones: ", diff)
            else:
                action, _states = model.predict(obs, deterministic=True)
            obs, reward, done, info = env.step(action)
            cumulative_reward += reward
            env.render()
        print("Episode's accumulated reward: {}".format(cumulative_reward))


if __name__ == '__main__':
    main()
