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
from stable_baselines import A2C
import argparse

# Instantiate the parser
parser = argparse.ArgumentParser(description='Script to evaluate RL agents')
parser.add_argument("-mn", "--modelName", action='store', default='model', type=str,
                    help="Name of the model to be evaluated [Default: model]")
args = parser.parse_args()

# Params
CONVERT2KERAS = False  # Save Stable Baselines model to Keras, and evaluate it.
MODEL_NAME = args.modelName  # Folder name where best_model.zip is held.
NET_LAYERS = [32, 32]
MEMORY_BUFFER = 1

# Environment
NORMALIZE = True
BACKLASH = True
SEED = 5
LoopFreq = 100  # Hz
StepPeriod = (1 / 240) * 1 / 10  # s
actions_per_step = int(round((1 / LoopFreq) / StepPeriod))  # For Microcontroller loop frequency compatibility

# Directories
training_save_path = os.path.join('../rl_data/models/')


def main():
    model = A2C.load(os.path.join(training_save_path, MODEL_NAME, 'best_model'))
    env = balancioGymEnv.BalancioGymEnv(action_repeat=actions_per_step, renders=True, normalize=NORMALIZE,
                                        backlash=BACKLASH, memory_buffer=MEMORY_BUFFER)

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
            env.render()


if __name__ == '__main__':
    main()
