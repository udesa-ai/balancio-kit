import numpy as np
import balancioGymEnv_simple
import balancioGymEnv

import tensorflow as tf
# tf.compat.v1.logging.set_verbosity(tf.compat.v1.logging.ERROR)

from stable_baselines import PPO2, A2C, ACKTR

import os


# Params
CONVERT2KERAS = True    # Save Stable Baselines model to Keras, and evaluate it.
MODEL_NAME = 'test'     # Folder name where best_model.zip is held.
NET_LAYERS = [32, 32]
# Environment
NORMALIZE = True
BACKLASH = True
SEED = 5

# Directories
training_save_path = os.path.join('Models', 'my_models')
training_log_path = os.path.join('Logs', 'my_logs')


LoopFreq = 100  # Hz
StepPeriod = (1/240) * 1/10  # s
actions_per_step = int(round((1/LoopFreq)/StepPeriod))  # For Microcontroller loop frequency compatibility


model = A2C.load(os.path.join(training_save_path, MODEL_NAME, 'best_model'))
env = balancioGymEnv.BalancioGymEnv(action_repeat=actions_per_step, renders=True, normalize=NORMALIZE, backlash=BACKLASH, algo_mode='RL')


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
