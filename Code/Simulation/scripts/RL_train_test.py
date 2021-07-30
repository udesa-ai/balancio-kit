import balancioGymEnv_simple
import balancioGymEnv

import tensorflow as tf
# tf.compat.v1.logging.set_verbosity(tf.compat.v1.logging.ERROR)

import stable_baselines
from stable_baselines.common.policies import MlpPolicy
from stable_baselines.common.vec_env import DummyVecEnv, SubprocVecEnv
from stable_baselines import PPO2, A2C, ACKTR
from stable_baselines.common.callbacks import EvalCallback, StopTrainingOnRewardThreshold
from stable_baselines.common import set_global_seeds

import os


# Policy
TIMESTEPS = 1000000  # 1000000
EVAL_FREQ = 5000
NUM_CPU = 10  # Number of processes to uses -> More implies more samples per time, but less efficiency.
NET_LAYERS = [32, 32]
# Environment
NORMALIZE = True
BACKLASH = True
SEED = 5
# Directories
training_save_path = os.path.join('Models', 'test')
training_log_path = os.path.join('Logs', 'test')


def make_env(rank, seed=0):
    """
    Utility function for multiprocessed env.

    :param seed: (int) the inital seed for RNG
    :param rank: (int) index of the subprocess
    """
    def _init():
        env = balancioGymEnv.BalancioGymEnv(renders=False, normalize=NORMALIZE, backlash=BACKLASH, seed=seed+rank)
        env.seed(seed + rank)
        return env
    set_global_seeds(seed)
    return _init

# env = balancioGymEnv.BalancioGymEnv(renders=False, normalize=NORMALIZE, backlash=BACKLASH)
# env = DummyVecEnv([lambda: env])


if __name__ == '__main__':

    env = SubprocVecEnv([make_env(rank=i, seed=SEED) for i in range(NUM_CPU)])
    eval_env = DummyVecEnv([lambda: balancioGymEnv.BalancioGymEnv(renders=False, normalize=NORMALIZE, backlash=BACKLASH)])

    eval_callback = EvalCallback(eval_env,
                                 eval_freq=EVAL_FREQ,
                                 best_model_save_path=training_save_path,
                                 verbose=1)
    policy_kwargs = dict(act_fun=tf.nn.relu, net_arch=NET_LAYERS)
    # model = PPO2(MlpPolicy, env, policy_kwargs=policy_kwargs, verbose=1, tensorboard_log=training_log_path)
    model = A2C(MlpPolicy, env, gamma=0.9, n_steps=16, ent_coef=3.2155672659533806e-05, max_grad_norm=0.6, vf_coef=0.4439617266032203,
                learning_rate=0.004030614464204483, alpha=0.9, policy_kwargs=policy_kwargs, verbose=1, tensorboard_log=training_log_path)
    model.learn(total_timesteps=TIMESTEPS, callback=eval_callback)

    env = balancioGymEnv.BalancioGymEnv(renders=True, normalize=NORMALIZE, backlash=BACKLASH)
    while True:
        obs = env.reset()
        done = False
        while done is False:
            action, _states = model.predict(obs, deterministic=True)  # deterministic false wtf(?)
            obs, reward, done, info = env.step(action)
            env.render()


    """ Policy Export """
    # with model.graph.as_default():
    #     tf.saved_model.simple_save(model.sess, 'Models/test', inputs={"obs": model.act_model.obs_ph},
    #                                outputs={"action": model.action_ph})  # ._policy_proba / model.action_ph

    # with model.graph.as_default():
    #     tf.saved_model.simple_save(model.sess, 'Models/test', inputs={"obs": model.act_model.obs_ph},
    #                                    outputs={"action": model.act_model._policy_proba})


    if False:
        keras_model = tf.keras.Sequential()
        keras_model.add(tf.keras.layers.Dense(16, input_dim=1))
        keras_model.add(tf.keras.layers.Activation('relu'))
        keras_model.add(tf.keras.layers.Dense(16))
        keras_model.add(tf.keras.layers.Activation('relu'))
        keras_model.add(tf.keras.layers.Dense(1))

        params = model.get_parameters()
        keras_model.layers[0].set_weights([params['model/shared_fc0/w:0'], params['model/shared_fc0/b:0']])
        keras_model.layers[2].set_weights([params['model/shared_fc1/w:0'], params['model/shared_fc1/b:0']])
        keras_model.layers[4].set_weights([params['model/pi/w:0'], params['model/pi/b:0']])

        keras_model.save('Models/model.h5')

        while True:
            obs = env.reset()
            done = False
            while done is False:
                action = keras_model.predict(obs)
                sb_action = model.predict(obs, deterministic=True)
                diff = action-sb_action[0]
                print("Diferencia entre predicciones: ", diff[0][0])
                obs, reward, done, info = env.step(action)
                env.render()
