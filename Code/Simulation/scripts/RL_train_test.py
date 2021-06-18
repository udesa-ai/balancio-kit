import balancioGymEnv_simple
import stable_baselines
from stable_baselines.common.policies import MlpPolicy
from stable_baselines.common.vec_env import DummyVecEnv
from stable_baselines import PPO2

import tensorflow as tf


NORMALIZE = False

env = balancioGymEnv_simple.BalancioGymEnv(renders=False, normalize=NORMALIZE)
policy_kwargs = dict(act_fun=tf.nn.relu, net_arch=[16, 16])
model = PPO2(MlpPolicy, env, policy_kwargs=policy_kwargs, verbose=1)
model.learn(total_timesteps=1000000)  # 1000000

env = balancioGymEnv_simple.BalancioGymEnv(renders=True, normalize=NORMALIZE)
while False:
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
