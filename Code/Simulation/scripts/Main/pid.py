import numpy as np
from Code.Simulation.scripts.Environments import balancioGymEnv

LoopFreq = 100  # Hz
StepPeriod = (1/240) * 1/10  # s
actions_per_step = int(round((1/LoopFreq)/StepPeriod))  # For Microcontroller loop frequency compatibility
env = balancioGymEnv.BalancioGymEnv(action_repeat=actions_per_step, renders=True, normalize=True, backlash=True, real_imu=True, seed=None, algo_mode='PID', memory_buffer=1, policy_feedback=True, only_pitch=False)

Kp = -2000      # -6.5
Ki = -22000     # -0.28
Kd = -20        # -0.20
tita_target = 0.0
delta_tita = 0
previous_delta_tita = 0
errorSum = 0
pid_p = 0
pid_i = 0
pid_d = 0

tita = env.reset()[0]
tita = env.normalizer_denormalize(tita)[0]

env.add_sliders()
done = False

# Buffer for mean and std deviation estimation.
buffer = []

while True:

    if done:
        tita = env.reset()[0]
        tita = env.normalizer_denormalize(tita)[0]
        errorSum = 0

    tita_target = env.get_slider_tita()
    yaw_rate = env.get_slider_yaw()
    delta_tita = tita_target - tita

    pid_p = Kp * delta_tita
    errorSum += delta_tita * (1/LoopFreq)
    errorSum = np.clip(errorSum, -5, 5)
    pid_i = Ki * errorSum
    pid_d = Kd * ((delta_tita - previous_delta_tita) / (1/LoopFreq))
    previous_delta_tita = delta_tita
    pwm = pid_p + pid_i + pid_d
    pwm = pwm/255
    # print(omega)
    # omega = 10*tita_target
    pwm = np.clip(pwm, -1, 1)
    action = [pwm + 0.2*yaw_rate, pwm - 0.2*yaw_rate]
    normalized_action = env.normalize_action(action)
    tita, rew, done, _ = env.step(normalized_action)

    # buffer.append(tita)

    tita = tita[0]
    # print(tita)
    tita = env.normalizer_denormalize(tita)[0]
