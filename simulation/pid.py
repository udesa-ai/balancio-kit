# ======================================================================
#  Balancio-Kit (c) 2021 Linar (UdeSA)
#  This code is licensed under MIT license (see LICENSE.txt for details)
# ======================================================================

import numpy as np
from balancio_lib.environments import balancioGymEnv
import argparse


# Instantiate the parser
parser = argparse.ArgumentParser(
    description='PID control algorithm script for the Balancio-Kit simulation.')
parser.add_argument("-kp", "--kp", action='store', default=2000.0,
                    type=float, help="Proportional gain (Default: 2000).")
parser.add_argument("-ki", "--ki", action='store', default=22000.0,
                    type=float, help="Derivative gain (Default: 22000).")
parser.add_argument("-kd", "--kd", action='store', default=20.0,
                    type=float, help="Integral gain (Default: 20).")
parser.add_argument("--RealIMU",   action='store_true',
                    help="Simulate real IMU [IN DEVELOPMENT]")
args = parser.parse_args()

if args.RealIMU:
    real_imu = True
else:
    real_imu = False

# Loop control frequency. This freq. has to match the one used in the microcontroller.
LoopFreq = 100  # Hz
# Simulation step period.
StepPeriod = (1/240) * 1/10  # s
# Simulation steps in each control cycle.
# For Microcontroller loop frequency compatibility
actions_per_step = int(round((1/LoopFreq)/StepPeriod))

# Gym environment.
env = balancioGymEnv.BalancioGymEnv(action_repeat=actions_per_step, renders=True,
                                    normalize=True, backlash=True, real_imu=real_imu, seed=None, algo_mode='PID')

# Variables.
Kp = args.kp
Ki = args.ki
Kd = args.kd
tita_target = 0.0
delta_tita = 0
previous_delta_tita = 0
errorSum = 0
pid_p = 0
pid_i = 0
pid_d = 0

# Reset agent.
tita = env.reset()[0]
tita = env.normalizer_denormalize(tita)[0]

# Add sliders in GUI to command robot.
env.add_sliders()

done = False

while True:

    if done:
        tita = env.reset()[0]
        tita = env.normalizer_denormalize(tita)[0]
        errorSum = 0

    # Get commands.
    tita_target = env.get_slider_tita()
    yaw_rate = env.get_slider_yaw()

    # Calculate error.
    delta_tita = tita_target - tita

    # Compute PID.
    pid_p = Kp * delta_tita
    errorSum += delta_tita * (1/LoopFreq)
    errorSum = np.clip(errorSum, -5, 5)
    pid_i = Ki * errorSum
    pid_d = Kd * ((delta_tita - previous_delta_tita) / (1/LoopFreq))
    previous_delta_tita = delta_tita
    pwm = -(pid_p + pid_i + pid_d)
    pwm = pwm/255
    pwm = np.clip(pwm, -1, 1)

    # action: [left, right]
    action = [pwm + 0.2*yaw_rate, pwm - 0.2*yaw_rate]
    # Normalize action
    normalized_action = env.normalize_action(action)

    # Step simulation. Get new state.
    tita, rew, done, _ = env.step(normalized_action)
    tita = tita[0]
    # print(tita)
    tita = env.normalizer_denormalize(tita)[0]
