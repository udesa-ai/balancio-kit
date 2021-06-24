import balancioGymEnv


env = balancioGymEnv.BalancioGymEnv(renders=True)

Kp = -6.5
Ki = -0.28
Kd = -0.20
tita_target = 0.0
delta_tita = 0
previous_delta_tita = 0
pid_p = 0
pid_i = 0
pid_d = 0

tita = env.reset()[0]
tita = env.denormalize_observation(tita)

env.add_sliders()

while True:
    tita_target = env.get_slider_tita()
    yaw_rate = env.get_slider_yaw()
    delta_tita = tita_target - tita

    pid_p = Kp * delta_tita
    pid_i += Ki * delta_tita
    pid_d = Kd * ((delta_tita - previous_delta_tita) / 0.005)
    omega = pid_p + pid_i + pid_d
    # print(omega)
    action = [omega + 0.2*yaw_rate, omega - 0.2*yaw_rate]
    normalized_action = env.normalize_action(action)
    tita = env.step(normalized_action)[0][0]
    tita = env.denormalize_observation(tita)
