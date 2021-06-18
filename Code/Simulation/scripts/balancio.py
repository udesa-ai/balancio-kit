""" Balancio: Pybullet self-balancing robot"""


class Balancio:

    def __init__(self, bullet_client, urdf_root_path='', time_step=0.01):
        self.urdf_root_path = urdf_root_path  # Usar para mnodificar el path del URDF a uno relativo (?)
        self.time_step = time_step  # Intentar dejarlo en el standar (240hz)
        self._p = bullet_client

        self.robotUniqueId = None
        self.max_vel = 10  # rad/s
        self.max_force = 20
        self.motors_num = 2
        self.joint_name2idx = {'left_wheel': 0,
                               'right_wheel': 1}

        self.reset()

    def reset(self):
        robot = self._p.loadURDF("/home/agus/Documents/UdeSA/Balancio_V0/Code/Simulation/urdf/balancio_v0.urdf",
                                 [0, 0, 0.6],
                                 useFixedBase=False)
        self.robotUniqueId = robot

    def get_action_dimension(self):
        return self.motors_num

    def get_observation_dimension(self):
        return len(self.get_observation())

    def get_observation(self):
        observation = []
        pos, orn = self._p.getBasePositionAndOrientation(self.robotUniqueId)

        orn_euler = self._p.getEulerFromQuaternion(orn)  # Pitch
        observation.extend([orn_euler[1]])

        return observation

    def apply_action(self, motor_commands):
        target_vel_left = motor_commands[0]
        target_vel_right = motor_commands[1]

        self._p.setJointMotorControlArray(bodyUniqueId=self.robotUniqueId,
                                          jointIndices=[self.joint_name2idx['left_wheel'], self.joint_name2idx['right_wheel']],
                                          controlMode=self._p.VELOCITY_CONTROL,
                                          targetVelocities=[target_vel_left, target_vel_right],
                                          forces=[self.max_force, self.max_force])
