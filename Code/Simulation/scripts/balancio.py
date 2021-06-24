""" Balancio: Pybullet self-balancing robot"""

import numpy as np
import motor


class Balancio:

    def __init__(self, bullet_client, urdf_root_path='', time_step=0.01, backlash=True):
        self.urdf_root_path = urdf_root_path  # Usar para mnodificar el path del URDF a uno relativo (?)
        self.time_step = time_step  # Intentar dejarlo en el estandar (240hz)
        self._p = bullet_client

        self.robotUniqueId = None
        self.backlash = backlash
        self.max_vel = 10  # rad/s
        self.max_force = 20
        self.motors_num = 2
        self.joint_name2idx = {'left_gearbox': 0,
                               'left_wheel': 1,
                               'right_gearbox': 2,
                               'right_wheel': 3}
        self.motors = motor.MotorModel()
        self.reset()

    def reset(self):
        robot = self._p.loadURDF("/home/agus/Documents/UdeSA/Balancio_V0/Code/Simulation/urdf/balancio_v1.urdf",
                                 [0, 0, 0.6],
                                 useFixedBase=False)
        self.robotUniqueId = robot

        # Disable default velocity control (Necessary for torque control)
        self._p.setJointMotorControlArray(bodyUniqueId=self.robotUniqueId,
                                          jointIndices=[self.joint_name2idx['left_gearbox'],
                                                        self.joint_name2idx['right_gearbox']],
                                          controlMode=self._p.VELOCITY_CONTROL,
                                          forces=[0, 0])
        if self.backlash:
            # Loose contact between gear and wheel
            self._p.setJointMotorControlArray(bodyUniqueId=self.robotUniqueId,
                                              jointIndices=[self.joint_name2idx['left_wheel'],
                                                            self.joint_name2idx['right_wheel']],
                                              controlMode=self._p.VELOCITY_CONTROL,
                                              targetVelocities=[0, 0],
                                              forces=[0, 0])
        # ## DEBUG
        # self._p.enableJointForceTorqueSensor(1, 0, 1)
        # self._p.enableJointForceTorqueSensor(1, 1, 1)
        # self._p.enableJointForceTorqueSensor(1, 2, 1)
        # self._p.enableJointForceTorqueSensor(1, 3, 1)

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
        """Args:
            motor_commands: List containing pwm signal for each motor.
                            PWM range --> [-1, 1]"""

        state = self._p.getJointStates(bodyUniqueId=self.robotUniqueId,
                                       jointIndices=[self.joint_name2idx['left_gearbox'], self.joint_name2idx['right_gearbox']])

        torque = self.motors.convert_to_torque(np.array(motor_commands), np.array([state[0][1], state[1][1]]))

        # Set torque
        self._p.setJointMotorControlArray(bodyUniqueId=self.robotUniqueId,
                                          jointIndices=[self.joint_name2idx['left_gearbox'], self.joint_name2idx['right_gearbox']],
                                          controlMode=self._p.TORQUE_CONTROL,
                                          forces=[torque[0], torque[1]])

        # ## DEBUG
        # self._p.applyExternalTorque(1, 0, (0, 0, 3), self._p.LINK_FRAME)
        # tst = self._p.getJointStates(bodyUniqueId=self.robotUniqueId,
        #                                jointIndices=[0, 1, 2, 3])
        # print(tst[0][2][2])

