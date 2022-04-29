# ======================================================================
#  Balancio-Kit (c) 2021 Linar (UdeSA)
#  This code is licensed under MIT license (see LICENSE.txt for details)
# ======================================================================


"""
This module implements the Balancio robot in a PyBullet simulation.
"""

import numpy as np
from ..robot import motor
import os

URDF_PATH = os.path.join('/', *os.path.dirname(os.path.realpath(__file__)).split('/')[:-2], 'urdf', 'balancio_v3.urdf')


class Balancio:

    def __init__(self, bullet_client, time_step=1 / 240, backlash=True):
        """Class constructor.
        @param bullet_client: Pybullet client.
        @param time_step: Period of each timestep.
        @param backlash: Whether to apply backlash to the motors or not.
        """
        self.time_step = time_step  # Intentar dejarlo en el estandar (240hz)
        self._p = bullet_client

        self.robotUniqueId = None
        self.backlash = backlash
        self.motors_num = 2
        self.joint_name2idx = {'left_gearbox': 1,
                               'left_wheel': 2,
                               'right_gearbox': 3,  # 3 for original 4 for debug urdf.
                               'right_wheel': 4,  # 4 for original 5 for debug urdf.
                               'imu': 0}
        self.gravity = 0.981  # Gravity: 0.981 dm/(10.s)^2
        self.previous_linear_vel = np.array([0, 0, 0])
        self.linear_accel = [0, 0, 0]
        self.motors = motor.MotorModel()
        self.reset()

    def reset(self):
        """Respawns the Balancio in the simulation.

        Resets location and orientation with random tilt.
        """
        # Randomize initial orientation.
        self.orientation_init_pitch = np.random.uniform(-0.1, 0.1)
        orientation_init = self._p.getQuaternionFromEuler([0, self.orientation_init_pitch, 0])
        robot = self._p.loadURDF(URDF_PATH,
                                 [0, 0, 0.8],
                                 orientation_init,
                                 useFixedBase=False,
                                 flags=self._p.URDF_USE_INERTIA_FROM_FILE)
        self.robotUniqueId = robot

        # This seems to decrease noise in acceleration, smoothening contact forces. (Empirical values)
        self._p.changeDynamics(self.robotUniqueId, self.joint_name2idx['left_wheel'], lateralFriction=1,
                               rollingFriction=0.005, contactDamping=300, contactStiffness=800)
        self._p.changeDynamics(self.robotUniqueId, self.joint_name2idx['right_wheel'], lateralFriction=1,
                               rollingFriction=0.005, contactDamping=300, contactStiffness=800)

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
                                              forces=[0, 0])  # Here you can add additional backlash friction.

    def get_action_dimension(self) -> int:
        """Get the number of actuators present in the robot.

        @return: Number of actuators.
        """
        return self.motors_num

    def get_pitch(self) -> list:
        """Get the actual pitch of the robot.

        @return: Pitch.
        """
        pitch = []
        pos, orn = self._p.getBasePositionAndOrientation(self.robotUniqueId)

        orn_euler = self._p.getEulerFromQuaternion(orn)  # [Roll, Pitch, Yaw]
        pitch.extend([orn_euler[1]])
        return pitch

    def get_yaw(self) -> list:
        """Get the actual yaw of the robot.

        @return: Yaw.
        """
        yaw = []
        pos, orn = self._p.getBasePositionAndOrientation(self.robotUniqueId)

        orn_euler = self._p.getEulerFromQuaternion(orn)  # [Roll, Pitch, Yaw]
        yaw.extend([orn_euler[2]])
        return yaw

    def get_angular_vel(self) -> list:
        """Get the angular velocity relative to the robot's IMU coordinates.

        angular_vel_imu = [omega_x, omega_y, omega_z]

        @return: List containing the angular velocity in the 3 IMU axis.
        """
        # Get angular velocity relative to global frame, in cartesian global coordinates.
        angular_vel_b = np.array(self._p.getBaseVelocity(self.robotUniqueId)[1])

        # Transform angular velocity, to base coordinates.
        _, orn = self._p.getBasePositionAndOrientation(self.robotUniqueId)
        rot_matrix = self._p.getMatrixFromQuaternion(orn)
        rot_matrix_b2r = np.array([rot_matrix[:3], rot_matrix[3:6], rot_matrix[6:9]])
        rot_matrix_r2b = np.linalg.inv(rot_matrix_b2r)
        angular_vel_r = np.matmul(rot_matrix_r2b, angular_vel_b)

        # Transform angular velocity to real IMU coordinates (based on orientation on real robot).
        angular_vel_imu = [-angular_vel_r[1], angular_vel_r[0], angular_vel_r[2]]

        return angular_vel_imu

    def linear_accel_update(self):
        """Calculate the linear acceleration of the IMU link.

        The linear acceleration is computed based on the derivative of the linear velocity,
        and stored in attribute "linear_accel".

        (!) Warning: This method should be called once per simulation step.
        """
        # Get linear velocity of base relative to global frame, in cartesian global coordinates.
        # linear_vel_b = np.array(self._p.getBaseVelocity(self.robotUniqueId)[0])
        # Get linear velocity of imu relative to global frame, in cartesian global coordinates.
        linear_vel_b = self._p.getLinkState(self.robotUniqueId, self.joint_name2idx['imu'], computeLinkVelocity=True)[
            -2]

        # Transform linear velocity, to base coordinates.
        _, orn = self._p.getBasePositionAndOrientation(self.robotUniqueId)
        rot_matrix = self._p.getMatrixFromQuaternion(orn)
        rot_matrix_b2r = np.array([rot_matrix[:3], rot_matrix[3:6], rot_matrix[6:9]])
        rot_matrix_r2b = np.linalg.inv(rot_matrix_b2r)
        linear_vel_r = np.matmul(rot_matrix_r2b, linear_vel_b)

        # Calculate linear acceleration, in base coordinates.
        linear_accel_r = (linear_vel_r - self.previous_linear_vel) / self.time_step
        self.previous_linear_vel = linear_vel_r
        # Gravity addition for IMU simulation
        linear_accel_r += np.matmul(rot_matrix_r2b, np.array([0, 0, self.gravity]))

        # Transform linear acceleration to real IMU coordinates (based on orientation on real robot).
        self.linear_accel[0] = -linear_accel_r[1]  # IMU X = SIM -Y
        self.linear_accel[1] = linear_accel_r[0]  # IMU Y = SIM  X
        self.linear_accel[2] = linear_accel_r[2]  # IMU Z = SIM  Z

    def get_linear_accel(self) -> list:
        """Get the linear acceleration of the IMU link.

        @return: Linear acceleration.
        """
        return self.linear_accel

    def linear_accel_reset(self):
        """ Resets the previous_linear_vel attribute.
        """
        self.previous_linear_vel = np.array([0, 0, 0])

    def apply_action(self, motor_commands: list):
        """ Apply the commanded PWM to each actuator.

        @param motor_commands: List containing pwm signal for each motor.
                               The size of the list has to be the same as the number of actuators ("motors_num").
                               PWM range --> [-1, 1]
        """
        state = self._p.getJointStates(bodyUniqueId=self.robotUniqueId,
                                       jointIndices=[self.joint_name2idx['left_gearbox'],
                                                     self.joint_name2idx['right_gearbox']])

        torque, static_friction = self.motors.convert_to_torque(np.array(motor_commands),
                                                                np.array([state[0][1], state[1][1]]))

        # Set static friction (set to 0 when dynamic friction starts acting).
        self._p.setJointMotorControlArray(bodyUniqueId=self.robotUniqueId,
                                          jointIndices=[self.joint_name2idx['left_gearbox'],
                                                        self.joint_name2idx['right_gearbox']],
                                          controlMode=self._p.VELOCITY_CONTROL,
                                          targetVelocities=[0, 0],
                                          forces=[static_friction[0], static_friction[1]])
        # Set torque
        self._p.setJointMotorControlArray(bodyUniqueId=self.robotUniqueId,
                                          jointIndices=[self.joint_name2idx['left_gearbox'],
                                                        self.joint_name2idx['right_gearbox']],
                                          controlMode=self._p.TORQUE_CONTROL,
                                          forces=[torque[0], torque[1]])
