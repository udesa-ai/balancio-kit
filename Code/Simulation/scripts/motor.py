"""This file implements an accurate motor model.
Source: https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/gym/pybullet_envs/minitaur/envs/motor.py
"""

import numpy as np


MOTOR_VOLTAGE = 7.0             # [6.6, 7.4]
MOTOR_RESISTANCE = 5.3          # +- 0.1
MOTOR_TORQUE_CONSTANT = 0.1859  # +- 0.02
MOTOR_VISCOUS_DAMPING = 0
NUM_MOTORS = 2

BACKLASH = 0.0611           # Radians
DYNAMIC_FRICTION = 0.012    # N.m
STATIC_FRICTION = 0.035     # N.m
VEL_DEADBAND = 0.05         # rad/s


class MotorModel(object):
    """The accurate motor model, which is based on the physics of DC motors.
    A pwm signal in the range of [-1.0, 1.0] is converted to the
    torque.
    The internal motor model takes the following factors into consideration:
    friction, back-EMF voltage and current-torque profile.
    """

    def __init__(self,):
        self._resistance = MOTOR_RESISTANCE
        self._voltage = MOTOR_VOLTAGE
        self._torque_constant = MOTOR_TORQUE_CONSTANT
        self._viscous_damping = MOTOR_VISCOUS_DAMPING

    def set_voltage(self, voltage):
        self._voltage = voltage

    def get_voltage(self):
        return self._voltage

    def set_viscous_damping(self, viscous_damping):
        self._viscous_damping = viscous_damping

    def get_viscous_dampling(self):
        return self._viscous_damping

    def convert_to_torque(self,
                          pwm,
                          true_motor_velocity):
        """Convert the commands (position control or torque control) to torque.
        Args:
          pwm: The pwm signal.
          true_motor_velocity: The true motor velocity. The true velocity is used
            to compute back EMF voltage and viscous damping.
        Returns:
          actual_torque: The torque that needs to be applied to the motor.
        """

        pwm = np.clip(pwm, -1.0, 1.0)
        motor_torque = self._convert_to_torque_from_pwm(pwm, true_motor_velocity)

        # Karnopp friction model
        vel_in_db = (abs(true_motor_velocity) <= VEL_DEADBAND)
        static_friction_torque = vel_in_db * -1 * np.sign(true_motor_velocity) * (np.minimum(abs(motor_torque), (np.ones(NUM_MOTORS)*STATIC_FRICTION)))
        dynamic_friction_torque = np.invert(vel_in_db) * -1 * np.sign(true_motor_velocity) * np.ones(NUM_MOTORS)*DYNAMIC_FRICTION

        friction_torque = static_friction_torque + dynamic_friction_torque

        actual_torque = motor_torque + friction_torque

        # TODO: Implement backlash.
        # Backlash
        # Implementation idea: It might need a new link for the gears. (https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=12134)

        return actual_torque

    def _convert_to_torque_from_pwm(self, pwm, true_motor_velocity):
        """Convert the pwm signal to torque.
        Args:
          pwm: The pulse width modulation.
          true_motor_velocity: The true motor velocity at the current moment. It is
            used to compute the back EMF voltage and the viscous damping.
        Returns:
          actual_motor_torque: The torque that needs to be applied to the motor.
        """

        voltage_net = pwm * self._voltage - (self._torque_constant + self._viscous_damping) * true_motor_velocity
        current = voltage_net / self._resistance
        actual_motor_torque = self._torque_constant * current

        return actual_motor_torque
