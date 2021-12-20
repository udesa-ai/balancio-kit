"""This file implements an accurate motor model.
Source: https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/gym/pybullet_envs/minitaur/envs/motor.py
"""

import numpy as np


MOTOR_VOLTAGE = 28.0             # 40*Kg.Dm²/A.(10.s)³       # 7.0 -> [6.6, 7.4]     V : Kg.M²/A.s³
MOTOR_RESISTANCE = 21.2          # 40*Kg.Dm²/A².(10.s)³      # 5.3 +- 0.1            Ohm : Kg.M²/A².s³
MOTOR_TORQUE_CONSTANT = 7.436    # 40*Kg.Dm²/A.(10.s)²       # 0.1859 +- 0.02        N.m/A : Kg.M²/A.s²
MOTOR_EMF_CONSTANT = 7.436       # 40*Kg.Dm²/A.(10.s)²       # 0.1859 +- 0.02        V.s/rad : Kg.M²/A.s²
MOTOR_VISCOUS_DAMPING = 0
NUM_MOTORS = 2

BACKLASH = 0.0611                # Radians
DYNAMIC_FRICTION = 0.48          # 40*Kg.Dm²/s²             # 0.012 N.m : Kg.M²/s²
STATIC_FRICTION = 1.40           # 40*Kg.Dm²/s²             # 0.035 N.m : Kg.M²/s²
VEL_DEADBAND = 0.3               # rad/(10*s)               # 3 rad/s


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
        self._EMF_constant = MOTOR_EMF_CONSTANT
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
          static_friction_torque: Max static friction applied in joint. Only applied if velocity is in dead-band range.
        """

        pwm = np.clip(pwm, -1.0, 1.0)
        motor_torque = self._convert_to_torque_from_pwm(pwm, true_motor_velocity)

        # Karnopp friction model
        vel_in_db = (abs(true_motor_velocity) <= VEL_DEADBAND)
        # static_friction_torque = vel_in_db * -1 * np.sign(true_motor_velocity) * (np.minimum(abs(motor_torque), (np.ones(NUM_MOTORS)*STATIC_FRICTION)))
        static_friction_torque = vel_in_db * STATIC_FRICTION
        dynamic_friction_torque = np.invert(vel_in_db) * -1 * np.sign(true_motor_velocity) * np.ones(NUM_MOTORS)*DYNAMIC_FRICTION

        friction_torque = dynamic_friction_torque  # + static_friction_torque

        actual_torque = motor_torque + friction_torque

        return actual_torque, static_friction_torque

    def _convert_to_torque_from_pwm(self, pwm, true_motor_velocity):
        """Convert the pwm signal to torque.
        Args:
          pwm: The pulse width modulation.
          true_motor_velocity: The true motor velocity at the current moment. It is
            used to compute the back EMF voltage and the viscous damping.
        Returns:
          actual_motor_torque: The torque that needs to be applied to the motor.
        """

        voltage_net = pwm * self._voltage - (self._EMF_constant + self._viscous_damping) * true_motor_velocity
        current = voltage_net / self._resistance
        actual_motor_torque = self._torque_constant * current

        return actual_motor_torque
