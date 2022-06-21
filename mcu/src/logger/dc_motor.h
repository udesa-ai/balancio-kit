/*
======================================================================
 Balancio-Kit (c) 2021 Linar (UdeSA)
 This code is licensed under MIT license (see LICENSE.txt for details)
======================================================================
*/

// Defines
#define IN1 18
#define IN2 19
#define ENA 17

#define IN3 0
#define IN4 2
#define ENB 4

// Global Function Protoyes
/**
 * Configure microcontroller PWM.
 */
void motor_init(void);

/**
 * Stop both motors.
 */
void stop_motor(void);

/**
 * Set the left motor speed in foward direction.
 *
 * @param  {int} pwm : Motor PWM. Values in range [0, 255]
 */
void fwd_L_motor(int pwm);

/**
 * Set the right motor speed in foward direction.
 *
 * @param  {int} pwm : Motor PWM. Values in range [0, 255]
 */
void fwd_R_motor(int pwm);

/**
 * Set the left motor speed in backward direction.
 *
 * @param  {int} pwm : Motor PWM. Values in range [0, 255]
 */
void bwd_L_motor(int pwm);

/**
 * Set the right motor speed in backward direction.
 *
 * @param  {int} pwm : Motor PWM. Values in range [0, 255]
 */
void bwd_R_motor(int pwm);

/**
 * Set the right motor speed and direction.
 *
 * @param  {int} pwm : Motor PWM. Values in range [-255, 255].
 *                     Positive (+) values --> Foward
 *                     Negative (-) values --> Backward
 */
void R_motor(int pwm);

/**
 * Set the left motor speed and direction.
 *
 * @param  {int} pwm : Motor PWM. Values in range [-255, 255].
 *                     Positive (+) values --> Foward
 *                     Negative (-) values --> Backward
 */
void L_motor(int pwm);
