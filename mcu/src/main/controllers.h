#ifndef controllers_h
#define controllers_h

#include "config.h"
#include <EloquentTinyML.h>


// /* Neural network related variables */
// #define NUMBER_OF_INPUTS 1
// #define NUMBER_OF_OUTPUTS 2
// // You may need to tweak this value. It's a trial and error process
// #define TENSOR_ARENA_SIZE 16*1024

// class controllers
// {
// private:
//     float error;
//     float errorSum = 0.0;
//     float prev_angle = 0.0;
//     int pwm;
//     Eloquent::TinyML::TfLite<NUMBER_OF_INPUTS, NUMBER_OF_OUTPUTS, TENSOR_ARENA_SIZE> ml;
    
// public:
//     int pwmL, pwmR;
//     controllers(void);
//     void pid(float current_angle, float target_angle);
//     void rl(float current_angle, float target_angle);
// };

/* -------------------------------------------------- */

class PID
{
private:
    float error;
    float errorSum = 0.0;
    float prev_angle = 0.0;
    float kp, ki, kd;
    float sum_constraint, output_constraint;
    
public:
    int output;
    PID(float kp_init, float ki_init, float kd_init, float sum_constraint_init, float output_constraint_init);
    void update(float current_angle, float target_angle);
};

#endif
