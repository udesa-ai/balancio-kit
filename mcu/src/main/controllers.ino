#include "config.h"
#include "controllers.h"
#include "rl_model.h"
#include <EloquentTinyML.h>


/*RL Single Input Class*/

RlSingleInput::RlSingleInput(void)
{
    /* Neural Network */
    ml = Eloquent::TinyML::TfLite<NUMBER_OF_INPUTS, NUMBER_OF_OUTPUTS, TENSOR_ARENA_SIZE>();
    ml.begin(rl_model);
}

void RlSingleInput::update(float current_angle, float target_angle)
{
    // Input Normalization (for neural net compatibility). 
    // WARNING: This must be the same normalization used during training.
    error = current_angle - target_angle;
    error /= RL_INPUT_NORMALIZATION;
    error = constrain(error, -1, 1);
    
    float input[1] = { error };
    float predicted[2]= {0};
    ml.predict(input, predicted);
    pwmL = (predicted[0] * 255.0);
    pwmL = constrain(pwmL, -255, 255);
    pwmR = (predicted[1] * 255.0);
    pwmR = constrain(pwmR, -255, 255);

}


/*PID Class*/

PID::PID(float kp_init, float ki_init, float kd_init, float sum_constraint_init, float output_constraint_init)
{
    kp = kp_init;
    ki = ki_init;
    kd = kd_init;
    sum_constraint = sum_constraint_init;  // 5
    output_constraint = output_constraint_init;   // 255
}

void PID::update(float current_angle, float target_angle)
{
    error = current_angle - target_angle;
    errorSum += error;  
    errorSum = constrain(errorSum, -sum_constraint, sum_constraint);
    output = kp*(error) + ki*(errorSum)*LOOP_PERIOD + kd*(current_angle-prev_angle)/LOOP_PERIOD;
    output = constrain(output, -output_constraint, output_constraint);
    prev_angle = current_angle;

}