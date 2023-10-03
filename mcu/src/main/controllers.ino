/* 
======================================================================
 Balancio-Kit (c) 2021 Linar (UdeSA)
 This code is licensed under MIT license (see LICENSE.txt for details)
====================================================================== 
*/

#include "config.h"
#include "controllers.h"
#include "rl_model.h"
#include <EloquentTinyML.h>
#include <vector>
#include "Arduino.h"

/* Controller Abstract Class*/

Controller *Controller::init_algo(String algo, controller_data_t controller_data)
{
    // Control algorithm initialization
    if (algo.equals("PID"))
    {
        // PID
        return new PID(controller_data.kp, controller_data.ki, controller_data.kd, 5.0);
    }
    else if (algo.equals("RL"))
    {
        // RL
        return new RlSingleInput();
    }
    else
    {
        Serial.println("Error loading control algorithm");
    }
}

/*RL Single Input Class*/

RlSingleInput::RlSingleInput(void)
{
    /* Neural Network */
    ml = Eloquent::TinyML::TfLite<NUMBER_OF_INPUTS, NUMBER_OF_OUTPUTS, TENSOR_ARENA_SIZE>();
    ml.begin(rl_model);
}

std::vector<float> RlSingleInput::update(float current_state, float target_state)
{
    std::vector<float> output;

    // Input Normalization (for neural net compatibility).
    // WARNING: This must be the same normalization used during training.
    error = current_state - target_state;
    error /= RL_INPUT_NORMALIZATION;
    error = constrain(error, -1, 1);

    float input[1] = {error};
    float predicted[2] = {0};
    ml.predict(input, predicted);

    output.push_back(RL_OUTPUT_DENORMALIZATION * predicted[0]);
    output.push_back(RL_OUTPUT_DENORMALIZATION * predicted[1]);

    return output;
}

/*PID Class*/

PID::PID(float kp_init, float ki_init, float kd_init, float sum_constraint_init)
{
    kp = kp_init;
    ki = ki_init;
    kd = kd_init;
    sum_constraint = sum_constraint_init; 
}

std::vector<float> PID::update(float current_state, float target_state)
{
    std::vector<float> output;

    error = current_state - target_state;
    errorSum += error;
    errorSum = constrain(errorSum, -sum_constraint, sum_constraint);
    out = kp * (error) + ki * (errorSum)*LOOP_PERIOD + kd * (current_state - prev_state) / LOOP_PERIOD;
    prev_state = current_state;

    output.push_back(out);
    return output;
}

void PID::reset(float previous_state)
{
    errorSum = 0.0;
    prev_state = previous_state;
}
