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

/* Controller Abstract Class*/

Controller::Controller(int buf_size=10) : pitch_error_buff(buf_size, 0.0), pwm_buff(buf_size, 0.0)
 {
    // Initialize buffer arrays
    buffer_size = buf_size;
}

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
    else if (algo.equals("Clautrol"))
    {
        // Clautrol
        return new Clautrol();
    }
    else
    {
        Serial.println("Error loading control algorithm");
    }
}

void Controller::update_buffer(String data_key, float data_value){
    if (data_key.equals("pitch_error")){
        pitch_error_buff.push_front(data_value);
        pitch_error_buff.pop_back();
    }
    else if (data_key.equals("pwm")){
        pwm_buff.push_front(data_value);
        pwm_buff.pop_back();
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
    
    // TODO: puch_back  --> emplace_back
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

/* Clautrol class */

Clautrol::Clautrol() : Controller(5){
}

std::vector<float> Clautrol::update(float current_state, float target_state)
{
    //  u(z)          -260.8 z^-1 + 473.2 z^-2 - 214.6 z^-3 - 0.08983 z^-4
    //  -----   =   -------------------------------------------------------------
    //  e(z)            1 - 2 z^-1 + z^-2 - 9.08e-05 z^-3 + 2.061e-09 z^-4
    
    std::vector<float> output;
    float out = 0.0;

    update_buffer("pitch_error", target_state - current_state);

    // Compute controller

    for (int i=0; i<buffer_size; i++)
    {   
        out = out + (e_vec[i] * pitch_error_buff[i] - u_vec[i] * pwm_buff[i]);
    }
    
    float scale = 3 * 2 * 3.1416;
    out = constrain(out, -scale, scale);
    update_buffer("pwm", float(out));

    out = (out * 255.0)/(scale);

    output.push_back(out);
    return output;
}

