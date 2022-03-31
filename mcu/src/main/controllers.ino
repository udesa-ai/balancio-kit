#include "config.h"
#include "controllers.h"
#include "rl_model.h"
#include <EloquentTinyML.h>
#include <vector>


/* Controller Abstract Class*/

Controller* Controller::init_algo(String algo, controller_data_t controller_data) {
    // Control algorithm initialization
    if (algo.equals("PID")){
        // PID
        return new PID(controller_data.kp, controller_data.ki, controller_data.kd, 5.0);
    }
    else if (algo.equals("RL")){
        // RL
        return new RlSingleInput();
    }
    else{
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

std::vector<float> RlSingleInput::update(float current_angle, float target_angle)
{   
    std::vector<float> output;

    // Input Normalization (for neural net compatibility). 
    // WARNING: This must be the same normalization used during training.
    error = current_angle - target_angle;
    error /= RL_INPUT_NORMALIZATION;
    error = constrain(error, -1, 1);
    
    float input[1] = { error };
    float predicted[2]= {0};
    ml.predict(input, predicted);

    output.push_back(predicted[0]);
    output.push_back(predicted[1]);

    return output;
}


/*PID Class*/

PID::PID(float kp_init, float ki_init, float kd_init, float sum_constraint_init)
{
    kp = kp_init;
    ki = ki_init;
    kd = kd_init;
    sum_constraint = sum_constraint_init;  // 5
}

std::vector<float> PID::update(float current_angle, float target_angle)
{   
    std::vector<float> output;

    error = current_angle - target_angle;
    errorSum += error;  
    errorSum = constrain(errorSum, -sum_constraint, sum_constraint);
    out = kp*(error) + ki*(errorSum)*LOOP_PERIOD + kd*(current_angle-prev_angle)/LOOP_PERIOD;
    prev_angle = current_angle;

    output.push_back(out);
    return output;
}
