/*
======================================================================
 Balancio-Kit (c) 2021 Linar (UdeSA)
 This code is licensed under MIT license (see LICENSE.txt for details)
======================================================================
*/

#ifndef controllers_h
#define controllers_h

#include "config.h"
#include <EloquentTinyML.h>
#include <deque>

/**
 * Struct datatype to store PID related parameters.
 */
struct controller_data_t
{
    float kp;
    float ki;
    float kd;
};

/**
 * A parent class containing different child controller classes.
 */
class Controller
{
public:
    /*Class Constructor*/
    Controller(int buf_size);

    /**
     * Class method that calculates a new output of the controller
     * based on a new inputted state.
     *
     * @param  {float} current_state : Actual state.
     * @param  {float} target_state  : Target state.
     * @return {std::vector<float>}  : Controller output.
     */
    virtual std::vector<float> update(float current_state, float target_state) = 0;

    /**
     * [static] Initializes a specific type of controller (e.g. PID, RL, etc),
     * with specific parameters..
     *
     * @param  {String} algo                       : Selected algorithm
     * @param  {controller_data_t} controller_data : Struct containing controller specific data.
     */
    static Controller *init_algo(String algo, controller_data_t controller_data);

    /**
     * Method that stores in memory a buffer of N_memory instances of the current state, 
     * the target state, and the outputs.
     *
     */
    void update_buffer(String data_key, float data_value);
    
    /* Variables */
    int buffer_size;
    std::deque<float> pitch_error_buff; 
    std::deque<float> pwm_buff; 
};

class RlSingleInput : public Controller
{
private:
    float error;
    Eloquent::TinyML::TfLite<NUMBER_OF_INPUTS, NUMBER_OF_OUTPUTS, TENSOR_ARENA_SIZE> ml;

public:
    /**
     * Class constructor.
     *
     * The Neural Network (hexadecimal array format) is loaded.
     * The following defines must be specified:
     * NUMBER_OF_INPUTS, NUMBER_OF_OUTPUTS, TENSOR_ARENA_SIZE
     */
    RlSingleInput(void);

    /**
     * Neural Network (trained via Reinforcement Learning) inference, based
     * on the actual state and the desired state.
     *
     * @param  {float} current_state : Current state.
     * @param  {float} target_state  : Target state.
     * @return {std::vector<float>}  : Model output.
     */
    std::vector<float> update(float current_state, float target_state) override;
};

class PID : public Controller
{
private:
    float error;
    float errorSum = 0.0;
    float prev_state = 0.0;
    float kp, ki, kd;
    float sum_constraint;
    int out;

public:
    /**
     * Class constructor.
     *
     * Controller specific parameters are loaded.
     *
     * @param  {float} kp_init             : Proportional parameter.
     * @param  {float} ki_init             : Integral parameter.
     * @param  {float} kd_init             : Derivative parameter.
     * @param  {float} sum_constraint_init : Integral constraint, to avoid overflow.
     */
    PID(float kp_init, float ki_init, float kd_init, float sum_constraint_init);

    /**
     * PID controller update and output.
     *
     * @param  {float} current_state : Current state.
     * @param  {float} target_state  : Target state.
     * @return {std::vector<float>}  : Controller output.
     */
    std::vector<float> update(float current_state, float target_state) override;

    /**
     * Reset PID controller computation.
     * It restarts the integral and derivative components.
     *
     * @param  {float} previous_state : Current state
     */
    void reset(float previous_state);
};

class Clautrol : public Controller
{
private:
    // std::vector<float> e_vec = {-260.8, 473.2, -214.6, -0.08983, 0.0};
    // std::vector<float> u_vec = {-2.0, 1.0, 0.0, 0.0, 0.0};

    // std::vector<float> e_vec = {0.0, -1162.0, 2109.0, -957.0, 0.0};
    // std::vector<float> u_vec = {-2.007, 1.013, -0.006738, 0.0, 0.0};
    
    std::vector<float> e_vec = {-542.7, -104.8, 975.7, 94.65, -443};
    std::vector<float> u_vec = {-0.0667, -1.222, 0.444, 0.444, 0.0};

    // float A = -2000 - (20/LOOP_PERIOD) - 22000 * LOOP_PERIOD;
    // float B = (20/LOOP_PERIOD) - 22000 * LOOP_PERIOD;
    // float C =  - 22000 * LOOP_PERIOD;
    //
    // std::vector<float> e_vec = {A, B, C, C, C, C, C, C, C, C, C, C, C, C, C, C, C, C, C, C};
    // std::vector<float> u_vec = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
public:
    /**
     * Class constructor.
     *
     * Controller specific parameters are loaded.
     *
     */
    Clautrol();

    /**
     * Controller update and output.
     *
     * @param  {float} current_state : Current state.
     * @param  {float} target_state  : Target state.
     * @return {std::vector<float>}  : Controller output.
     */
    std::vector<float> update(float current_state, float target_state) override;
};

#endif
