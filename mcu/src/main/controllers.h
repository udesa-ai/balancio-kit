#ifndef controllers_h
#define controllers_h

#include "config.h"
#include <EloquentTinyML.h>

struct controller_data_t
{
    float kp;
    float ki;
    float kd;
};

class Controller
{
public:
    virtual std::vector<float> update(float current_angle, float target_angle) = 0;

    static Controller *init_algo(String algo, controller_data_t controller_data);
};

class RlSingleInput : public Controller
{
private:
    // static int NUMBER_OF_INPUTS = 1;
    float error;
    Eloquent::TinyML::TfLite<NUMBER_OF_INPUTS, NUMBER_OF_OUTPUTS, TENSOR_ARENA_SIZE> ml;

public:
    RlSingleInput(void);
    std::vector<float> update(float current_angle, float target_angle) override;
};

class PID : public Controller
{
private:
    float error;
    float errorSum = 0.0;
    float prev_angle = 0.0;
    float kp, ki, kd;
    float sum_constraint;
    int out;

public:
    PID(float kp_init, float ki_init, float kd_init, float sum_constraint_init);
    std::vector<float> update(float current_angle, float target_angle) override;
    void reset(float previous_angle);
};

#endif
