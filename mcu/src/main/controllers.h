#ifndef controllers_h
#define controllers_h

#include "config.h"
#include <EloquentTinyML.h>


/* Neural network related variables */
#define NUMBER_OF_INPUTS 1
#define NUMBER_OF_OUTPUTS 2
// You may need to tweak this value. It's a trial and error process
#define TENSOR_ARENA_SIZE 16*1024
#define RL_INPUT_NORMALIZATION 0.06519447  //1.5708  // 0.06519447


class Controller
{   public:
        virtual void update(float current_angle, float target_angle) = 0;
        
        static Controller* init_algo(String algo);
};


class RlSingleInput: public Controller
{
private:
    float error;
    Eloquent::TinyML::TfLite<NUMBER_OF_INPUTS, NUMBER_OF_OUTPUTS, TENSOR_ARENA_SIZE> ml;
    
public:
    int pwmL, pwmR;
    RlSingleInput(void);
    void update(float current_angle, float target_angle) override;
};


class PID: public Controller
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
    void update(float current_angle, float target_angle) override;
};

#endif
