#include "Arduino.h"
#include "PID_h"


PID::PID()
{
    PID::tune_PID(0, 0, 0);
    PID::set_limits(0, 255);
    
    this->last_error = 0;
    this->last_input = 0;
    this->I_error = 0;

PID::PID(float Kp, float Ki,  Kd)
{
    PID::tune_PID(Kp, Ki, Kd)
    PID::set_limits(0, 255);
    
    this->last_error = 0;
    this->last_input = 0;
    this->I_error = 0;
}

PID::tune_PID(float Kp, float Ki, float Kd)
{
    this->KP = Kp;
    this->KP = Ki;
    this->KD = Kd;
}

PID::set_limits(float MIN_VAL, float MAX_VAL)
{
    if (MIN_VAL >= MAX_VAL)
        MIN_VAL = -MIN_VAL;

    if (MAX_VAL <= MIN_VAL)
        MAX_VAL = -MAX_VAL;        
}

PID::compute(float input_val, float target_val, float dt
{
    float error = target_val - input_val;

    // Calculate integral term
    I_error = error * dt;

    // Calculate derivative term
    float D_error = (last_input - input_val) / dt;

    // Store error and output values for feedback
    this->last_input = input_val;
    this->last_error = error;

    // 
    float result = (this->KP * error) + (this->KI * I_error) + (this->KD * D_error);
  
    result = result > max_val ? max_val : result;
    result = result < min_val ? min_val : result; 
    return result;
}
