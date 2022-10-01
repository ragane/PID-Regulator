#include "PID.h"

#if (ARDUINO >= 100)
    #include "Arduino.h"
#else
    #include "WProgram.h"
#endif

PID::PID()
{
    PID::tune_PID(0, 0, 0);
    PID::set_limits(0, 255);
    
    this->last_error = 0;
    this->last_input = 0;
    this->I_error = 0;
}

PID::PID(float Kp, float Ki, float Kd)
{
    PID::tune_PID(Kp, Ki, Kd);
    PID::set_limits(0, 255);
    
    this->last_error = 0;
    this->last_input = 0;
    this->I_error = 0;
}

void PID::tune_PID(float Kp, float Ki, float Kd)
{
    this->KP = Kp;
    this->KP = Ki;
    this->KD = Kd;
}

void PID::set_limits(int MIN_VAL, int MAX_VAL)
{
    if (MIN_VAL >= MAX_VAL)
        MIN_VAL = -MIN_VAL;

    if (MAX_VAL <= MIN_VAL)
        MAX_VAL = -MAX_VAL;        
}

float PID::compute(float input_val, float target_val, float dt)
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
  
    result = result > (float)max_val ? (float)max_val : result;
    result = result < (float)min_val ? (float)min_val : result; 
    return result;
}
