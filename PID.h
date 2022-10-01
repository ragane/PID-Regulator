#ifndef _PID_h
#define _PID_h

#if (ARDUINO >= 100)
    #include "Arduino.h"
#else
    #inlude "WProgram.h"
#endif

class PID
{
    private:
        // Proportional Parameter
        float KP;
        // Integrat Parameter
        float KI;
        // Derivative Parameter
        float KD;
        // Integral error
        float I_error;
        // The minimum and maximum value that can be output
        int min_val, max_val;
        // The last sensor value
        float last_input;
        // The last error value to compute target value
        float last_error;
        
    public:
        // Constructors and deconstructor
        PID();
        PID(float Kp, float Ki, float Kd);
        
        
        // Change PID values
        void tune_PID(float Kp, float Ki, float Kd);
        // Set range of output values
        void set_limits(int MIN_VAL, int MAX_VAL);
        // PID controller       
        float compute(float input_val, float target_val, float dt); 
        
};

#endif
