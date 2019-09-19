#ifndef PID_H
#define PID_H

#include "mbed.h"

#define FIX_COMMAND 0
#define FOLLOW_UP 1


//PID calculator
/**/

class PID
{
public:
    PID(float p, float i, float d, float t, float max, float allow_e, float range);
    PID(float p, float i, float d, float t, float max, bool type, float allow_e, float range);
    
    bool compute();
    
    float output; //result
    float *sensor, *target; //sensor value pointer & target value pointer

private:
    float kp, ki, kd, delta_t, abs_max_output, time_range;
    bool pid_type;
    
    float timer;
    float _output;
    float integral;
    float error, pre_error;
    float pre_proportion;
    float last_target;
    double start_time;
    float allowable_error;
    
    bool _isConvergence();
    void _reset();
    float _gurd(float val);
};
#endif
