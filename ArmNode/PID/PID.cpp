#include "mbed.h"
#include "PID.h"


/*  p,i,d:gain  t:control cycle  max:max output  type:mode setting  */
PID::PID(float p, float i, float d, float t, float max, float allow_error, float range)
{
    kp = p; ki = i; kd = d; delta_t = t; abs_max_output = max; allowable_error = allow_error; time_range = range;
}

PID::PID(float p, float i, float d, float t, float max, bool type, float allow_e, float range)
{
    kp = p; ki = i; kd = d; delta_t = t; abs_max_output = max; pid_type = type; allowable_error = allow_e; time_range = range;
}

void PID::_reset()
{
    integral = 0;
    pre_error = 0;
}

bool PID::compute()
{    
    float proportion, differential;
    
    error = *target - *sensor;
    
    if(pid_type == FIX_COMMAND) {
        
        proportion   = kp * error;
        integral    += ki * error * delta_t;
        differential = kd * (error - pre_error) / delta_t;
        
        integral = _gurd(integral);
        
        _output = proportion + integral + differential;
    } else if(pid_type == FOLLOW_UP) {
        
        proportion   = kp * (error - pre_error);
        integral     = ki * error * delta_t;
        differential = kd * (proportion - pre_proportion) / kp / delta_t;
        
        _output += proportion + integral + differential;
    }
    output = _gurd(_output);
    
    last_target = *target;
    pre_error = error;
    pre_proportion = proportion;
    
    return _isConvergence();
}

float PID::_gurd(float val)
{
    if(val > abs_max_output)
        return abs_max_output;
    else if(val < -abs_max_output)
        return -1 * abs_max_output;
    else return val;
}

bool PID::_isConvergence()
{
    if(abs(error) <= allowable_error) {
        timer += delta_t;
        if(timer > time_range) return 1;
        else return 0;
    } else {
        timer = 0;
        return 0;
    }
}
