#ifndef MOTOR_H
#define MOTOR_H

#include "mbed.h"

#define SMB 0
#define LAP 1

class Motor
{
public:
    Motor(PinName _pwm, PinName _dire, int freq = 10000);
    void drive(float output);
    float abs_max_output;
private:
    PwmOut pwm;
    DigitalOut dire;
    int pwm_mode;
};

#endif