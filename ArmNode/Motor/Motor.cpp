#include "mbed.h"
#include "Motor.h"

Motor::Motor(PinName _pwm, PinName _dire, int freq) : pwm(_pwm), dire(_dire)
{
    abs_max_output = 0.95;
    pwm.period(1.0 / freq); 
    pwm.write(0);
    dire = 0;
    pwm_mode = SMB;
}


void Motor::drive(float output)
{
    switch(pwm_mode)
    {
        case SMB:
            if (abs(output) > abs_max_output)
            {
                pwm = 0;
                dire = 0;
            }
            else if (output > 0.01)
            {
                pwm = output * abs_max_output;
                dire = 0;
            }
            else if (output < -0.01)
            {
                pwm = -output * abs_max_output;
                dire = 1;
            }
            else
            {
                pwm = 0;
                dire = 0;
            }
            break;
        case LAP:
            if(abs(output) > abs_max_output)
            {
                pwm = 0;
                dire = 0;
            }
            else if(output > abs_max_output * 0.05)
            {
                pwm = 0.5 + (output * abs_max_output / 2);
                dire = 1;
            }
            else if(output < -abs_max_output * 0.05)
            {
                pwm = 0.5 - (output * abs_max_output / 2);
                dire = 1;
            }
            else
            {
                pwm = 0;
                dire = 0;
            }
            break;
    }
}