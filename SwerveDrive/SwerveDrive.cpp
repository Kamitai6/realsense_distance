#include "mbed.h"
#include "SwerveDrive.h"
#define PI 3.141593


SwerveDrive::SwerveDrive(int num, float r)
{
    wheel_num = num;
    radius = r;
    
    for(int i; i < wheel_num; i++)
    {
        switch(wheel_num)
        {
            case 3:
                angle[i] = PI * (i * 4 + 1) / 6;
                break;
            case 4:
                angle[i] = PI * (i * 2 + 1) / 4;
                break;
        }
    }
}


void SwerveDrive::setVelocity(float _vel[])
{
    //local
    /*for(int i; i < 3; i++)
        robot_vel[i] = _vel[i];*/
    //global
    robot_vel[0] = vel[0];
    robot_vel[1] = vel[1];
    robot_vel[2] = vel[2];
}


void SwerveDrive::computeWheels()
{
    float max_vel;
    float abs_max_vel = 0.95;
    
    for(int i; i < wheel_num; i++)
    {
        wheel_vector[0] = robot_vel[0] - radius * robot_vel[2] * sin(angle[i]);
        wheel_vector[1] = robot_vel[1] + radius * robot_vel[2] * cos(angle[i]);
        //velocity
        wheel_vel[i] = sqrt(wheel_vector[0] * wheel_vector[0] + wheel_vector[1] * wheel_vector[1]);
        //angle
        wheel_angle[i] = atan2(wheel_vector[1], wheel_vector[0]) / PI * 180;
        
        //rescale
        if(max_vel < abs(wheel_vel[i]))
            max_vel = abs(wheel_vel[i]);
        if(max_vel > abs_max_vel)
            for(int i; i < wheel_num; i++)
                wheel_vel[i] *= abs_max_vel / max_vel;
        
        for(int i; i < wheel_num; i++)
            if(wheel_angle[i] < 0)
                wheel_angle[i] += 360;
    }
}