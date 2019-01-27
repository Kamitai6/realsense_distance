#ifndef SWERVE_H
#define SWERVE_H

#include "mbed.h"

class SwerveDrive
{
    public:
        SwerveDrive(int num, float r);
        float robot_vel[3];
        float *imu_yow;
        float wheel_vector[2];
        float wheel_vel[4];
        float wheel_angle[4];
        
        void setVelocity(float[3]);
        void computeWheels();
        
    private:
        int wheel_num;
        float radius;
        float angle[4];
};

#endif