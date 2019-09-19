#ifndef Kinematics_H
#define Kinematics_H

#include "mbed.h"

#define PI 3.141592653589793
#define HORIZONTAL 0
#define VERTICAL 1


class Kinematics {
    public:
        Kinematics(float l[3], float* _angle, float* _cartesian[3], float* _a_vel, float* _c_vel[3], float* _c_acc, float t);
        Kinematics(float l[3], float* _angle, float* _cartesian[3], float* _a_vel, float* _c_vel[3], float* _c_acc);
        void computeKinematics();
        void computeInverseKinematics();
        void computeJacobian();
        void computeInverseJacobian();
        void computeAcceleration();
        void HorizontalVertical(bool type);
        
        float x[3];
        float q[4];
        float dx[3];
        float dq[3];
        float ddq[3];
        bool mode;
        
    private:
        float length[3];
        float* angle;
        float* cartesian[3];
        float* a_velocity;
        float* c_velocity[3];
        float* c_acceleration;
        float delta_t;
        float p_jacobian[3][3];
};

#endif