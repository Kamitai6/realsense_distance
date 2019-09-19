#include "Kinematics.h"


Kinematics::Kinematics(float l[3], float* _angle, float* _cartesian[3], float* _a_vel, float* _c_vel[3], float* _c_acc, float t)
{
    for (int i = 0; i < 3; i++) {
        length[i] = l[i]*2;
        cartesian[i] = _cartesian[i];
        c_velocity[i] = _c_vel[i];
    }
    angle = _angle;
    a_velocity = _a_vel;
    c_acceleration = _c_acc;
    delta_t = t;
    mode = HORIZONTAL;
};

Kinematics::Kinematics(float l[3], float* _angle, float* _cartesian[3], float* _a_vel, float* _c_vel[3], float* _c_acc)
{
    for (int i = 0; i < 3; i++) {
        length[i] = l[i]*2;
        cartesian[i] = _cartesian[i];
        c_velocity[i] = _c_vel[i];
    }
    angle = _angle;
    a_velocity = _a_vel;
    c_acceleration = _c_acc;
    delta_t = 0.01;
    mode = HORIZONTAL;
};

void Kinematics::computeKinematics()
{
    x[0] = (length[1] * cos(angle[1]) + length[2] * cos(angle[1] + angle[2])) * cos(angle[0]);
    x[1] = (length[1] * cos(angle[1]) + length[2] * cos(angle[1] + angle[2])) * sin(angle[0]);
    x[2] = length[1] * sin(angle[1]) + length[2] * sin(angle[1] + angle[2]) + length[0];
}

void Kinematics::computeInverseKinematics()
{
    float xi = sqrt(*cartesian[0] * *cartesian[0] + *cartesian[1] * *cartesian[1]);
    float eta = *cartesian[2] - length[0];
    
    q[2] = -acos(xi*xi + eta*eta - length[1]*length[1] - length[2]*length[2]);
    q[1] = atan2(eta, xi) - atan2(length[2]*sin(q[2]), length[1] + length[2] * cos(q[2]));
    q[0] = atan2(*cartesian[1], *cartesian[0]);
}

void Kinematics::computeJacobian()
{
    float c = length[2] * cos(angle[1] + angle[2]);
    float d = length[2] * sin(angle[1] + angle[2]);
    float a = length[1] * cos(angle[1]) + c;
    float b = length[1] * sin(angle[1]) + d;
    float jacobian[3][3] = {{-1 * a * sin(angle[0]),    -1 * b * cos(angle[0]), -1 * d * cos(angle[0])},
                            {a * cos(angle[0]),         -1 * b * sin(angle[0]), -1 * d * sin(angle[0])},
                            {0,                         a,                      c}};
    
    for (int i = 0; i < 3; i++)
        dx[i] = jacobian[i][0] * a_velocity[0] + jacobian[i][1] * a_velocity[1] + jacobian[i][2] * a_velocity[2];
}

void Kinematics::computeInverseJacobian()
{
    float c = length[2] * cos(angle[1] + angle[2]);
    float d = length[2] * sin(angle[1] + angle[2]);
    float a = length[1] * cos(angle[1]) + c;
    float b = length[1] * sin(angle[1]) + d;
    
    float jacobian[3][3] = {{-1 * sin(angle[0]) / a,                    cos(angle[0]) / a,                         0},
                            {c * cos(angle[0]) / (a * d - b * c),       c * sin(angle[0]) / (a * d - b * c),       d / (a * d - b * c)},
                            {-1 * a * cos(angle[0]) / (a * d - b * c),  -1 * a * sin(angle[0]) / (a * d - b * c),  -1 * b / (a * d - b * c)}};
    
    for (int i = 0; i < 3; i++)
        dq[i] = jacobian[i][0] * *c_velocity[0] + jacobian[i][1] * *c_velocity[1] + jacobian[i][2] * *c_velocity[2];
}

void Kinematics::computeAcceleration()
{
    float ddx[3] = {};
    
    float c = length[2] * cos(angle[1] + angle[2]);
    float d = length[2] * sin(angle[1] + angle[2]);
    float a = length[1] * cos(angle[1]) + c;
    float b = length[1] * sin(angle[1]) + d;
    
    float jacobian[3][3] = {{-1 * a * sin(angle[0]),    -1 * b * cos(angle[0]), -1 * d * cos(angle[0])},
                            {a * cos(angle[0]),         -1 * b * sin(angle[0]), -1 * d * sin(angle[0])},
                            {0,                         a,                      c}};
    float i_jacobian[3][3] = {{-1 * sin(angle[0]) / a,                    cos(angle[0]) / a,                         0},
                            {c * cos(angle[0]) / (a * d - b * c),       c * sin(angle[0]) / (a * d - b * c),       d / (a * d - b * c)},
                            {-1 * a * cos(angle[0]) / (a * d - b * c),  -1 * a * sin(angle[0]) / (a * d - b * c),  -1 * b / (a * d - b * c)}};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            ddx[i] += ((jacobian[i][j] - p_jacobian[i][j]) / delta_t) * *c_velocity[j];
            p_jacobian[i][j] = jacobian[i][j];
        }
    }
    for (int i = 0; i < 3; i++)
        ddq[i] = i_jacobian[i][0] * (c_acceleration[0] - ddx[0]) + i_jacobian[i][1] * (c_acceleration[1] - ddx[1]) + i_jacobian[i][2] * (c_acceleration[2] - ddx[2]);
}

void Kinematics::HorizontalVertical(bool type)
{
    mode = type;
    if(mode) q[3] = angle[1] + angle[2] + PI * 0.5;
    else q[3] = angle[1] + angle[2];
}

