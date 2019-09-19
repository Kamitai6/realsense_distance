#include "mbed.h"
#include "Inertia.h"


Inertia::Inertia(float l[3], float i[10], float w[5], float r[3], float* _angle, float t)
{
    for (int j = 0; j < 5; j++) {
        inertia[j*2] = i[j*2];
        inertia[j*2+1] = i[j*2+1];
        weight[j] = w[j];
    }
    for (int j = 0; j < 3; j++) {
        length[j] = l[j];
        ratio[j] = r[j];
    }
    angle = _angle;
    delta_t = t;
};

Inertia::Inertia(float l[3], float i[10], float w[5], float r[3], float* _angle)
{
    for (int j = 0; j < 5; j++) {
        inertia[j*2] = i[j*2];
        inertia[j*2+1] = i[j*2+1];
        weight[j] = w[j];
    }
    for (int j = 0; j < 3; j++) {
        length[j] = l[j];
        ratio[j] = r[j];
    }
    angle = _angle;
    delta_t = 0.01;
};


void Inertia::calculate()
{
    float l_base[3] = {sqrt(pow(length[0]*2+length[1]*cos(angle[2]), 2) + pow(length[1]*sin(angle[2]), 2)), 
                       sqrt(pow(length[0]*2+length[1]*2*cos(angle[2]), 2) + pow(length[1]*2*sin(angle[2]), 2)), 
                       sqrt(pow(length[0]*2+length[1]*2*cos(angle[2])+length[2]*cos(angle[2]+angle[3]), 2) + pow(length[1]*2*sin(angle[2])+length[2]*sin(angle[2]+angle[3]), 2))};
    float l_mid = sqrt(pow(length[1]*2+length[2]*cos(angle[3]), 2) + pow(length[2]*sin(angle[3]), 2));
    
    i_moment[0] = (inertia[0] + weight[0] * length[0]*length[0] + 
                   inertia[1] + weight[1] * length[0]*length[0]*4 + 
                   inertia[2] + weight[2] * l_base[0]*l_base[0] + 
                   inertia[3] + weight[3] * l_base[1]*l_base[1] + 
                   inertia[4] + weight[4] * l_base[2]*l_base[2]);
    i_moment[1] = (inertia[5] + weight[0] * length[0]*length[0] + 
                   inertia[6] + weight[1] * length[0]*length[0]*4 + 
                   inertia[7] + weight[2] * l_base[0]*l_base[0] + 
                   inertia[8] + weight[3] * l_base[1]*l_base[1] + 
                   inertia[9] + weight[4] * l_base[2]*l_base[2]);
    i_moment[2] = (inertia[7] + weight[2] * length[1]*length[1] + 
                   inertia[8] + weight[3] * length[1]*length[1]*4 + 
                   inertia[9] + weight[4] * l_mid*l_mid);
    i_moment[3] = (inertia[9] + weight[4] * length[2]*length[2]);
    
    gravity[0] = 0;
    gravity[1] = (weight[0] * G * length[0] * cos(angle[1]) + 
                 weight[1] * G * length[0]*2 * cos(angle[1]) + 
                 weight[2] * G * (length[0]*2 * cos(angle[1]) + length[1] * cos(angle[1]+angle[2])) + 
                 weight[3] * G * (length[0]*2 * cos(angle[1]) + length[1]*2 * cos(angle[1]+angle[2])) + 
                 weight[4] * G * (length[0]*2 * cos(angle[1]) + length[1]*2 * cos(angle[1]+angle[2]) + length[2] * cos(angle[1] + angle[2] + angle[3])));
    gravity[2] = (weight[2] * G * length[1] * cos(angle[1]+angle[2]) + 
                 weight[3] * G * length[1]*2 * cos(angle[1]+angle[2]) + 
                 weight[4] * G * (length[1]*2 * cos(angle[1]+angle[2]) + length[2] * cos(angle[1] + angle[2] + angle[3])));
    gravity[3] = (weight[4] * G * length[2] * cos(angle[1] + angle[2] + angle[3]));
}
