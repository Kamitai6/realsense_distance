#ifndef Inertia_H
#define Inertia_H

#include "mbed.h"

#define G 9.80665
#define PI 3.14159265359


class Inertia
{
    public: /*length means half-length*/
        Inertia(float l[3], float i[10], float w[5], float r[3], float* _angle, float t);
        Inertia(float l[3], float i[10], float w[5], float r[3], float* _angle);
        void calculate();
        
        float gravity[4];
        float i_moment[4];
        
    private:
        float length[3], inertia[10], weight[5], ratio[3], delta_t;
        float* angle;
};

#endif