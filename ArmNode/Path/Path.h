#ifndef PATH_H
#define PATH_H

#include "mbed.h"
#include <math.h>

using namespace std;


class Path
{
    public:
        Path(float* _y, int _num);
        void InitParam();
        void Calc(float t);
        
        float f;
        float df;
        float ddf;
    
    private:
        float* y;
        int num;
        float a_[10];
        float b_[10];
        float c_[10];
        float d_[10];
        
        void _gurd();
};

#endif