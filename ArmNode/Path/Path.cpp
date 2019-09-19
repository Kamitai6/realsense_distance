#include "Path.h"


/**
 *  @brief Cubic Spline header library
 *
 *  @author Atsushi Sakai
 *
 **/
 
 

/* ---------- limit ---------- */
//y > 0, z > 0, 0.3 > x > -0.3, x > 0.3 && z > 0.15, position vector < 0.739103626


Path::Path(float* _y, int _num)
{
    y = _y;
    num = _num;
    InitParam();
}

void Path::InitParam()
{
    float w_[10];
    int ndata=num-1;
    
    for(int i=0;i<=ndata;i++)
        a_[i] = y[i];
    for(int i=0;i<ndata;i++) {
        if(i==0)
            c_[0] = 0.0;
        else
            c_[i] = 3.0*(a_[i-1]-2.0*a_[i]+a_[i+1]);
    }
    for(int i=0;i<ndata;i++) {
        if(i==0)
            w_[0] = 0.0;
        else {
            float tmp=4.0-w_[i-1];
            c_[i]=(c_[i]-c_[i-1])/tmp;
            w_[i] = 1.0/tmp;
        }
    }
    for(int i=(ndata-1);i>0;i--)
        c_[i]=c_[i]-c_[i+1]*w_[i];
    for(int i=0;i<=ndata;i++) {
        if(i==ndata) {
            d_[i] = 0.0;
            b_[i] = 0.0;
        }
        else {
            d_[i] = (c_[i+1]-c_[i])/3.0;
            b_[i] = (a_[i+1]-a_[i]-c_[i]-d_[i]);
        }
    }
}

void Path::Calc(float t) {
    int j=int(floor(t));
    
    if(j<0) j=0;
    else if(j>num) j=(num-1);
    
    float dt=t-j;
    f = a_[j]+(b_[j]+(c_[j]+d_[j]*dt)*dt)*dt;
    df = b_[j] + 2.0*c_[j]*dt + 3.0*d_[j]*dt*dt;
    ddf = 2.0*c_[j] + 6.0*d_[j]*dt;
}

