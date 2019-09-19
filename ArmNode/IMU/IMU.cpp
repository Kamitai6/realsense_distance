#include"IMU.h"
#include"mbed.h"
#include "mpu9250_i2c.h"
#include "math.h"


IMU::IMU(float t, PinName sla , PinName sld) : i2c(sla, sld), imu(i2c, AD0_LOW)
{
    delta_t = t;
    imu.setAcc();
    imu.setGyro(_1000DPS);
}


void IMU::_updataImuValue()
{
    imu.getAcc (&acc[0],  &acc[1],  &acc[2]);
    imu.getGyro(&gyro[0], &gyro[1], &gyro[2]);
    imu.getMag (&mag[0],  &mag[1],  &mag[2]);
}


void IMU::computeAngles()
{
    float gyro_angle = 0;
    float mag_angle = 0;
    float k = 0.95;
    
    _updataImuValue();
    /*
    mag_angle = (atan2(mag[1], mag[0]) - init_mag_angle[2]) / (PI);
    if(mag_angle > 1) mag_angle -= 2;
    else if(mag_angle < -1) mag_angle += 2;
    
    gyro_angle = angle[2] + gyro[2] * delta_t / 180;
    */
    //angle[2] = k * gyro_angle - (1-k) * mag_angle;
    angle[2] = angle[2] + gyro[2] * delta_t;
}


void IMU::setOffset(float* _offset)
{
    for(int i = 0; i < 9; i++)
        offset[i] = _offset[i];
}


void IMU::performCalibration()
{
    double raw_value[9] = {};
    
    wait(1);
    for(int i = 0; i < 1000; i++)
    {
        imu.getAcc (&raw_value[0], &raw_value[1], &raw_value[2]);
        imu.getGyro(&raw_value[3], &raw_value[4], &raw_value[5]);
        imu.getMag (&raw_value[6], &raw_value[7], &raw_value[8]);
        
        for(int j = 0; j < 6; j++)
            offset[j] += raw_value[j];
        init_mag_angle[2] += atan2(raw_value[7], raw_value[6]);
    }
    for(int i = 0; i < 6; i++)
        offset[i] /= 1000;
    for(int i = 0; i < 3; i++)
        init_mag_angle[i] /= 1000;
    
    imu.setOffset(  offset[0], offset[1], offset[2] - 1,
                    offset[3], offset[4], offset[5],
                    offset[6], offset[7], offset[8]);
}
