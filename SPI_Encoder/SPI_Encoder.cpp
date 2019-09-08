#include "mbed.h"
#include "SPI_Encoder.h"


SPI_Encoder::SPI_Encoder(PinName mosi, PinName miso, PinName sclk, PinName _cs0, PinName _cs1, PinName _cs2, PinName _cs3, float t) : encoder(mosi, miso, sclk), cs0(_cs0), cs1(_cs1), cs2(_cs2), cs3(_cs3)
{
    encoder.format(8,1);
    encoder.frequency(8000000);
    delta_t = t;
    for(int i = 0; i < encoder_num; i++)
        direction[i] = 0;
    cs0 = 1;
    cs1 = 1;
    cs2 = 1;
    cs3 = 1;
    wait(0.1); //encoder init
};

void SPI_Encoder::inverse(int num)
{
    direction[num] = 1;
}

void SPI_Encoder::getPosition(int num)
{
    float pre_angle = angle[num];
    float _angle = 0;
    
    uint8_t received = _SPI_T(num, 0x10);   //read command
    while (received != 0x10) {              //loop while encoder is not ready to send
        received = _SPI_T(num, 0x00);
    }
    temp[0][num] = _SPI_T(num, 0x00);       //Recieve MSB
    temp[1][num] = _SPI_T(num, 0x00);       //recieve LSB
    temp[0][num] &=~ 0xF0;                  //mask out the first 4 bits
    EncoderByteData[num]  = temp[0][num] << 8;
    EncoderByteData[num]  += temp[1][num];
    
    if(!direction[num]) _angle = EncoderByteData[num] / 4096.0f * 2.0f * PI;    //normal
    else _angle = abs((EncoderByteData[num] / 4096.0f * 2.0f * PI) - 2*PI);     //inverse
    if(_angle > PI) angle[num] = _angle - 2*PI;                                 //0~2π → -π~π
    else angle[num] = _angle;
    
    velocity[num] = (angle[num] - pre_angle) / delta_t;
}

bool SPI_Encoder::setZero(int num)
{
    uint8_t received = _SPI_T(num, 0x70);   //read command
    while (received != 0x80) {              //loop while encoder is not ready to send
        received = _SPI_T(num, 0x00);
    }
    return true;
}

/* switch MSB or LSB */
uint8_t SPI_Encoder::_SPI_T (int num, uint8_t SPITransmit)
{
    _switching(num, 0);
    uint8_t SPI_temp = encoder.write(SPITransmit);
    _switching(num, 1);
    wait_us(20);            //Timmig is critical
    return (SPI_temp);
}

/* switch read encoder */
void SPI_Encoder::_switching(int num, int value)
{
    switch (num) {
        case 0:
            cs0 = value;
            break;
        case 1:
            cs1 = value;
            break;
        case 2:
            cs2 = value;
            break;
        case 3:
            cs3 = value;
            break;
        default:
            break;
    }
}
