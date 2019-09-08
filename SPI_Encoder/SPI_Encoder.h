#ifndef SPI_ENCODER_H
#define SPI_ENCODER_H
 
#include "mbed.h"

#define encoder_num 4
#define PI 3.14159265359


/* cui amt203 */
class SPI_Encoder
{
    public:
        SPI_Encoder(PinName mosi, PinName miso, PinName sclk, PinName _cs0, PinName _cs1, PinName _cs2, PinName _cs3, float t);
        void getPosition(int num);
        bool setZero(int num);
        void inverse(int num);
        
        float angle[encoder_num];
        float velocity[encoder_num];
        
    private:
        SPI encoder;
        DigitalOut cs0;
        DigitalOut cs1;
        DigitalOut cs2;
        DigitalOut cs3;
        float delta_t;
        bool direction[4];
        uint8_t temp[2][encoder_num];
        uint16_t EncoderByteData[encoder_num];
        uint8_t _SPI_T (int num, uint8_t SPITransmit);
        void _switching(int num, int value);
};

#endif
