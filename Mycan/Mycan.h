#ifndef MYCAN_H
#define MYCAN_H

#include "mbed.h"
#include <map>

using namespace std;

class Mycan
{
    public:
        Mycan(PinName _rd, PinName _td, float freq = 500000);
        
        void setI (uint32_t _id, int _num, int16_t _data);
        void setF (uint32_t _id, int _num, float _data);
        
        bool send ();
        
        void readI ();
        void readF ();
        
        float get(uint32_t _id, int _num);      //get data

    private:
        CAN can;
        uint32_t rd_id, td_id;
        int      rd_num, td_num;
        bool     write_type, read_type;
        
        struct can_integer {
            char value[8];
        };
        can_integer td_integer, rd_integer;
        
        struct integer {
            int16_t value[8];
        };
        integer td_data, integer_storage;
        
        struct can_decimal {
            float value[2];
        };
        can_decimal td_decimal, rd_decimal;
        
        map<uint32_t, integer> integer_values_storage;
        map<uint32_t, can_decimal> decimal_values_storage;
};
#endif

