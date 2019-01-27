#ifndef MYCAN_H
#define MYCAN_H

#include "mbed.h"

class Mycan
{
    public:
        Mycan(PinName _pin_rd, PinName _pin_td);//(現在IDは1~10の設定になっている）
        
        void set (unsigned int _id, int _num, short _data);//CANMessageの設定, numは１～７, dataは-255~255
        
        bool send ();//書き込み
        
        void read ();//読み込み
        
        float get(unsigned int _id, int _num);//readした値を持ってくる, IDやnumはコンストラクタと一緒

    private:
        CAN can;
        unsigned int rd_id, td_id;
        int rd_num, td_num;
        unsigned int min_id;
        short data;
        static short write_val[8];
        static short read_val[10][8];//行がIDの最大数
        void _expressAbsoluteValue();
        void _expressSignVal(unsigned int);
        
        typedef struct
        {
            char writeVal_0;
            char writeVal_1;
            char writeVal_2;
            char writeVal_3;
            char writeVal_4;
            char writeVal_5;
            char writeVal_6;
            char writeVal_7;
        }write_can;
        
        write_can td_data;
        
        typedef struct
        {
            char readVal_0;
            char readVal_1;
            char readVal_2;
            char readVal_3;
            char readVal_4;
            char readVal_5;
            char readVal_6;
            char readVal_7;
        }read_can;
        
        read_can rd_data;
};
#endif