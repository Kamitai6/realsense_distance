#include "mbed.h"
#include "Mycan.h"

Mycan::Mycan(PinName _pin_rd, PinName _pin_td) : can(_pin_rd, _pin_td)
{
    min_id = 1;//※ここで最小IDを設定
};

void Mycan::set(unsigned int _id, int _num, short _data)
{
    td_id = _id;
    td_num = _num;
    data = _data;
    write_val[td_num] = data;
    _expressAbsoluteValue();//絶対値にする関数
}

bool Mycan::send()
{
    return can.write(CANMessage(td_id, (char*)&td_data, 8));
}

void Mycan::read()
{
    CANMessage received;
    can.read(received);
    rd_data = *(read_can *)received.data;
    _expressSignVal(received.id);//符号を付与する関数
}

float Mycan::get(unsigned int _id, int _num)
{
    rd_id = _id;
    rd_num = _num;
    return read_val[rd_id - min_id][rd_num];
}

void Mycan::_expressAbsoluteValue()
{
    write_val[0] = 0;
    for (int i = 1; i < 8; i++)
    {
        if (write_val[i] >= 0)
            write_val[0] &= ~(1 << i);
        else
            write_val[0] |= (1 << i);
    }
    td_data.writeVal_0 = write_val[0];
    td_data.writeVal_1 = abs(write_val[1]);
    td_data.writeVal_2 = abs(write_val[2]);
    td_data.writeVal_3 = abs(write_val[3]);
    td_data.writeVal_4 = abs(write_val[4]);
    td_data.writeVal_5 = abs(write_val[5]);
    td_data.writeVal_6 = abs(write_val[6]);
    td_data.writeVal_7 = abs(write_val[7]);
}

void Mycan::_expressSignVal(unsigned int id)
{
    read_val[id - min_id][0] = rd_data.readVal_0;
    read_val[id - min_id][1] = rd_data.readVal_1;
    read_val[id - min_id][2] = rd_data.readVal_2;
    read_val[id - min_id][3] = rd_data.readVal_3;
    read_val[id - min_id][4] = rd_data.readVal_4;
    read_val[id - min_id][5] = rd_data.readVal_5;
    read_val[id - min_id][6] = rd_data.readVal_6;
    read_val[id - min_id][7] = rd_data.readVal_7;
    
    for (int j = 1; j < 8; j++)
    {
        if (read_val[id - min_id][0] & (1 << j))
            read_val[id - min_id][j] *= -1;
    }
}

short Mycan::read_val[10][8];
short Mycan::write_val[8];