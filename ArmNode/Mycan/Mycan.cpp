#include "Mycan.h"


/*----------------------------------------------------*/

//"id"は小さくしたほうがメモリ効率がいいですね
//あと、整数で使うときは"num"は1~7、小数は0か1で御願いします

/*----------------------------------------------------*/

Mycan::Mycan(PinName _rd, PinName _td, float freq) : can(_rd, _td)
{
    can.frequency(freq);
};

void Mycan::setI(uint32_t _id, int _num, int16_t _data)
{
    write_type = 0;
    td_id = _id;
    td_num = _num;
    td_data.value[td_num] = _data;
    
    for (int i = 0; i < 7; i++)
    {
        if (td_data.value[i] >= 0)
            td_integer.value[7] &= ~(1 << i);
        else
            td_integer.value[7] |= (1 << i);
        td_integer.value[i] = abs(td_data.value[i]);
    }
}

void Mycan::setF(uint32_t _id, int _num, float _data)
{
    write_type = 1;
    td_id = _id;
    td_num = _num;
    td_decimal.value[td_num] = _data;
}

bool Mycan::send()
{
    if (!write_type) return can.write(CANMessage(td_id, (char*)&td_integer, 8));
    else if (write_type) return can.write(CANMessage(td_id, (char*)&td_decimal, 8));
}

void Mycan::readI()
{
    read_type = 0;
    CANMessage received;
    can.read(received);
    rd_integer = *(can_integer*)received.data;
    
    for (int i = 0; i < 7; i++) {
        if (rd_integer.value[7] & (1 << i))
            integer_storage.value[i] = rd_integer.value[i] * -1;
        else integer_storage.value[i] = rd_integer.value[i];
    }
    integer_values_storage[received.id] = integer_storage;
}

void Mycan::readF()
{
    read_type = 1;
    CANMessage received;
    can.read(received);
    rd_decimal = *(can_decimal *)received.data;
    
    decimal_values_storage[received.id] = rd_decimal;
}

float Mycan::get(uint32_t _id, int _num)
{
    rd_id = _id;
    rd_num = _num;
    if (!read_type) return integer_values_storage[rd_id].value[rd_num];
    else if (read_type) return decimal_values_storage[rd_id].value[rd_num];
}

