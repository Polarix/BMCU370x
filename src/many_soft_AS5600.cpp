
#include "many_soft_AS5600.h"
#include <Arduino.h>

#define I2C_DELAY 1
#define SET_H(port, pin)                   \
    {                                      \
        for (auto i = 0; i < numbers; i++) \
        {                                  \
            if (error[i] == 0)             \
                port[i]->BSHR = pin[i];    \
        }                                  \
    }
#define SET_L(port, pin)                   \
    {                                      \
        for (auto i = 0; i < numbers; i++) \
        {                                  \
            if (error[i] == 0)             \
                port[i]->BCR = pin[i];     \
        }                                  \
    }
#define iic_delay()           \
    {                         \
        delayMicroseconds(10); \
    }

// address
#define AS5600_IIC_ADDR (0x36 << 1)
#define AS5600_IIC_ADDR_R ((0x36 << 1) + 1)
/* 角度寄存器地址 */
#define AS5600_REG_ADDR_RAW_ANGLE 0x0C
// #define AS5600_angle 0x0E

/* 状态寄存器地址 */
#define AS5600_REG_ADDR_STATUS 0x0B
// #define AS5600_agc 0x1A
// #define AS5600_magnitude 0x1B
// #define AS5600_burn 0xFF

CAS5600::CAS5600()
{
    numbers = 0;
}
CAS5600::~CAS5600()
{
    if (numbers > 0)
    {
        delete IO_SDA;
        delete IO_SCL;
        delete port_SDA;
        delete port_SCL;
        delete pin_SDA;
        delete pin_SCL;
        delete online;
        delete magnet_stu;
        delete error;
        delete raw_angle;
        delete data;
    }
}

void CAS5600::init(const uint32_t* GPIO_SCL, const uint32_t* GPIO_SDA, int num)
{
    numbers = num;
    online = (new bool[numbers]);
    magnet_stu = (new _AS5600_magnet_stu[numbers]);
    error = (new int[numbers]);
    raw_angle = (new uint16_t[numbers]);
    data = (new uint16_t[numbers]);
    IO_SDA = (new uint32_t[numbers]);
    IO_SCL = (new uint32_t[numbers]);
    port_SDA = (new GPIO_TypeDef *[numbers]);
    port_SCL = (new GPIO_TypeDef *[numbers]);
    pin_SDA = (new uint16_t[numbers]);
    pin_SCL = (new uint16_t[numbers]);
    for (auto i = 0; i < numbers; i++)
    {
        IO_SDA[i] = GPIO_SDA[i];
        IO_SCL[i] = GPIO_SCL[i];
        port_SDA[i] = get_GPIO_Port(CH_PORT(digitalPinToPinName(IO_SDA[i])));
        pin_SDA[i] = CH_GPIO_PIN(digitalPinToPinName(IO_SDA[i]));
        port_SCL[i] = get_GPIO_Port(CH_PORT(digitalPinToPinName(IO_SCL[i])));
        pin_SCL[i] = CH_GPIO_PIN(digitalPinToPinName(IO_SCL[i]));
        magnet_stu[i] = offline;
        online[i] = false;
        raw_angle[i] = 0;
    }

    init_iic();
    updata_stu();
}
void CAS5600::clear_datas()
{
    for (int i = 0; i < numbers; i++)
    {
        error[i] = 0;
        data[i] = 0;
    }
}
void CAS5600::updata_stu()
{
    /* 读取数据到data成员里。 */
    read_reg8(AS5600_REG_ADDR_STATUS);
    for (int i = 0; i < numbers; i++)
    {
        if (error[i])
            online[i] = false;
        else
            online[i] = true;
        if (!(data[i] & 0x20))
        {
            magnet_stu[i] = offline;
        }
        else
        {
            if (data[i] & 0x10)
                magnet_stu[i] = low;
            else if (data[i] & 0x08)
                magnet_stu[i] = high;
            else
                magnet_stu[i] = normal;
        }
    }
}
void CAS5600::updata_angle()
{
    /* 读取数据到data成员里。 */
    read_reg16(AS5600_REG_ADDR_RAW_ANGLE);
    for (auto i = 0; i < numbers; i++)
    {
        if (error[i] == 0)
        {
            raw_angle[i] = data[i];
            online[i] = true;
        }
        else
        {
            raw_angle[i]=0;
            online[i] = false;
        }
    }
    
}
void CAS5600::init_iic()
{
    for (auto i = 0; i < numbers; i++)
    {
        digitalWrite(IO_SCL[i], 1);
        digitalWrite(IO_SDA[i], 1);
        pinMode(IO_SCL[i], OUTPUT); // to open clock
        pinMode(IO_SDA[i], OUTPUT_OD);
        error[i] = 0;
    }
}
void CAS5600::start_iic(unsigned char ADR)
{
    iic_delay();
    SET_H(port_SDA, pin_SDA);
    SET_H(port_SCL, pin_SCL);
    iic_delay();
    SET_L(port_SDA, pin_SDA);
    iic_delay();
    SET_L(port_SCL, pin_SCL);
    write_iic(ADR);
}

void CAS5600::stop_iic()
{
    SET_L(port_SCL, pin_SCL);
    SET_L(port_SDA, pin_SDA);
    iic_delay();
    SET_H(port_SCL, pin_SCL);
    iic_delay();
    SET_H(port_SDA, pin_SDA);
    iic_delay();
}

void CAS5600::write_iic(uint8_t byte)
{
    for (uint8_t i = 0x80; i; i >>= 1)
    {
        iic_delay();
        if (byte & i)
        {
            SET_H(port_SDA, pin_SDA);
        }
        else
        {
            SET_L(port_SDA, pin_SDA);
        }
        SET_H(port_SCL, pin_SCL);
        iic_delay();
        SET_L(port_SCL, pin_SCL);
    }
    wait_ack_iic();
}

void CAS5600::read_iic(bool ack)
{
    SET_H(port_SDA, pin_SDA);

    for (int i = 0; i < 8; i++)
    {
        
        iic_delay();
        SET_H(port_SCL, pin_SCL);
        iic_delay();
        for (int j = 0; j < numbers; j++)
        {
            data[j] <<= 1;
            if (port_SDA[j]->INDR & pin_SDA[j])
            {
                data[j] |= 0x01;
            }
        }
        SET_L(port_SCL, pin_SCL);
    }
    iic_delay();
    if (ack)
    {
        SET_L(port_SDA, pin_SDA);
    }
    else
    {
        SET_H(port_SDA, pin_SDA);
    }
    iic_delay();
    SET_H(port_SCL, pin_SCL);
    iic_delay();
    SET_L(port_SCL, pin_SCL);
    iic_delay();
}

void CAS5600::wait_ack_iic()
{
    SET_H(port_SDA, pin_SDA);
    iic_delay();
    SET_H(port_SCL, pin_SCL);
    iic_delay();
    for (auto i = 0; i < numbers; i++)
    {
        if (port_SDA[i]->INDR & pin_SDA[i])
            error[i] = 1;
    }
    SET_L(port_SCL, pin_SCL);
    return;
}

void CAS5600::read_reg8(uint8_t reg)
{
    if (!numbers)
        return;
    clear_datas();
    start_iic(AS5600_IIC_ADDR);
    write_iic(reg);
    start_iic(AS5600_IIC_ADDR_R);
    read_iic(false);
}

void CAS5600::read_reg16(uint8_t reg)
{
    if (!numbers)
        return;
    clear_datas();
    start_iic(AS5600_IIC_ADDR);
    write_iic(reg);
    start_iic(AS5600_IIC_ADDR_R);
    read_iic(true);
    read_iic(false);
}