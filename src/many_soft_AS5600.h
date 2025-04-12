#ifndef _INCLUDE_CLASS_AS5600_DRV_H_
#define _INCLUDE_CLASS_AS5600_DRV_H_

#include <stdint.h>
#include <stddef.h>
#include <ch32v20x_gpio.h>

class CAS5600
{
public:
    explicit CAS5600(void);
    virtual ~CAS5600(void);
    void init(const uint32_t* GPIO_SCL, const uint32_t* GPIO_SDA,int num);

    bool *online;
    enum _AS5600_magnet_stu
    {
        low=1,
        high=2,
        offline=-1,
        normal=0
    }*magnet_stu;
    uint16_t *raw_angle;

    void updata_stu();
    void updata_angle();
    int numbers;
    uint16_t *data;
private:

    int* error; /* 读写数据时是否有错误发生，用来标记从机是否在线，与online成员关联。 */
    uint32_t *IO_SDA;
    uint32_t *IO_SCL;
    GPIO_TypeDef **port_SDA;
    uint16_t *pin_SDA;
    GPIO_TypeDef **port_SCL;
    uint16_t *pin_SCL;
    
    void init_iic();
    void start_iic(unsigned char ADR);
    void stop_iic();
    void write_iic(uint8_t byte);
    void read_iic(bool ack);
    void wait_ack_iic();
    void clear_datas();
    void read_reg8(uint8_t reg);
    void read_reg16(uint8_t reg);
};

#endif /* _INCLUDE_CLASS_AS5600_DRV_H_ */