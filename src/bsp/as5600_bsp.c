//===============================================================//
//= Include files.                                              =//
//===============================================================//
#include <bsp/soft_iic_master.h>
#include <stdbool.h>
#include <ch32v20x_gpio.h>

//===============================================================//
//= Macro definition.                                           =//
//===============================================================//
#define AS5600_IIC_ADDR (0x36 << 1)

//===============================================================//
//= Static function declare.                                    =//
//===============================================================//
static void as5600_bsp_init_gpio(void);
static void as5600_bsp_init_delay(uint16_t times);
static void as5600_ch1_bsp_scl_set_bit(uint8_t val);
static void as5600_ch2_bsp_scl_set_bit(uint8_t val);
static void as5600_ch3_bsp_scl_set_bit(uint8_t val);
static void as5600_ch4_bsp_scl_set_bit(uint8_t val);
static void as5600_ch1_bsp_sda_set_bit(uint8_t val);
static void as5600_ch2_bsp_sda_set_bit(uint8_t val);
static void as5600_ch3_bsp_sda_set_bit(uint8_t val);
static void as5600_ch4_bsp_sda_set_bit(uint8_t val);
static int8_t as5600_ch1_bsp_sda_get_bit(void);
static int8_t as5600_ch2_bsp_sda_get_bit(void);
static int8_t as5600_ch3_bsp_sda_get_bit(void);
static int8_t as5600_ch4_bsp_sda_get_bit(void);
static void as5600_ch1_bsp_sda_set_dir(soft_iic_io_dir_t dir);
static void as5600_ch2_bsp_sda_set_dir(soft_iic_io_dir_t dir);
static void as5600_ch3_bsp_sda_set_dir(soft_iic_io_dir_t dir);
static void as5600_ch4_bsp_sda_set_dir(soft_iic_io_dir_t dir);

//===============================================================//
//= Static variable declaration.                                =//
//===============================================================//
static const soft_iic_master_t s_as5600[4] = 
{
    /* Channel 1 */
    {
        .addr = AS5600_IIC_ADDR,
        .scl_set_bit = as5600_ch1_bsp_scl_set_bit,
        .sda_set_bit = as5600_ch1_bsp_sda_set_bit,
        .sda_get_bit = as5600_ch1_bsp_sda_get_bit,
        .sda_set_dir = as5600_ch1_bsp_sda_set_dir,
        .delay = as5600_bsp_init_delay
    },
    /* Channel 2 */
    {
        .addr = AS5600_IIC_ADDR,
        .scl_set_bit = as5600_ch2_bsp_scl_set_bit,
        .sda_set_bit = as5600_ch2_bsp_sda_set_bit,
        .sda_get_bit = as5600_ch2_bsp_sda_get_bit,
        .sda_set_dir = as5600_ch2_bsp_sda_set_dir,
        .delay = as5600_bsp_init_delay
    },
    /* Channel 3 */
    {
        .addr = AS5600_IIC_ADDR,
        .scl_set_bit = as5600_ch3_bsp_scl_set_bit,
        .sda_set_bit = as5600_ch3_bsp_sda_set_bit,
        .sda_get_bit = as5600_ch3_bsp_sda_get_bit,
        .sda_set_dir = as5600_ch3_bsp_sda_set_dir,
        .delay = as5600_bsp_init_delay
    },
    /* Channel 4 */
    {
        .addr = AS5600_IIC_ADDR,
        .scl_set_bit = as5600_ch4_bsp_scl_set_bit,
        .sda_set_bit = as5600_ch4_bsp_sda_set_bit,
        .sda_get_bit = as5600_ch4_bsp_sda_get_bit,
        .sda_set_dir = as5600_ch4_bsp_sda_set_dir,
        .delay = as5600_bsp_init_delay
    },
};

//===============================================================//
//= Function definition.                                        =//
//===============================================================//
void as5600_bsp_init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    as5600_bsp_init_gpio();

}

static void as5600_bsp_init_delay(uint16_t times)
{
    uint8_t ticks = 144; /* @144MHz */
	while(times--)
    {
        while(--ticks);
    }
}

static void as5600_bsp_init_gpio(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | 
                                  GPIO_Pin_2 | GPIO_Pin_3 | 
                                  GPIO_Pin_4 | GPIO_Pin_5 | 
                                  GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

static void as5600_ch1_bsp_scl_set_bit(uint8_t val)
{
	if(val)
    {
        GPIO_SetBits(GPIOA, GPIO_Pin_0);
    }
    else
    {
        GPIO_ResetBits(GPIOA, GPIO_Pin_0);
    }        
}

static void as5600_ch2_bsp_scl_set_bit(uint8_t val)
{
	if(val)
    {
        GPIO_SetBits(GPIOA, GPIO_Pin_2);
    }
    else
    {
        GPIO_ResetBits(GPIOA, GPIO_Pin_2);
    }        
}

static void as5600_ch3_bsp_scl_set_bit(uint8_t val)
{
	if(val)
    {
        GPIO_SetBits(GPIOA, GPIO_Pin_4);
    }
    else
    {
        GPIO_ResetBits(GPIOA, GPIO_Pin_4);
    }        
}

static void as5600_ch4_bsp_scl_set_bit(uint8_t val)
{
	if(val)
    {
        GPIO_SetBits(GPIOA, GPIO_Pin_6);
    }
    else
    {
        GPIO_ResetBits(GPIOA, GPIO_Pin_6);
    }        
}

static void as5600_ch1_bsp_sda_set_bit(uint8_t val)
{
	if(val)
    {
        GPIO_SetBits(GPIOA, GPIO_Pin_1);
    }
    else
    {
        GPIO_ResetBits(GPIOA, GPIO_Pin_1);
    }        
}

static void as5600_ch2_bsp_sda_set_bit(uint8_t val)
{
	if(val)
    {
        GPIO_SetBits(GPIOA, GPIO_Pin_3);
    }
    else
    {
        GPIO_ResetBits(GPIOA, GPIO_Pin_3);
    }        
}

static void as5600_ch3_bsp_sda_set_bit(uint8_t val)
{
	if(val)
    {
        GPIO_SetBits(GPIOA, GPIO_Pin_5);
    }
    else
    {
        GPIO_ResetBits(GPIOA, GPIO_Pin_5);
    }        
}

static void as5600_ch4_bsp_sda_set_bit(uint8_t val)
{
	if(val)
    {
        GPIO_SetBits(GPIOA, GPIO_Pin_7);
    }
    else
    {
        GPIO_ResetBits(GPIOA, GPIO_Pin_7);
    }        
}

static int8_t as5600_ch1_bsp_sda_get_bit(void)
{
	return GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1);
}

static int8_t as5600_ch2_bsp_sda_get_bit(void)
{
	return GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3);
}

static int8_t as5600_ch3_bsp_sda_get_bit(void)
{
	return GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5);
}

static int8_t as5600_ch4_bsp_sda_get_bit(void)
{
	return GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7);
}

static void as5600_ch1_bsp_sda_set_dir(soft_iic_io_dir_t dir)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    if(SOFT_IIC_DIN == dir)
    {
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    }
    else
    {
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    }
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

static void as5600_ch2_bsp_sda_set_dir(soft_iic_io_dir_t dir)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    if(SOFT_IIC_DIN == dir)
    {
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    }
    else
    {
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    }
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

static void as5600_ch3_bsp_sda_set_dir(soft_iic_io_dir_t dir)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    if(SOFT_IIC_DIN == dir)
    {
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    }
    else
    {
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    }
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

static void as5600_ch4_bsp_sda_set_dir(soft_iic_io_dir_t dir)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    if(SOFT_IIC_DIN == dir)
    {
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    }
    else
    {
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    }
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void as5600_bsp_flush_bus(int8_t ch)
{
    if(ch < 4)
    {
        soft_iic_master_bsp_flush_bus(&s_as5600[ch]);
    }
}

void as5600_bsp_write_byte(int8_t ch, uint8_t addr, uint8_t val)
{
    if(ch < 4)
    {
        soft_iic_master_bsp_write_data(&s_as5600[ch], addr, &val, sizeof(val));
    }
}

uint16_t as5600_bsp_write_data(int8_t ch, uint8_t addr, const void* data, uint32_t size)
{
    uint16_t write_num = 0;
    if((size > 0) && (ch < 4))
    {
        write_num = soft_iic_master_bsp_write_data(&s_as5600[ch], addr, data, size);
    }
    return write_num;
}

uint8_t as5600_bsp_read_byte(int8_t ch, uint8_t addr)
{
    uint8_t read_byte = 0xFF;
    // IIC start
    soft_iic_master_bsp_start(&s_as5600[ch]);
    // Write device address.
    soft_iic_master_bsp_write_byte(&s_as5600[ch], s_as5600[ch].addr);
    soft_iic_master_bsp_wait_ack(&s_as5600[ch]);
    // Write register address.
    soft_iic_master_bsp_write_byte(&s_as5600[ch], addr);
    soft_iic_master_bsp_wait_ack(&s_as5600[ch]);
    // IIC start
    soft_iic_master_bsp_start(&s_as5600[ch]);
    // Write device address.
    soft_iic_master_bsp_write_byte(&s_as5600[ch], s_as5600[ch].addr + 1);
    soft_iic_master_bsp_wait_ack(&s_as5600[ch]);
    read_byte = soft_iic_master_bsp_read_byte(&s_as5600[ch]);
    soft_iic_master_bsp_nack(&s_as5600[ch]);
    soft_iic_master_bsp_stop(&s_as5600[ch]);

    return read_byte;
}

uint16_t as5600_bsp_read_data(int8_t ch, uint8_t addr, void* buffer, uint16_t size)
{
    uint16_t read_num = 0;
    if((size > 0) && (ch < 4))
    {
        soft_iic_master_bsp_read_data(&s_as5600[ch], addr, buffer, size);
    }
    return read_num;
}