#ifndef _INCLUDE_GENERAL_SOFT_MASTER_IIC_BSP_H_
#define _INCLUDE_GENERAL_SOFT_MASTER_IIC_BSP_H_
//===========================================================//
//= Include files.                                          =//
//===========================================================//
#include <stdint.h>
#include <stddef.h>

//===========================================================//
//= User Macro definition.                                  =//
//===========================================================//

//===========================================================//
//= Data type definition.                                   =//
//===========================================================//
typedef enum _e_soft_iic_io_dir_
{
    SOFT_IIC_DIN = 0,
    SOFT_IIC_DOUT
}soft_iic_io_dir_t;

typedef   void(*soft_iic_io_dir)(soft_iic_io_dir_t dir);
typedef   void(*soft_iic_io_set)(uint8_t val);
typedef int8_t(*soft_iic_io_get)(void);
typedef   void(*soft_iic_io_delay)(uint16_t times);


typedef struct _st_soft_iic_
{
    uint8_t             addr;
    soft_iic_io_set     scl_set_bit;
    soft_iic_io_set     sda_set_bit;
    soft_iic_io_get     sda_get_bit;
    soft_iic_io_dir     sda_set_dir;
    soft_iic_io_delay   delay;
}soft_iic_master_t;

//===========================================================//
//= Public function declaration.                            =//
//===========================================================//
void     soft_iic_master_bsp_start(const soft_iic_master_t* iic);
void     soft_iic_master_bsp_stop(const soft_iic_master_t* iic);
void     soft_iic_master_bsp_ack(const soft_iic_master_t* iic);
void     soft_iic_master_bsp_nack(const soft_iic_master_t* iic);
uint8_t  soft_iic_master_bsp_wait_ack(const soft_iic_master_t* iic);
void     soft_iic_master_bsp_write_byte(const soft_iic_master_t* iic, uint8_t byte);
uint8_t  soft_iic_master_bsp_read_byte(const soft_iic_master_t* iic);
void     soft_iic_master_bsp_flush_bus(const soft_iic_master_t* iic);
uint16_t soft_iic_master_bsp_write_data(const soft_iic_master_t* iic, uint8_t addr, const void* data, uint16_t len);
uint16_t soft_iic_master_bsp_fill_data(const soft_iic_master_t* iic, uint8_t addr, uint8_t data, uint16_t num);
uint16_t soft_iic_master_bsp_read_data(const soft_iic_master_t* iic, uint8_t addr, void* buffer, uint16_t len);
#endif // _INCLUDE_GENERAL_SOFT_MASTER_IIC_BSP_H_
