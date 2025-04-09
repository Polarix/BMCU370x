#ifndef _INCLUDE_AS5600_SOFT_IIC_BSP_H_
#define _INCLUDE_AS5600_SOFT_IIC_BSP_H_
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

//===========================================================//
//= Public function declaration.                            =//
//===========================================================//
void     as5600_bsp_init(void);
void     as5600_bsp_flush_bus(int8_t ch);
void     as5600_bsp_write_byte(int8_t ch, uint8_t addr, uint8_t val);
uint16_t as5600_bsp_write_data(int8_t ch, uint8_t addr, const void* data, uint32_t size);
uint8_t  as5600_bsp_read_byte(int8_t ch, uint8_t addr);
uint16_t as5600_bsp_read_data(int8_t ch, uint8_t addr, void* buffer, uint16_t len);
#endif // _INCLUDE_AS5600_SOFT_IIC_BSP_H_
