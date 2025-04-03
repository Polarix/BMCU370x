#ifndef _INCLUDE_RS485_BSP_H_
#define _INCLUDE_RS485_BSP_H_

#include <stdint.h>
#include <stddef.h>

typedef void(*rs485_byte_rev_handle)(uint8_t byte);

#ifdef __cplusplus
extern "C"
{
#endif

void rs_485_bsp_write(const void* data, uint16_t length);
void rs_485_bsp_init(void);
void rs_485_bsp_register_byte_rev_callback(rs485_byte_rev_handle callback);
void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

#ifdef __cplusplus
}
#endif

#endif /* _INCLUDE_RS485_BSP_H_ */