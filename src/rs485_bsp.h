#ifndef _INCLUDE_RS485_BSP_H_
#define _INCLUDE_RS485_BSP_H_

#include <stdint.h>

typedef void(*rs485_byte_rev_handle)(uint8_t byte);

#ifdef __cplusplus
extern "C"
{
#endif

void bambu_bus_send_data(const uint8_t *data, uint16_t length);
void bambu_bus_init_usart(void);
void rs_485_bsp_register_byte_rev_callback(rs485_byte_rev_handle callback);
void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

#ifdef __cplusplus
}
#endif

#endif /* _INCLUDE_RS485_BSP_H_ */