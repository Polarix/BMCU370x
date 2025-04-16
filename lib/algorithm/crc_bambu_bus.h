#ifndef _INCLUDE_CRC_BAMBU_BUS_H_
#define _INCLUDE_CRC_BAMBU_BUS_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <crc.h>

#ifdef __cplusplus
extern "C"{
#endif

void bambu_bus_crc8_init(crc8_t* crc8);
uint8_t bambu_bus_crc8_step(crc8_t* crc8, uint8_t byte);
uint8_t bambu_bus_crc8_finialize(crc8_t* crc8);
void bambu_bus_crc16_init(crc16_t* crc16);
uint8_t bambu_bus_crc16_step(crc16_t* crc16, uint8_t byte);
uint8_t bambu_bus_crc16_finialize(crc16_t* crc16);

#ifdef __cplusplus
}
#endif

#endif /* _INCLUDE_CRC_BAMBU_BUS_H_ */
