#include <crc_bambu_bus.h>

void bambu_bus_crc8_init(crc8_t* crc8)
{
    crc8_init(crc8, 0x39, 0x66, false, false, 0x00);
}

uint8_t bambu_bus_crc8_step(crc8_t* crc8, uint8_t byte)
{
    return crc8_step(crc8, byte);
}

uint8_t bambu_bus_crc8_finialize(crc8_t* crc8)
{
    return crc8_finialize(crc8);
}

void bambu_bus_crc16_init(crc16_t* crc16)
{
    crc16_init(crc16, 0x1021, 0x913D, true, true, 0x0000, false);
}

uint8_t bambu_bus_crc16_step(crc16_t* crc16, uint8_t byte)
{
    return crc16_step(crc16, byte);
}

uint8_t bambu_bus_crc16_finialize(crc16_t* crc16)
{
    return crc16_finialize(crc16);
}
