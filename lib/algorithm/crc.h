#ifndef _INCLUDE_CRC_H_
#define _INCLUDE_CRC_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

typedef struct _st_crc8_data_
{
    uint8_t     polynome;
    uint8_t     initial;
    uint8_t     xor_out;
    bool        reverse_in;
    bool        reverse_out;
    uint8_t     cala;
}crc8_t;

typedef struct _st_crc16_data_
{
    uint16_t    polynome;
    uint16_t    initial;
    uint16_t    xor_out;
    bool        reverse_in;
    bool        reverse_out;
    uint16_t    cala;
}crc16_t;

#ifdef __cplusplus
extern "C"{
#endif

// ==================== CRC8 Basic functions ====================
void crc8_init(crc8_t* crc8, uint8_t polynomial, uint8_t init, bool refin, bool refout, uint8_t xorout);
uint8_t crc8_step(crc8_t* crc8, uint8_t byte);
uint8_t crc8_finialize(crc8_t* crc8);
// ==================== CRC8 Preset parameters ====================
// CRC-8/MAXIM (Dallas 1-Wire)
uint8_t crc8(const uint8_t *data, size_t length);
// CRC-8/ITU
uint8_t crc8_itu(const uint8_t *data, size_t length);
// CRC-8/ROHC
uint8_t crc8_rohc(const uint8_t *data, size_t length);
// CRC-8/SAE-J1850
uint8_t crc8_sae_j1850(const uint8_t *data, size_t length);
// CRC-8/Bambu Bus
uint8_t crc8_bambu_bus(const uint8_t *data, size_t length);


// ==================== CRC16 Basic functions ====================
void crc16_init(crc16_t* crc16, uint16_t polynomial, uint16_t init, bool refin, bool refout, uint16_t xorout);
uint16_t crc16_step(crc16_t* crc16, uint8_t byte);
uint16_t crc16_finialize(crc16_t* crc16, bool byte_swap);
// ==================== CRC16 Preset parameters ====================
// CRC-16/CCITT-FALSE
uint16_t crc16_ccitt_false(const uint8_t *data, size_t length);
// CRC-16/CCITT (KERMIT)
uint16_t crc16_ccitt_true(const uint8_t *data, size_t length);
// CRC-16/MODBUS
uint16_t crc16_modbus(const uint8_t *data, size_t length);
// CRC-16/USB
uint16_t crc16_usb(const uint8_t *data, size_t length);
// CRC-16/XMODEM
uint16_t crc16_xmodem(const uint8_t *data, size_t length);
// CRC-16/IBM
uint16_t crc16_ibm(const uint8_t *data, size_t length);
// CRC-16/DNP
uint16_t crc16_dnp(const uint8_t *data, size_t length);
// CRC-16/X25
uint16_t crc16_x25(const uint8_t *data, size_t length);
// CRC-16/Bambu Bus
uint16_t crc16_bambu_bus(const uint8_t *data, size_t length);

#ifdef __cplusplus
}
#endif

#endif /* _INCLUDE_CRC_H_ */
