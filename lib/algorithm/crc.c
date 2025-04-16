#include <crc.h>

void crc8_init(crc8_t* crc8, uint8_t polynomial, uint8_t init, bool refin, bool refout, uint8_t xorout)
{
    crc8->polynome = polynomial;
    crc8->initial = init;
    crc8->reverse_in = refin;
    crc8->reverse_out = refout;
    crc8->xor_out = xorout;
    crc8->cala = init;
}

uint8_t crc8_step(crc8_t* crc8, uint8_t byte)
{
    if(crc8->reverse_in)
    {
        /* reverse bit order. */
        byte = (((byte & 0xAA) >> 1) | ((byte & 0x55) << 1));
        byte = (((byte & 0xCC) >> 2) | ((byte & 0x33) << 2));
        byte =          ((byte >> 4) | (byte << 4));
    }

    crc8->cala ^= byte;

    for (uint8_t bit = 0; bit < 8; bit++)
    {
        if (crc8->cala & 0x80)
        {
            crc8->cala = (crc8->cala << 1) ^ crc8->polynome;
        }
        else
        {
            crc8->cala <<= 1;
        }
    }
    return crc8->cala;
}

uint8_t crc8_finialize(crc8_t* crc8)
{
    if (crc8->reverse_out)
    {
        /* reverse bit order. */
        crc8->cala = (((crc8->cala & 0xAA) >> 1) | ((crc8->cala & 0x55) << 1));
        crc8->cala = (((crc8->cala & 0xCC) >> 2) | ((crc8->cala & 0x33) << 2));
        crc8->cala =          ((crc8->cala >> 4) | (crc8->cala << 4));
    }
    crc8->cala = crc8->cala ^ crc8->xor_out;
    return crc8->cala;
}

uint8_t crc8(const uint8_t *data, size_t length)
{
    // CRC-8/MAXIM (Dallas 1-Wire)
    crc8_t crc8;
    crc8_init(&crc8, 0x31, 0x00, true, true, 0x00);
    for(size_t i=0; i<length; ++i)
    {
        crc8_step(&crc8, data[i]);
    }
    return crc8_finialize(&crc8);
}

uint8_t crc8_itu(const uint8_t *data, size_t length)
{
    // CRC-8/ITU
    crc8_t crc8;
    crc8_init(&crc8, 0x07, 0x00, false, false, 0x55);
    for(size_t i=0; i<length; ++i)
    {
        crc8_step(&crc8, data[i]);
    }
    return crc8_finialize(&crc8);
}

uint8_t crc8_rohc(const uint8_t *data, size_t length)
{
    // CRC-8/ROHC
    crc8_t crc8;
    crc8_init(&crc8, 0x07, 0xFF, true, true, 0x00);
    for(size_t i=0; i<length; ++i)
    {
        crc8_step(&crc8, data[i]);
    }
    return crc8_finialize(&crc8);
}

uint8_t crc8_sae_j1850(const uint8_t *data, size_t length)
{
    // CRC-8/SAE-J1850
    crc8_t crc8;
    crc8_init(&crc8, 0x1D, 0xFF, false, false, 0xFF);
    for(size_t i=0; i<length; ++i)
    {
        crc8_step(&crc8, data[i]);
    }
    return crc8_finialize(&crc8);
}

uint8_t crc8_bambu_bus(const uint8_t *data, size_t length)
{
    // CRC-8/Test for bambu bus.
    crc8_t crc8;
    crc8_init(&crc8, 0x39, 0x66, false, false, 0x00);
    for(size_t i=0; i<length; ++i)
    {
        crc8_step(&crc8, data[i]);
    }
    return crc8_finialize(&crc8);
}

void crc16_init(crc16_t* crc16, uint16_t polynomial, uint16_t init, bool refin, bool refout, uint16_t xorout)
{
    crc16->polynome = polynomial;
    crc16->initial = init;
    crc16->reverse_in = refin;
    crc16->reverse_out = refout;
    crc16->xor_out = xorout;
    crc16->cala = init;
}

uint16_t crc16_step(crc16_t* crc16, uint8_t byte)
{
    if(crc16->reverse_in)
    {
        /* reverse bit order. */
        byte = (((byte & 0xAA) >> 1) | ((byte & 0x55) << 1));
        byte = (((byte & 0xCC) >> 2) | ((byte & 0x33) << 2));
        byte =          ((byte >> 4) | (byte << 4));
    }

    crc16->cala ^= (uint16_t)byte << 8;

    for (uint8_t bit = 0; bit < 8; bit++)
    {
        if (crc16->cala & 0x8000)
        {
            crc16->cala = (crc16->cala << 1) ^ crc16->polynome;
        }
        else
        {
            crc16->cala <<= 1;
        }
    }
    return crc16->cala;
}

uint16_t crc16_finialize(crc16_t* crc16, bool byte_swap)
{
    if (crc16->reverse_out)
    {
        crc16->cala = (((crc16->cala & 0XAAAA) >> 1) | ((crc16->cala & 0X5555) << 1));
        crc16->cala = (((crc16->cala & 0xCCCC) >> 2) | ((crc16->cala & 0X3333) << 2));
        crc16->cala = (((crc16->cala & 0xF0F0) >> 4) | ((crc16->cala & 0X0F0F) << 4));
        crc16->cala = (( crc16->cala >> 8) | (crc16->cala << 8));
    }
    crc16->cala = crc16->cala ^ crc16->xor_out;
    if(byte_swap)
    {
        crc16->cala = (crc16->cala >> 8) | (crc16->cala << 8);
    }
    return crc16->cala;
}

uint16_t crc16_ccitt_false(const uint8_t *data, size_t length)
{
    // CRC-16/CCITT-FALSE
    crc16_t crc16;
    crc16_init(&crc16, 0x1021, 0xFFFF, false, false, 0x0000);
    for(size_t i=0; i<length; ++i)
    {
        crc16_step(&crc16, data[i]);
    }
    return crc16_finialize(&crc16, false);
}

uint16_t crc16_ccitt_true(const uint8_t *data, size_t length)
{
    // CRC-16/CCITT (KERMIT)
    crc16_t crc16;
    crc16_init(&crc16, 0x1021, 0x0000, true, true, 0x0000);
    for(size_t i=0; i<length; ++i)
    {
        crc16_step(&crc16, data[i]);
    }
    return crc16_finialize(&crc16, false);
}

uint16_t crc16_modbus(const uint8_t *data, size_t length)
{
    // CRC-16/MODBUS
    crc16_t crc16;
    crc16_init(&crc16, 0x8005, 0xFFFF, true, true, 0x0000);
    for(size_t i=0; i<length; ++i)
    {
        crc16_step(&crc16, data[i]);
    }
    return crc16_finialize(&crc16, false);
}

uint16_t crc16_usb(const uint8_t *data, size_t length)
{
    // CRC-16/USB
    crc16_t crc16;
    crc16_init(&crc16, 0x8005, 0xFFFF, true, true, 0xFFFF);
    for(size_t i=0; i<length; ++i)
    {
        crc16_step(&crc16, data[i]);
    }
    return crc16_finialize(&crc16, false);
}

uint16_t crc16_xmodem(const uint8_t *data, size_t length)
{
    // CRC-16/XMODEM
    crc16_t crc16;
    crc16_init(&crc16, 0x1021, 0x0000, false, false, 0x0000);
    for(size_t i=0; i<length; ++i)
    {
        crc16_step(&crc16, data[i]);
    }
    return crc16_finialize(&crc16, false);
}

uint16_t crc16_ibm(const uint8_t *data, size_t length)
{
    // CRC-16/IBM
    crc16_t crc16;
    crc16_init(&crc16, 0x8005, 0x0000, true, true, 0x0000);
    for(size_t i=0; i<length; ++i)
    {
        crc16_step(&crc16, data[i]);
    }
    return crc16_finialize(&crc16, false);
}

uint16_t crc16_dnp(const uint8_t *data, size_t length)
{
    // CRC-16/DNP
    crc16_t crc16;
    crc16_init(&crc16, 0x3D65, 0x0000, true, true, 0xFFFF);
    for(size_t i=0; i<length; ++i)
    {
        crc16_step(&crc16, data[i]);
    }
    return crc16_finialize(&crc16, false);
}

uint16_t crc16_x25(const uint8_t *data, size_t length)
{
    // CRC-16/X25
    crc16_t crc16;
    crc16_init(&crc16, 0x1021, 0xFFFF, true, true, 0xFFFF);
    for(size_t i=0; i<length; ++i)
    {
        crc16_step(&crc16, data[i]);
    }
    return crc16_finialize(&crc16, false);
}

uint16_t crc16_bambu_bus(const uint8_t *data, size_t length)
{
    // CRC-16/BAMBU BUS
    crc16_t crc16;
    crc16_init(&crc16, 0x1021, 0x913D, true, true, 0x0000);
    for(size_t i=0; i<length; ++i)
    {
        crc16_step(&crc16, data[i]);
    }
    return crc16_finialize(&crc16, false);
}
