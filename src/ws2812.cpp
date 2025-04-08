#include <WS2812.h>
#include <stdlib.h>
#include <Arduino.h>

#define RGB_H() m_gpio_port->BSHR = m_pin
#define RGB_L() m_gpio_port->BCR = m_pin

void CWS2812::init(uint8_t index, uint32_t _GPIO_pin)
{
    m_index = index;
    m_rgb_buf = new uint32_t[m_index * 3];
    m_gpio_pin = _GPIO_pin;
    m_gpio_port = get_GPIO_Port(CH_PORT(digitalPinToPinName(m_gpio_pin)));
    m_pin = CH_GPIO_PIN(digitalPinToPinName(m_gpio_pin));
    RGB_L();

    pinMode(_GPIO_pin, OUTPUT);
    clear();
}
CWS2812::~CWS2812(void)
{
    delete m_rgb_buf;
}

void CWS2812::clear(void)
{
    uint8_t i;
    for (i = 0; i < m_index * 3; i++)
    {
        m_rgb_buf[i] = 0;
    }
}
void CWS2812::reset(void)
{
    RGB_L();
    delayMicroseconds(50);
}

void CWS2812::updata(void)
{
    int i, j;
    int max = m_index * 3;
    uint32_t DATA;
    for (i = 0; i < max; i++)
    {
        DATA = m_rgb_buf[i];
        for (j = 0; j < 24; j++)
        {
            if ((DATA >> j) & 0x01)
                RGB_H();
            else
                RGB_L();
            __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
            __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
            __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
            __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();//@144Mhz
        }
    }
    reset();
}
void CWS2812::set_rgb(uint8_t R, uint8_t G, uint8_t B, uint8_t index)
{
    uint32_t DATA = 0;
    int i;
    for (i = 0; i < 8; i++)
    {
        DATA <<= 3;
        DATA |= 1;
        DATA |= ((G >> i) & 1) << 1;
    }
    m_rgb_buf[index + 0] = DATA;
    DATA = 0;
    for (i = 0; i < 8; i++)
    {
        DATA <<= 3;
        DATA |= 1;
        DATA |= ((R >> i) & 1) << 1;
    }
    m_rgb_buf[index + 1] = DATA;
    DATA = 0;
    for (i = 0; i < 8; i++)
    {
        DATA <<= 3;
        DATA |= 1;
        DATA |= ((B >> i) & 1) << 1;
    }
    m_rgb_buf[index + 2] = DATA;
}
