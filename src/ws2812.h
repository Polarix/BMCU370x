#pragma once
#include "main.h"


class WS2812_class
{
public:
    uint32_t *RGB_buf;
	uint8_t m_num;
    uint32_t GPIO_pin;
    WS2812_class(){}
    ~WS2812_class();
    void init( uint8_t _num ,uint32_t GPIO_pin);
	void clear( void );
	void RST( void );
	void updata();
	void set_RGB(uint8_t R, uint8_t G, uint8_t B, uint8_t index);
private:
    GPIO_TypeDef * port;
    uint16_t pin;
};

