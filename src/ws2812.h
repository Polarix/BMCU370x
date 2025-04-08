#ifndef _INCLUDE_CLASS_WS2812_DRV_H_
#define _INCLUDE_CLASS_WS2812_DRV_H_

#include <stdint.h>
#include <stddef.h>
#include <ch32v20x_gpio.h>

class CWS2812
{
public:
    uint32_t* m_rgb_buf;
	uint8_t m_index;
    uint32_t m_gpio_pin;
    explicit CWS2812(void){}
    virtual ~CWS2812(void);
    void init(uint8_t _num, uint32_t GPIO_pin);
	void clear(void);
	void reset(void);
	void updata(void);
	void set_rgb(uint8_t R, uint8_t G, uint8_t B, uint8_t index);
private:
    GPIO_TypeDef* m_gpio_port;
    uint16_t m_pin;
};

#endif /* _INCLUDE_CLASS_WS2812_DRV_H_ */
