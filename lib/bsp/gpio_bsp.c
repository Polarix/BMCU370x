//===========================================================//
//= Include files.                                          =//
//===========================================================//
#include <gpio_bsp.h>
#include <ch32v20x_usart.h>
#include <ch32v20x_dma.h>
#include <ch32v20x_gpio.h>

//===========================================================//
//= Static function declare.                                =//
//===========================================================//

//===========================================================//
//= Static variable defition.                               =//
//===========================================================//
static GPIO_InitTypeDef s_gpio_input_init_struct;
static GPIO_InitTypeDef s_gpio_output_init_struct;

//===========================================================//
//= Function definition.                                    =//
//===========================================================//
void gpio_bsp_init(void)
{
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_StructInit(&s_gpio_input_init_struct);
    GPIO_StructInit(&s_gpio_output_init_struct);
    
    s_gpio_input_init_struct.GPIO_Pin = GPIO_Pin_10;
    s_gpio_input_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
    s_gpio_input_init_struct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &s_gpio_input_init_struct);
}

bool gpio_bsp_set_dir(void);
void gpio_bsp_get(const void* data, uint32_t len);
