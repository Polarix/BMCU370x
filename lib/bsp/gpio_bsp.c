//===========================================================//
//= Include files.                                          =//
//===========================================================//
#include <gpio_bsp.h>
#include <ch32v20x_gpio.h>

//===========================================================//
//= Static function declare.                                =//
//===========================================================//

//===========================================================//
//= Static variable defition.                               =//
//===========================================================//
static GPIO_InitTypeDef s_gpio_input_init_struct;
static GPIO_InitTypeDef s_gpio_output_init_struct;
static struct
{
    GPIO_TypeDef*   port;
    uint16_t        pin;
}s_gpio_config[GPIO_ID_MAX] = 
{
    {GPIOB, GPIO_Pin_12}, // GPIO_ID_PULL_CH1
    {GPIOB, GPIO_Pin_13}, // GPIO_ID_PULL_CH2
    {GPIOB, GPIO_Pin_14}, // GPIO_ID_PULL_CH3
    {GPIOB, GPIO_Pin_15}, // GPIO_ID_PULL_CH4
    {GPIOC, GPIO_Pin_13}, // GPIO_ID_ONLINE_CH1
    {GPIOC, GPIO_Pin_14}, // GPIO_ID_ONLINE_CH2
    {GPIOC, GPIO_Pin_15}, // GPIO_ID_ONLINE_CH3
    {GPIOD, GPIO_Pin_0},  // GPIO_ID_ONLINE_CH4
};

//===========================================================//
//= Function definition.                                    =//
//===========================================================//
void gpio_bsp_init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_PD01, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);

    GPIO_StructInit(&s_gpio_input_init_struct);
    GPIO_StructInit(&s_gpio_output_init_struct);
    
    s_gpio_input_init_struct.GPIO_Pin = GPIO_Pin_10;
    s_gpio_input_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
    s_gpio_input_init_struct.GPIO_Mode = GPIO_Mode_IPU;
    for(int gpio_idx=0; gpio_idx<GPIO_ID_MAX; ++gpio_idx)
    {
        s_gpio_input_init_struct.GPIO_Pin = s_gpio_config[gpio_idx].pin;
        GPIO_Init(s_gpio_config[gpio_idx].port, &s_gpio_input_init_struct);
    }
}


/* 读取GPIO的输入状态 */
uint8_t gpio_bsp_read_gpio(gpio_id_t gpio_id)
{
    uint8_t val;
    if(gpio_id < GPIO_ID_MAX)
    {
        val = GPIO_ReadInputDataBit(s_gpio_config[gpio_id].port, s_gpio_config[gpio_id].pin);
    }
    else
    {
        val = 1;
    }
    return val;
}
