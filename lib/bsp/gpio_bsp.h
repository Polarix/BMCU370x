#ifndef _INCLUDE_GPIO_BSP_H_
#define _INCLUDE_GPIO_BSP_H_
//===========================================================//
//= Include files.                                          =//
//===========================================================//
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

//===========================================================//
//= Macro definition.                                       =//
//===========================================================//

//===========================================================//
//= Data type definition.                                   =//
//===========================================================//
typedef enum _e_gpio_id_
{
    GPIO_ID_PULL_CH1 = 0,
    GPIO_ID_PULL_CH2,
    GPIO_ID_PULL_CH3,
    GPIO_ID_PULL_CH4,
    GPIO_ID_ONLINE_CH1,
    GPIO_ID_ONLINE_CH2,
    GPIO_ID_ONLINE_CH3,
    GPIO_ID_ONLINE_CH4,
    GPIO_ID_MAX
}gpio_id_t;

//===========================================================//
//= Public function declaration.                            =//
//===========================================================//
#ifdef __cplusplus
extern "C"
{
#endif

void gpio_bsp_init(void);
uint8_t gpio_bsp_read_gpio(gpio_id_t gpio_id);

#ifdef __cplusplus
}
#endif

#endif /* _INCLUDE_GPIO_BSP_H_ */