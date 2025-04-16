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


//===========================================================//
//= Public function declaration.                            =//
//===========================================================//
#ifdef __cplusplus
extern "C"
{
#endif

void gpio_bsp_init(void);
bool gpio_bsp_set_dir(void);
void gpio_bsp_get(const void* data, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* _INCLUDE_GPIO_BSP_H_ */