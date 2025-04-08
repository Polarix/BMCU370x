#ifndef _INCLUDE_PWM_OUT_BSP_H_
#define _INCLUDE_PWM_OUT_BSP_H_

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

void pwm_out_bsp_init(void);
void pwm_out_bsp_set(uint8_t CHx, int PWM);

#ifdef __cplusplus
}
#endif

#endif /* _INCLUDE_PWM_OUT_BSP_H_ */
