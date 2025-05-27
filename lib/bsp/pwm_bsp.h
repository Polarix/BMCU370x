#ifndef _INCLUDE_MC_PWM_BSP_H_
#define _INCLUDE_MC_PWM_BSP_H_
//===========================================================//
//= Include files.                                          =//
//===========================================================//
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

//===========================================================//
//= Macro definition.                                       =//
//===========================================================//
#define PWM_VAL_MAX     (900)

//===========================================================//
//= Data type declare.                                      =//
//===========================================================//
typedef enum _e_channel_index_
{
    MOTOR_CH1 = 0,
    MOTOR_CH2,
    MOTOR_CH3,
    MOTOR_CH4,
}motor_ch_t;

//===========================================================//
//= Public function declaration.                            =//
//===========================================================//
#ifdef __cplusplus
extern "C"
{
#endif

void pwm_bsp_init(void);
void pwm_set_value(int channel, int value);

#ifdef __cplusplus
}
#endif

#endif /* _INCLUDE_MC_PWM_BSP_H_ */
