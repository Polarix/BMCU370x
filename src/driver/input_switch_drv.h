#ifndef _INCLUDE_INPUT_SWITCH_DRIVER_H_
#define _INCLUDE_INPUT_SWITCH_DRIVER_H_

#include <stdint.h>
#include <stddef.h>

typedef enum _e_pin_input_func_
{
    PIN_FUNC_UNKNOWN = 0,
    PIN_FUNC_PULL,
    PIN_FUNC_ONLINE,
}pin_input_func_t;

typedef enum _e_filament_channel_
{
    FILAMENT_CH0 = 0,
    FILAMENT_CH1,
    FILAMENT_CH2,
    FILAMENT_CH3,
}filament_ch_idx;

#ifdef __cplusplus
extern "C" {
#endif

void input_driver_init(void);


#ifdef __cplusplus
}
#endif

#endif /* _INCLUDE_INPUT_SWITCH_DRIVER_H_ */
