#ifndef _INCLUDE_MOTION_CTRL_H_
#define _INCLUDE_MOTION_CTRL_H_

#include "main.h"


extern void Motion_control_init();
extern void Motion_control_set_PWM(uint8_t CHx,int PWM);
extern void motion_control_ticks_handler(int error);

#endif /* _INCLUDE_MOTION_CTRL_H_ */
