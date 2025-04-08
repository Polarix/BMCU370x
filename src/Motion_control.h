#ifndef _INCLUDE_MOTION_CTRL_H_
#define _INCLUDE_MOTION_CTRL_H_

#include <stdint.h>
#include <stddef.h>

extern void motion_control_init(void);
extern void motion_control_ticks_handler(int error);

#endif /* _INCLUDE_MOTION_CTRL_H_ */
