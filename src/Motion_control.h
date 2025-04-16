#ifndef _INCLUDE_MC_MODEL_H_
#define _INCLUDE_MC_MODEL_H_
//===========================================================//
//= Include files.                                          =//
//===========================================================//
#include "main.h"

//===========================================================//
//= Public function declaration.                            =//
//===========================================================//
#ifdef __cplusplus
extern "C"
{
#endif

extern void Motion_control_init();
extern void Motion_control_set_PWM(uint8_t CHx,int PWM);
extern void Motion_control_run(int error);

#ifdef __cplusplus
}
#endif

#endif _INCLUDE_MC_MODEL_H_
