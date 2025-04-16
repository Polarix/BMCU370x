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

void mc_init(void);
void mc_ticks_handler(int error);

#ifdef __cplusplus
}
#endif

#endif /* _INCLUDE_MC_MODEL_H_ */
