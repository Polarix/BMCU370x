#ifndef _INCLUDE_DEBUG_LOG_H_
#define _INCLUDE_DEBUG_LOG_H_
//===========================================================//
//= Include files.                                          =//
//===========================================================//
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

//===========================================================//
//= Macro definition.                                       =//
//===========================================================//
#define DEBUG_LOG_BAUDRATE  (115200)

//===========================================================//
//= Public function declaration.                            =//
//===========================================================//
#ifdef __cplusplus
extern "C"
{
#endif

void debug_log_bsp_init(void);
void debug_log_bsp_send_data(const void* data, uint32_t len);
bool debug_log_bsp_isbusy(void);

#ifdef __cplusplus
}
#endif

#endif /* _INCLUDE_DEBUG_LOG_H_ */