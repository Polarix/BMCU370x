#ifndef _INCLUDE_DEBUG_LOG_DRV_H_
#define _INCLUDE_DEBUG_LOG_DRV_H_
//===========================================================//
//= Include files.                                          =//
//===========================================================//
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

//===========================================================//
//= Macro definition.                                       =//
//===========================================================//
#define DEBUG_USART_EN

#ifdef DEBUG_USART_EN
#define TRC_LOG(FMT, ...)   dbg_log_fmt("[T]" FMT "\n", ##__VA_ARGS__ )
#define INF_LOG(FMT, ...)   dbg_log_fmt("[I]" FMT "\n", ##__VA_ARGS__ )
#define DBG_LOG(FMT, ...)   dbg_log_fmt("[D]" FMT "\n", ##__VA_ARGS__ )
#define WRN_LOG(FMT, ...)   dbg_log_fmt("[W]" FMT "\n", ##__VA_ARGS__ )
#define ERR_LOG(FMT, ...)   dbg_log_fmt("[E]" FMT "\n", ##__VA_ARGS__ )
#define RAW_LOG(B, N)       dbg_log_raw(B, N)
#define TXT_LOG(logs)       dbg_log_text(logs)
#else
#define TRC_LOG(FMT, ...)
#define INF_LOG(FMT, ...)
#define DBG_LOG(FMT, ...)
#define WRN_LOG(FMT, ...)
#define ERR_LOG(FMT, ...)
#define RAW_LOG(B, N)
#define TXT_LOG(logs)
#endif /* DEBUG_USART_EN */

//===========================================================//
//= Public function declaration.                            =//
//===========================================================//
#ifdef __cplusplus
extern "C"
{
#endif

void debug_init(void);
void dbg_log_fmt(const char* fmt, ...);
void dbg_log_text(const char* txt);
void dbg_log_raw(const void *data, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* _INCLUDE_DEBUG_LOG_DRV_H_ */

