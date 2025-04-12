#ifndef _INCLUDE_DEBUG_LOG_BSP_H_
#define _INCLUDE_DEBUG_LOG_BSP_H_

#include <stdint.h>
#include <stddef.h>

#define DEBUG_LOG_EN
#define Debug_log_baudrate 115200

#ifdef __cplusplus
extern "C"
{
#endif

void debug_log_bsp_init(void);
uint64_t Debug_log_count64();
void Debug_log_time(void);
void debug_bsp_write_cstr(const char* cstr);
void debug_bsp_write_data(const void* data, uint32_t len);

#define DEBUG_time_log();
#ifdef DEBUG_LOG_EN
#define DEBUG_INIT() debug_log_bsp_init()
#define DEBUG_RAW(B, N) debug_bsp_write_data(B, N)
#define DEBUG_TIME_STAMP() Debug_log_time()
#define DEBUG_get_time() Debug_log_count64()
#else
#define DEBUG_INIT() ;
#define DEBUG(logs) ;
#define DEBUG_num(logs,num) ;
#define DEBUG_time() ;
#endif

#ifdef __cplusplus
}
#endif

#endif /* _INCLUDE_DEBUG_LOG_BSP_H_ */