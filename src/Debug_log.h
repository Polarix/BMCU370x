#pragma once

#include "main.h"
#define Debug_log_on

#ifdef __cplusplus
extern "C"
{
#endif
extern void Debug_log_init();
extern uint64_t Debug_log_count64();
extern void Debug_log_time();
extern void Debug_log_write(const void *data);
extern void Debug_log_write_num(const void *data,int num);
#define DEBUG_time_log();
#ifdef Debug_log_on
#define DEBUG_INIT() Debug_log_init()
#define DEBUG_STR_OUT(logs) Debug_log_write(logs)
#define DEBUG_NUM_OUT(logs,num) Debug_log_write_num(logs,num)
#define DEBUG_TIME_STAMP_OUT() Debug_log_time()
#define DEBUG_TIME_GET() Debug_log_count64()
#else
#define DEBUG_INIT() ;
#define DEBUG_STR_OUT(logs); ;
#define DEBUG_NUM_OUT(logs,num); ;
#define DEBUG_TIME_GET(); ;
#endif
#ifdef __cplusplus
}
#endif
