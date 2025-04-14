#ifndef _INCLUDE_RUN_TIME_STAMP_MS_H_
#define _INCLUDE_RUN_TIME_STAMP_MS_H_

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C"
{
#endif

uint32_t get_monotonic_timestamp_ms(void);

uint64_t get_monotonic_timestamp64_ms(void);

uint32_t get_timestamp_interval(uint32_t before, uint32_t after);

uint64_t get_timestamp_interval64(uint64_t before, uint64_t after);

#ifdef __cplusplus
}
#endif

#endif /* _INCLUDE_RUN_TIME_STAMP_MS_H_ */
