#include <time_stamp.h>
#include <clock.h> /* 此文件依赖Arduino组件，移植时需要自行配置Systick定时器。 */

/* Arduino的库函数中，毫秒时间戳是uint32的计数，最大值是4294967296 */
#define TIME_STAMP_VAL_MAX      (UINT32_MAX)
#define TIME_STAMP64_VAL_MAX    (UINT64_MAX)

/* 获取当前系统运行的时间戳，单位ms */
uint32_t get_monotonic_timestamp_ms(void)
{
    /* getCurrentMillis函数是Arduino的库函数。 */
    uint32_t time_stamp_ms = getCurrentMillis();
    return time_stamp_ms;
}

/* 获取时间差，考虑时间戳环绕的情况 */
uint32_t get_timestamp_interval(uint32_t before, uint32_t after)
{
    uint32_t elapsed;
    if(after > before)
    {
        elapsed = after - before;
    }
    else
    {
        elapsed = TIME_STAMP_VAL_MAX - before + after + 1;
    }

    return elapsed;
}

uint64_t get_monotonic_timestamp64_ms(void)
{
    /* GetTick函数是Arduino的库函数，返回64位时间戳。 */
    return GetTick();
}

uint64_t get_timestamp_interval64(uint64_t before, uint64_t after)
{
    uint64_t elapsed;
    if(after > before)
    {
        elapsed = after - before;
    }
    else
    {
        elapsed = TIME_STAMP64_VAL_MAX - before + after + 1;
    }

    return elapsed;
}