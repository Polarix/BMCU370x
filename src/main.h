#pragma once
#include "ch32v20x.h"
#include <Arduino.h>
#include "stdlib.h"
#include "debug_log.h"
#include "Flash_saves.h"
#include "Motion_control.h"
#include "BambuBus.h"
#include "time_stamp.h"
#include "many_soft_AS5600.h"

#define delay_any_us(time)\
{\
    const uint64_t _delay_any_div_time =(uint64_t)(8000000.0/time);\
    SysTick->SR &= ~(1 << 0);\
    SysTick->CMP = SystemCoreClock/_delay_any_div_time;\
    SysTick->CTLR |= (1 << 5) |(1 << 4)| (1 << 0);\
\
    while(!(SysTick->SR & 1));\
    SysTick->CTLR &= ~(1 << 0);\
}

#define delay_any_ms(time)\
{\
    const uint64_t _delay_any_div_time =(uint64_t)(80000.0/time);\
    SysTick->SR &= ~(1 << 0);\
    SysTick->CMP = SystemCoreClock/_delay_any_div_time;\
    SysTick->CTLR |= (1 << 5) |(1 << 4)| (1 << 0);\
\
    while(!(SysTick->SR & 1));\
    SysTick->CTLR &= ~(1 << 0);\
}

extern void RGB_set(uint8_t CHx,uint8_t R, uint8_t G, uint8_t B);

//#include "AMCU.h"
