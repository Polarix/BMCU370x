#ifndef _INCLUDE_ON_CHIP_FLASH_H_

#include "ch32v20x_flash.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C"{
#endif

bool on_chip_flash_save(const void *buf, uint32_t length, uint32_t address);

#ifdef __cplusplus
}
#endif

#endif /* _INCLUDE_ON_CHIP_FLASH_H_ */
