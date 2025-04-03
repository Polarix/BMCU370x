#ifndef _INCLUDE_FLASH_SAVE_H_

#include <stdint.h>
#include <stddef.h>

void flash_storage_init(void);
bool flash_storage_save(void*buf,uint32_t length,uint32_t address);

#endif /* _INCLUDE_FLASH_SAVE_H_ */
