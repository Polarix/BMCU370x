#include "Flash_saves.h"
#include <bsp/on_chip_flash.h>

void flash_storage_init(void)
{

}

bool flash_storage_save(void *buf, uint32_t length, uint32_t address)
{
    return on_chip_flash_save(buf, length, address);
}