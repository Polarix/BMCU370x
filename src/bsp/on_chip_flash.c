#include "on_chip_flash.h"

// #define PAGE_WRITE_START_ADDR ((uint32_t)0x08008000) /* Start from 32K */
// #define PAGE_WRITE_END_ADDR ((uint32_t)0x08009000)   /* End at 36K */
#define FLASH_PAGE_SIZE 4096
#define FLASH_PAGES_TO_BE_PROTECTED FLASH_WRProt_Pages60to63
/* Global Variable */
volatile FLASH_Status s_flash_status = FLASH_COMPLETE;

/*********************************************************************
 * @fn      Flash_Test
 *
 * @brief   Flash Program Test.
 *
 * @return  none
 */
bool on_chip_flash_save(const void *buf, uint32_t length, uint32_t address)
{
    uint32_t end_address = address + length;
    uint32_t erase_counter = 0;
    uint32_t address_i = 0;
    uint32_t page_num = length / FLASH_PAGE_SIZE;
    uint16_t *data_ptr=(uint16_t *)buf;

    __disable_irq(); // 禁用中断
    FLASH_Unlock();

    FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_WRPRTERR);

    for (erase_counter = 0; (erase_counter < page_num) && (s_flash_status == FLASH_COMPLETE); erase_counter++)
    {
        s_flash_status = FLASH_ErasePage(address + (FLASH_PAGE_SIZE * erase_counter)); // Erase 4KB

        if (s_flash_status != FLASH_COMPLETE)
            return false;
    }

    address_i = address;
    while ((address_i < end_address) && (s_flash_status == FLASH_COMPLETE))
    {
        s_flash_status = FLASH_ProgramHalfWord(address_i, *data_ptr);
        address_i = address_i + 2;
        data_ptr++;
    }

    FLASH_Lock();
    __enable_irq();

    return true;
}