//===========================================================//
//= Include files.                                          =//
//===========================================================//
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <lfs.h>
#include <lfs_adapter.h>
#include <ch32v20x_flash.h>

//===========================================================//
//= Macro definition.                                       =//
//===========================================================//
/* Storage flash address range is 0xC000 - 0xFFFF, total 16K  */
#define STORAGE_AREA_BASE_ADDR      ((uint32_t)0x0800C000)
#define STORAGE_AREA_MAX_ADDR       ((uint32_t)0x0800FFFF)
#define FLASH_PAGE_SIZE (4096)
#define FLASH_PAGE_NUM  (4)

//===========================================================//
//= Static function declare.                                =//
//===========================================================//
static int user_flash_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size);
static int user_flash_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size);
static int user_flash_erase(const struct lfs_config *c, lfs_block_t block);
static int user_flash_sync(const struct lfs_config *c);

//===========================================================//
//= Static variable defition.                               =//
//===========================================================//
#include "lfs.h"

// Flash操作结构体
static const struct lfs_config s_lfs_cfg = {
    .read  = user_flash_read,
    .prog  = user_flash_prog,
    .erase = user_flash_erase,
    .sync  = user_flash_sync,

    .read_size = 16,                // 读取粒度（通常等于Flash最小读取单位）
    .prog_size = 4,                 // 编程粒度（半字编程则为2）
    .block_size = FLASH_PAGE_SIZE,  // 块大小=Flash页大小（根据实际情况调整）
    .block_count = FLASH_PAGE_NUM,  // 块数量=总空间/block_size（64KB/2KB=32）
    .cache_size = 16,
    .lookahead_size = 16,
    .block_cycles = 500,            // 磨损均衡周期
};

static lfs_t s_lfs_instance;

//===========================================================//
//= Function definition.                                    =//
//===========================================================//
bool fs_init(void)
{
    bool result = false;
    int lfs_err = lfs_mount(&s_lfs_instance, &s_lfs_cfg);
    if(lfs_err)
    {
        // 挂载失败，尝试格式化
        lfs_format(&s_lfs_instance, &s_lfs_cfg);
        lfs_err = lfs_mount(&s_lfs_instance, &s_lfs_cfg);
    }

    if(LFS_ERR_OK == lfs_err)
    {
        result = true;
    }

    return result;
}

static int user_flash_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size)
{
    uint32_t addr = STORAGE_AREA_BASE_ADDR + block * c->block_size + off;
    memcpy(buffer, (void*)addr, size);
    return 0;
}

static int user_flash_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size)
{
    uint32_t addr = STORAGE_AREA_BASE_ADDR + block * c->block_size + off;
    FLASH_Unlock();
    for (size_t i = 0; i < size; i += 2)
    {
        // 以半字形式写入，需确保数据对齐
        uint16_t data = *((uint16_t*)((uint8_t*)buffer + i));
        if (FLASH_ProgramHalfWord(addr + i, data) != FLASH_COMPLETE)
        {
            FLASH_Lock();
            return LFS_ERR_IO;
        }
    }
    FLASH_Lock();
    return 0;
}

static int user_flash_erase(const struct lfs_config *c, lfs_block_t block)
{
    uint32_t addr = STORAGE_AREA_BASE_ADDR + block * c->block_size;
    
    FLASH_Unlock();
    if (FLASH_ErasePage(addr) != FLASH_COMPLETE)
    {
        FLASH_Lock();
        return LFS_ERR_IO;
    }
    FLASH_Lock();
    return 0;
}

static int user_flash_sync(const struct lfs_config *c)
{
    // 若Flash操作需要缓存同步，可在此实现
    return 0;
}
