//===========================================================//
//= Include files.                                          =//
//===========================================================//
#include <raw_queue.h>
#include <string.h>
#include <ch32v20x.h>
#include <core_riscv.h>

//===========================================================//
//= Function definition.                                    =//
//===========================================================//
static inline void disable_irq(void)
{
    __disable_irq();
}

static inline void enable_irq(void)
{
    __enable_irq();
}

void queue_init(raw_queue_t* queue, uint8_t* buffer, uint32_t size)
{
    queue->buffer = buffer;
    queue->size = size;
    queue->head = 0;
    queue->tail = 0;
}

bool enqueue(raw_queue_t* queue, uint8_t data)
{
    bool success = false;
    uint16_t next_head = (queue->head + 1) % (queue->size);

    /* 检查缓冲区是否已满 */
    if (next_head != queue->tail)
    {
        queue->buffer[queue->head] = data;
        queue->head = next_head;
        success = true;
    }

    return success;
}

uint32_t enqueue_block(raw_queue_t* queue, const void* data, uint32_t data_len)
{
    uint32_t free_space;
    const uint32_t head = queue->head;
    const uint32_t tail = queue->tail;

    /* 计算可用空间 */
    if (head >= tail)
    {
        free_space = ((queue->size) - 1) - (head - tail);
    }
    else
    {
        free_space = (tail - head) - 1;
    }

    if (free_space == 0) return 0;

    /* 计算实际能写入的字节数 */
    uint32_t to_write = (data_len < free_space) ? data_len : free_space;
    uint32_t first_chunk = (queue->size) - head;

    if (first_chunk >= to_write)
    {
        /* 不需要环绕 */
        memcpy((void*)&queue->buffer[head], data, to_write);
        queue->head = (head + to_write) & ((queue->size)-1);
    }
    else
    {
        /* 需要环绕 */
        memcpy((void*)&queue->buffer[head], data, first_chunk);
        memcpy((void*)queue->buffer, (uint8_t*)data + first_chunk, to_write - first_chunk);
        queue->head = to_write - first_chunk;
    }

    return to_write;
}

bool dequeue(raw_queue_t* queue, uint8_t *data)
{
    bool success = false;

    /* 临时禁用中断以确保原子操作 */
    disable_irq();

    /* 检查缓冲区是否为空 */
    if (queue->head != queue->tail)
    {
        *data = queue->buffer[queue->tail];
        queue->tail = (queue->tail + 1) % (queue->size);
        success = true;
    }

    enable_irq();
    return success;
}

uint32_t dequeue_block(raw_queue_t* queue, void* data, uint32_t len)
{
    uint32_t bytes_read = 0;

    disable_irq();

    uint32_t available;
    if (queue->head >= queue->tail)
    {
        available = queue->head - queue->tail;
    }
    else
    {
        available = (queue->size) - queue->tail;
    }

    uint32_t to_read = (len < available) ? len : available;

    if (to_read > 0)
    {
        memcpy(data, (const uint8_t *)&queue->buffer[queue->tail], to_read);
        queue->tail = (queue->tail + to_read) % (queue->size);
        bytes_read = to_read;

        /* 如果还有剩余空间且需要更多数据（环绕情况）*/
        if (bytes_read < len && queue->head != queue->tail)
        {
            uint32_t remaining = len - bytes_read;
            uint32_t second_part = (queue->head < remaining) ? queue->head : remaining;

            memcpy((uint8_t*)data + bytes_read, (const void*)queue->buffer, second_part);
            queue->tail = second_part;
            bytes_read += second_part;
        }
    }

    enable_irq();
    return bytes_read;
}

/* 从主程序调用 - 检查缓冲区是否为空 */
bool queue_is_empty(raw_queue_t* queue)
{
    bool isEmpty;

    // 临时禁用中断以确保原子操作
    disable_irq();
    isEmpty = (queue->head == queue->tail);
    enable_irq();

    return isEmpty;
}

/* 从主程序调用 - 获取缓冲区中可用数据量 */
uint32_t queue_get_remaining(raw_queue_t* queue)
{
    uint32_t remaining = 0;
    uint32_t head, tail;

    // 临时禁用中断以确保原子操作
    disable_irq();
    head = queue->head;
    tail = queue->tail;
    enable_irq();

    if (head >= tail)
    {
        remaining = head - tail;
    }
    else
    {
        remaining = (queue->size) - (tail - head);
    }

    return remaining;
}
