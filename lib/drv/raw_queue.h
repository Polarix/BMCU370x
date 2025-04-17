#ifndef _INCLUDED_RAW_QUEUE_H_
#define _INCLUDED_RAW_QUEUE_H_
//===========================================================//
//= Include files.                                          =//
//===========================================================//
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdbool.h>

//===========================================================//
//= Macro definition.                                       =//
//===========================================================//

//===========================================================//
//= Data type definition.                                   =//
//===========================================================//
typedef struct _st_raw_queue_
{
    volatile uint8_t* buffer;  /* 数据缓冲区 */
    volatile uint32_t size;
    volatile uint32_t head; /* 写入指针(由ISR修改) */
    volatile uint32_t tail; /* 读取指针(由主程序修改) */
}raw_queue_t;

//===========================================================//
//= Public function declaration.                            =//
//===========================================================//
#ifdef __cplusplus
extern "C"
{
#endif

void queue_init(raw_queue_t* queue, uint8_t* buffer, uint32_t size);
bool enqueue(raw_queue_t* queue, uint8_t data);
uint32_t enqueue_block(raw_queue_t* queue, const void* data, uint32_t data_len);
bool dequeue(raw_queue_t* queue, uint8_t *data);
uint32_t dequeue_block(raw_queue_t* queue, void* data, uint32_t len);
bool queue_is_empty(raw_queue_t* queue);
uint32_t queue_get_remaining(raw_queue_t* queue);

#ifdef __cplusplus
}
#endif

#endif // _INCLUDED_RAW_QUEUE_H_
