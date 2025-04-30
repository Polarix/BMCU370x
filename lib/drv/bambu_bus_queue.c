//===========================================================//
//= Include files.                                          =//
//===========================================================//
#include <bambu_bus_queue.h>
#include <rs485_bsp.h>
#include <string.h>

//===========================================================//
//= Macro definition.                                       =//
//===========================================================//
#define BAMBU_BUS_QUEUE_SIZE        (1024)

//===========================================================//
//= Static function declare.                                =//
//===========================================================//
static void bambu_bus_queue_on_byte_received(uint8_t byte);

//===========================================================//
//= Static variable defition.                               =//
//===========================================================//
static raw_queue_t  s_bambu_bus_queue;
static uint8_t      s_bambu_bus_queue_content[BAMBU_BUS_QUEUE_SIZE];

//===========================================================//
//= Function definition.                                    =//
//===========================================================//
/* 初始化接收队列 */
void bambu_bus_queue_init(void)
{
    /* 初始化队列结构 */
    queue_init(&s_bambu_bus_queue, s_bambu_bus_queue_content, BAMBU_BUS_QUEUE_SIZE);
    /* 注册数据接收回调 */
    rs_485_bsp_register_byte_rev_callback(bambu_bus_queue_on_byte_received);
    /* 初始化4856串口 */
    bambu_bus_init_usart();
}

/* 接收到数据后的回调处理 */
static void bambu_bus_queue_on_byte_received(uint8_t byte)
{
    /* 队列已满的话，后续数据将被忽略。 */
    enqueue(&s_bambu_bus_queue, byte);
}

/* 从接收队列中读取一个字节 */
bool bambu_bus_read_byte(uint8_t* byte)
{
    return dequeue(&s_bambu_bus_queue, byte);
}

/* 从接收队列中读取一个块数据 */
uint32_t bambu_bus_queue_read(uint8_t* buffer, uint32_t size)
{
    /* 读取数据块 */
    return dequeue_block(&s_bambu_bus_queue, buffer, size);
}
