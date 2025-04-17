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
void bambu_bus_queue_init(void)
{
    queue_init(&s_bambu_bus_queue, s_bambu_bus_queue_content, BAMBU_BUS_QUEUE_SIZE);
    rs_485_bsp_register_byte_rev_callback(bambu_bus_queue_on_byte_received);
    bambu_bus_init_usart();
}

static void bambu_bus_queue_on_byte_received(uint8_t byte)
{
    /* 队列已满的话，后续数据将被忽略。 */
    enqueue(&s_bambu_bus_queue, byte);
}

bool bambu_bus_read_byte(uint8_t* byte)
{
    return dequeue(&s_bambu_bus_queue, byte);
}

uint32_t bambu_bus_queue_read(uint8_t* buffer, uint32_t size)
{
    /* 读取数据块 */
    return dequeue_block(&s_bambu_bus_queue, buffer, size);
}
