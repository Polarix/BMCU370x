#include "debug_log_bsp.h"
#include <ch32v20x_usart.h>
#include <ch32v20x_dma.h>
#include <ch32v20x_gpio.h>
#include <string.h>

#ifdef DEBUG_LOG_EN

static void debug_bsp_init_usart(void);
static void debug_bsp_write_raw(const void *data, uint32_t len);

uint32_t stack[1000];
//mbed::Timer USB_debug_timer;
DMA_InitTypeDef s_debug_log_tx_dma_cfg;

void debug_log_bsp_init(void)
{
    debug_bsp_init_usart();
}

static void debug_bsp_init_usart(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure = {0};
    USART_InitTypeDef USART_InitStructure = {0};
    NVIC_InitTypeDef  NVIC_InitStructure = {0};

    RCC_APB1PeriphClockCmd( RCC_APB1Periph_USART3, ENABLE);
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    // USART3 TX-->B.10  RX-->B.11
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStructure);


    USART_InitStructure.USART_BaudRate = Debug_log_baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_9b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_Even;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    USART_Init(USART3, &USART_InitStructure);
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // Configure DMA1 channel 2 for USART3 TX
    s_debug_log_tx_dma_cfg.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DATAR;
    s_debug_log_tx_dma_cfg.DMA_MemoryBaseAddr = (uint32_t)0;
    s_debug_log_tx_dma_cfg.DMA_DIR = DMA_DIR_PeripheralDST;
    s_debug_log_tx_dma_cfg.DMA_Mode = DMA_Mode_Normal;
    s_debug_log_tx_dma_cfg.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    s_debug_log_tx_dma_cfg.DMA_MemoryInc = DMA_MemoryInc_Enable;
    s_debug_log_tx_dma_cfg.DMA_Priority = DMA_Priority_Low;
    s_debug_log_tx_dma_cfg.DMA_M2M = DMA_M2M_Disable;
    s_debug_log_tx_dma_cfg.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    s_debug_log_tx_dma_cfg.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    s_debug_log_tx_dma_cfg.DMA_BufferSize = 0;

    USART_Cmd(USART3, ENABLE);
}

uint64_t Debug_log_count64(void)
{
    return 0;
}

void Debug_log_time(void)
{

}

void debug_bsp_write_cstr(const char* cstr)
{
    uint32_t len = strlen((const char*)cstr);
    debug_bsp_write_raw((const char*)cstr, len);
}

void debug_bsp_write_data(const void* data, uint32_t len)
{
    debug_bsp_write_raw(data, len);
}

static void debug_bsp_write_raw(const void *data, uint32_t len)
{
    DMA_DeInit(DMA1_Channel2);
    // Configure DMA1 channel 2 for USART3 TX
    s_debug_log_tx_dma_cfg.DMA_MemoryBaseAddr = (uint32_t)data;
    s_debug_log_tx_dma_cfg.DMA_BufferSize = len;
    DMA_Init(DMA1_Channel2, &s_debug_log_tx_dma_cfg);
    DMA_Cmd(DMA1_Channel2, ENABLE);
    // 使能USART3 DMA发送
    USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
}


void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        //uint8_t x = 
        USART_ReceiveData(USART3);
        //USART_SendData(USART3, x);
    }
}

#endif /* DEBUG_LOG_EN */