//===========================================================//
//= Include files.                                          =//
//===========================================================//
#include <debug_log_bsp.h>
#include <ch32v20x_usart.h>
#include <ch32v20x_dma.h>
#include <ch32v20x_gpio.h>

//===========================================================//
//= Static function declare.                                =//
//===========================================================//
static void debug_log_bsp_init_dma(void);

//===========================================================//
//= Static variable defition.                               =//
//===========================================================//
static DMA_InitTypeDef Debug_log_DMA_InitStructure;
static volatile uint8_t s_dma_tx_busy = 0;

void debug_log_bsp_init(void)
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


    USART_InitStructure.USART_BaudRate = DEBUG_LOG_BAUDRATE;
    USART_InitStructure.USART_WordLength = USART_WordLength_9b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_Even;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    USART_Init(USART3, &USART_InitStructure);
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

    debug_log_bsp_init_dma();

    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART3, ENABLE);

    /* 使能 USART3 的 DMA 发送请求 */
    USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
}

static void debug_log_bsp_init_dma(void)
{
    // Configure DMA1 channel 2 for USART3 TX
    Debug_log_DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DATAR;
    Debug_log_DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)0;
    Debug_log_DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    Debug_log_DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    Debug_log_DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    Debug_log_DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    Debug_log_DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
    Debug_log_DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    Debug_log_DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    Debug_log_DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    Debug_log_DMA_InitStructure.DMA_BufferSize = 0;

    DMA_Init(DMA1_Channel2, &Debug_log_DMA_InitStructure);
    // 使能 DMA 传输完成中断
    DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);   // 关键步骤！
    DMA_Cmd(DMA1_Channel2, ENABLE);                   // 启动 DMA 通道
}

void debug_log_bsp_send_data(const void* data, uint32_t len)
{
    /* 等待上一次DMA传输完成 */
    while(!DMA_GetFlagStatus((DMA1_FLAG_TC2)));
    DMA_Cmd(DMA1_Channel2, DISABLE);
    DMA_DeInit(DMA1_Channel2);
    // Configure DMA1 channel 2 for USART3 TX
    Debug_log_DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)data;
    Debug_log_DMA_InitStructure.DMA_BufferSize = len;
    DMA_Init(DMA1_Channel2, &Debug_log_DMA_InitStructure);
    DMA_Cmd(DMA1_Channel2, ENABLE);
    // 使能USART3 DMA发送
    s_dma_tx_busy = 1;
    // USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
}

void DMA1_Channel2_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_IT_TC2))
    {
        DMA_ClearITPendingBit(DMA1_IT_TC2);    // 清除中断标志
        s_dma_tx_busy = 0;
    }
}

void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(USART3, USART_IT_RXNE))
    {
        USART_ClearFlag(USART3, USART_IT_RXNE);
        // USART_ReceiveData(USART3);
    }
    if(USART_GetITStatus(USART3, USART_IT_IDLE))
    {
        USART_ClearFlag(USART3, USART_IT_IDLE);
    }
}

bool debug_log_bsp_isbusy(void)
{
    return s_dma_tx_busy;
}
