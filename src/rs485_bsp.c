#include <rs485_bsp.h>
#include <ch32v20x_usart.h>
#include <ch32v20x_dma.h>
#include <ch32v20x_gpio.h>

static DMA_InitTypeDef s_rs485_tx_dma_cfg;
static rs485_byte_rev_handle s_rs485_rev_byte_handler = NULL;

void bambu_bus_send_data(const uint8_t *data, uint16_t length)
{
    DMA_DeInit(DMA1_Channel4);
    // Configure DMA1 channel 4 for USART1 TX
    s_rs485_tx_dma_cfg.DMA_MemoryBaseAddr = (uint32_t)data;
    s_rs485_tx_dma_cfg.DMA_BufferSize = length;
    DMA_Init(DMA1_Channel4, &s_rs485_tx_dma_cfg);
    DMA_Cmd(DMA1_Channel4, ENABLE);
    GPIOA->BSHR = GPIO_Pin_12;
    // Enable USART1 DMA send
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
}

void bambu_bus_init_usart(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    USART_InitTypeDef USART_InitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    /* USART1 TX-->A.9   RX-->A.10 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; // TX
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; // RX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; // DE
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIOA->BCR = GPIO_Pin_12;

    USART_InitStructure.USART_BaudRate = 1250000;
    USART_InitStructure.USART_WordLength = USART_WordLength_9b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_Even;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    USART_Init(USART1, &USART_InitStructure);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART1, USART_IT_TC, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // Configure DMA1 channel 4 for USART1 TX
    s_rs485_tx_dma_cfg.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DATAR;
    s_rs485_tx_dma_cfg.DMA_MemoryBaseAddr = (uint32_t)0;
    s_rs485_tx_dma_cfg.DMA_DIR = DMA_DIR_PeripheralDST;
    s_rs485_tx_dma_cfg.DMA_Mode = DMA_Mode_Normal;
    s_rs485_tx_dma_cfg.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    s_rs485_tx_dma_cfg.DMA_MemoryInc = DMA_MemoryInc_Enable;
    s_rs485_tx_dma_cfg.DMA_Priority = DMA_Priority_VeryHigh;
    s_rs485_tx_dma_cfg.DMA_M2M = DMA_M2M_Disable;
    s_rs485_tx_dma_cfg.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    s_rs485_tx_dma_cfg.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    s_rs485_tx_dma_cfg.DMA_BufferSize = 0;

    USART_Cmd(USART1, ENABLE);
}

__attribute__((interrupt("WCH-Interrupt-fast")))
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        s_rs485_rev_byte_handler(USART_ReceiveData(USART1));
    }
    if (USART_GetITStatus(USART1, USART_IT_TC) != RESET)
    {
        USART_ClearITPendingBit(USART1, USART_IT_TC);
        GPIOA->BCR = GPIO_Pin_12;
    }
}

void rs_485_bsp_register_byte_rev_callback(rs485_byte_rev_handle callback)
{
    s_rs485_rev_byte_handler = callback;
}