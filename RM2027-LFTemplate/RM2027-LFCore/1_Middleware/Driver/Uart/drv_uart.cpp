/**
 * @brief uart驱动层
 * @author Bayekq
 */
#include "drv_uart.h"
#include "arm_math.h"

extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef hdma_uart5_rx;

/**
 * @brief 使用uart发送数据
 * @param huart 
 * @param pData 
 * @param Size 
 * @param Timeout 
 * @return 
 */
UartSendState_e UartSendTxMessage(
    UART_HandleTypeDef * huart, uint8_t * pData, uint16_t Size, uint32_t Timeout)
{
    uint8_t cnt = 5;  //最大重发次数
    HAL_StatusTypeDef status = HAL_UART_Transmit(huart, pData, Size, Timeout);
    while (cnt-- && status != HAL_OK) {
        status = HAL_UART_Transmit(huart, pData, Size, Timeout);
    }

    if (status == HAL_OK) {
        return UART_SEND_OK;
    }
    return UART_SEND_FAIL;
}

/**
 * @brief 遥控器 UART5 DMA 接收初始化
 */
void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    SET_BIT(huart5.Instance->CR3, USART_CR3_DMAR);
    __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);

    __HAL_DMA_DISABLE(&hdma_uart5_rx);
    while (((DMA_Stream_TypeDef *)hdma_uart5_rx.Instance)->CR & DMA_SxCR_EN) {
        __HAL_DMA_DISABLE(&hdma_uart5_rx);
    }

    ((DMA_Stream_TypeDef *)hdma_uart5_rx.Instance)->PAR = (uint32_t)&(UART5->RDR);
    ((DMA_Stream_TypeDef *)hdma_uart5_rx.Instance)->M0AR = (uint32_t)rx1_buf;
    ((DMA_Stream_TypeDef *)hdma_uart5_rx.Instance)->M1AR = (uint32_t)rx2_buf;
    ((DMA_Stream_TypeDef *)hdma_uart5_rx.Instance)->NDTR = dma_buf_num;
    SET_BIT(((DMA_Stream_TypeDef *)hdma_uart5_rx.Instance)->CR, DMA_SxCR_DBM);

    __HAL_DMA_ENABLE(&hdma_uart5_rx);
}

void RC_unable(void)
{
    __HAL_UART_DISABLE(&huart5);
}

void RC_restart(uint16_t dma_buf_num)
{
    __HAL_UART_DISABLE(&huart5);
    __HAL_DMA_DISABLE(&hdma_uart5_rx);

    ((DMA_Stream_TypeDef *)hdma_uart5_rx.Instance)->NDTR = dma_buf_num;

    __HAL_DMA_ENABLE(&hdma_uart5_rx);
    __HAL_UART_ENABLE(&huart5);
}
