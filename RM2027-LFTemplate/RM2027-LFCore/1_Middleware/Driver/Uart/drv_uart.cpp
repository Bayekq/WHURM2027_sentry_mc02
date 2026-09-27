/**
 * @brief uart驱动层
 * @author Bayekq
 */
#include "drv_uart.h"
#include "arm_math.h"

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

