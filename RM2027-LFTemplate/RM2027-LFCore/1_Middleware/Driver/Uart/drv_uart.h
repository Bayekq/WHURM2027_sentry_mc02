
#ifndef BSP_UART_H
#define BSP_UART_H

#include "stm32h7xx_hal.h"
#include "usart.h"

typedef UART_HandleTypeDef huart_t;

typedef enum __UartSendState {
    UART_SEND_FAIL = 0,
    UART_SEND_OK,
} UartSendState_e;

extern UartSendState_e UartSendTxMessage(
    UART_HandleTypeDef * huart, uint8_t * pData, uint16_t Size, uint32_t Timeout);

#endif  // BSP_UART_H
