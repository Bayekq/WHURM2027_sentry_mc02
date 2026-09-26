/**
 * @file drv_can.h
 * @author Bayekq (206858817@qq.com)
 * @brief 仿照USTC-RoboWalker改写的CAN通信初始化与配置流程
 * @version 0.1
 * @date 2026-9-25
 *
 * @copyright WHU-LuojiaFox (c) 2026
 *
 */

#ifndef DRV_CAN_H
#define DRV_CAN_H

/* Includes ------------------------------------------------------------------*/

#include "fdcan.h"
#include "stm32h7xx_hal.h"
#include <string.h>
#include "arm_math.h"
#include "cmsis_os.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief CAN通信接收回调函数数据类型
 *
 */
typedef void (*CAN_Callback)(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer);

/**
 * @brief CAN通信处理结构体
 *
 */
struct CAN_Manage_Object
{
    FDCAN_HandleTypeDef *CAN_Handler;
    CAN_Callback Callback_Function;

    // 与接收相关的数据
    FDCAN_RxHeaderTypeDef Rx_Header;
    uint8_t Rx_Buffer[64];

    // 接收时间戳
    uint64_t Rx_Timestamp;
};

/* Exported variables ---------------------------------------------------------*/

extern bool init_finished;

extern struct CAN_Manage_Object CAN1_Manage_Object;
extern struct CAN_Manage_Object CAN2_Manage_Object;
extern struct CAN_Manage_Object CAN3_Manage_Object;

extern uint8_t CAN1_0x1fe_Tx_Data[];
extern uint8_t CAN1_0x1ff_Tx_Data[];
extern uint8_t CAN1_0x200_Tx_Data[];
extern uint8_t CAN1_0x2fe_Tx_Data[];
extern uint8_t CAN1_0x2ff_Tx_Data[];
extern uint8_t CAN1_0x3fe_Tx_Data[];
extern uint8_t CAN1_0x4fe_Tx_Data[];

extern uint8_t CAN2_0x1fe_Tx_Data[];
extern uint8_t CAN2_0x1ff_Tx_Data[];
extern uint8_t CAN2_0x200_Tx_Data[];
extern uint8_t CAN2_0x2fe_Tx_Data[];
extern uint8_t CAN2_0x2ff_Tx_Data[];
extern uint8_t CAN2_0x3fe_Tx_Data[];
extern uint8_t CAN2_0x4fe_Tx_Data[];

extern uint8_t CAN3_0x1fe_Tx_Data[];
extern uint8_t CAN3_0x1ff_Tx_Data[];
extern uint8_t CAN3_0x200_Tx_Data[];
extern uint8_t CAN3_0x2fe_Tx_Data[];
extern uint8_t CAN3_0x2ff_Tx_Data[];
extern uint8_t CAN3_0x3fe_Tx_Data[];
extern uint8_t CAN3_0x4fe_Tx_Data[];

extern uint8_t CAN_Supercap_Tx_Data[];
extern uint8_t CAN_Board_Tx_Data[];

/* Exported function declarations ---------------------------------------------*/

void CAN_Init(FDCAN_HandleTypeDef *hfdcan, CAN_Callback Callback_Function);

uint8_t CAN_Transmit_Data(FDCAN_HandleTypeDef *hfdcan, uint16_t ID, uint8_t *Data, uint16_t Length);

#endif

/************************ COPYRIGHT(C) WHU-LuojiaFox **************************/