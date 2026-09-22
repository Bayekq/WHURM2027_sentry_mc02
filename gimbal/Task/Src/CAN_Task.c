
/**
  ******************************************************************************
  * @file           : CAN_Task.c
  * @brief          : CAN task
  * @author         : GrassFam Wang
  * @date           : 2025/1/22
  * @version        : v1.1
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "CAN_Task.h"
#include "Control_Task.h"
#include "INS_Task.h"
#include "Motor.h"
#include "bsp_can.h"
#include "Remote_Control.h"
#include "Control_Task.h"
#include "Minipc.h"
#include "Shoot.h"
#include "MiniPC_Task.h"

uint8_t *Yaw_Ange;
uint8_t *Yaw_Gyro;
uint8_t *Shoot_HeatCtrl_Qres;


/* USER CODE BEGIN Header_CAN_Task */
/**
* @brief Function implementing the StartCANTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN_Task */
  TickType_t CAN_Task_SysTick = 0;
 void CAN_Task(void const * argument)
{
	uint8_t CAN_FD_Tx_Buffer[64] = { 0 };
	for(;;)
  {
	  CAN_Task_SysTick = osKernelSysTick();
	  FDCAN3_TxFrame.Header.Identifier = 0X200;

	  Yaw_Ange = (uint8_t *)&INS_Info.Yaw_Angle;
	  Yaw_Gyro = (uint8_t *)&INS_Info.Yaw_Gyro;
	  Shoot_HeatCtrl_Qres = (uint8_t *)&Shoot_Info.Shoot_HeatCtrl.Qres;

	  CAN_FD_Tx_Buffer[0] = (uint8_t)((remote_ctrl.rc_lost) << 4 | (remote_ctrl.rc.s[0]) << 2 | (remote_ctrl.rc.s[1]));
	  CAN_FD_Tx_Buffer[1] = (uint8_t)(remote_ctrl.rc.ch[3] >> 8);
	  CAN_FD_Tx_Buffer[2] = (uint8_t)(remote_ctrl.rc.ch[3]);
	  CAN_FD_Tx_Buffer[3] = (uint8_t)(remote_ctrl.rc.ch[2] >> 8);
	  CAN_FD_Tx_Buffer[4] = (uint8_t)(remote_ctrl.rc.ch[2]);
	  CAN_FD_Tx_Buffer[5] = (uint8_t)(remote_ctrl.rc.ch[0] >> 8);
	  CAN_FD_Tx_Buffer[6] = (uint8_t)(remote_ctrl.rc.ch[0]);
	  CAN_FD_Tx_Buffer[7] = (uint8_t)(remote_ctrl.rc.ch[4] >> 8);
	  CAN_FD_Tx_Buffer[8] = (uint8_t)(remote_ctrl.rc.ch[4]);

	  CAN_FD_Tx_Buffer[9] = *Yaw_Ange;
	  CAN_FD_Tx_Buffer[10] = *(Yaw_Ange + 1);
	  CAN_FD_Tx_Buffer[11] = *(Yaw_Ange + 2);
	  CAN_FD_Tx_Buffer[12] = *(Yaw_Ange + 3);

	  CAN_FD_Tx_Buffer[13] = *Yaw_Gyro;
	  CAN_FD_Tx_Buffer[14] = *(Yaw_Gyro + 1);
	  CAN_FD_Tx_Buffer[15] = *(Yaw_Gyro + 2);
	  CAN_FD_Tx_Buffer[16] = *(Yaw_Gyro + 3);

	  // µ¼º½
	  CAN_FD_Tx_Buffer[17] = MiniPc_Auto.Vx_test[0];
	  CAN_FD_Tx_Buffer[18] = MiniPc_Auto.Vx_test[1];
	  CAN_FD_Tx_Buffer[19] = MiniPc_Auto.Vx_test[2];
	  CAN_FD_Tx_Buffer[20] = MiniPc_Auto.Vx_test[3];

	  CAN_FD_Tx_Buffer[21] = MiniPc_Auto.Vy_test[0];
	  CAN_FD_Tx_Buffer[22] = MiniPc_Auto.Vy_test[1];
	  CAN_FD_Tx_Buffer[23] = MiniPc_Auto.Vy_test[2];
	  CAN_FD_Tx_Buffer[24] = MiniPc_Auto.Vy_test[3];

	  // ×ÔÃé
	  CAN_FD_Tx_Buffer[25] = MiniPc_Rx_Vision.Yaw_test[0];
	  CAN_FD_Tx_Buffer[26] = MiniPc_Rx_Vision.Yaw_test[1];
	  CAN_FD_Tx_Buffer[27] = MiniPc_Rx_Vision.Yaw_test[2];
	  CAN_FD_Tx_Buffer[28] = MiniPc_Rx_Vision.Yaw_test[3];

	  CAN_FD_Tx_Buffer[29] = MiniPc_Rx_Vision.Vision_Grap;
	  CAN_FD_Tx_Buffer[30] = MiniPc_Rx_Vision.Fire_Switch;
	  CAN_FD_Tx_Buffer[35] = Control_Info.Symbol.Auto_to_Vision;
	  CAN_FD_Tx_Buffer[36] = MiniPc_Rx_Vision.Fire_Switch_Spin;

	  CAN_FD_Tx_Buffer[31] = *Shoot_HeatCtrl_Qres;
	  CAN_FD_Tx_Buffer[32] = *(Shoot_HeatCtrl_Qres + 1);
	  CAN_FD_Tx_Buffer[33] = *(Shoot_HeatCtrl_Qres + 2);
	  CAN_FD_Tx_Buffer[34] = *(Shoot_HeatCtrl_Qres + 3);

	  HAL_FDCAN_AddMessageToTxFifoQ(FDCAN3_TxFrame.hcan, &FDCAN3_TxFrame.Header, CAN_FD_Tx_Buffer);

	  FDCAN2_TxFrame.Header.Identifier = 0X01;
	  DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame, &DM_Pitch_Motor, 0, 0, 0, 0, Control_Info.Gimbal.Output.Pitch); // PitchÖá
	  if ((Control_Info.Control_Mode != Control_Info.Gimbal_Last_Mode) || (Control_Info.Control_Mode == Off))
		  DM_Motor_Command(&FDCAN2_TxFrame, &DM_Pitch_Motor, Motor_Disable);
	else   if  (Control_Info.Control_Mode != Off)               //((Control_Info.Control_Mode != Control_Info.Gimbal_Last_Mode) &&
		  DM_Motor_Command(&FDCAN2_TxFrame, &DM_Pitch_Motor, Motor_Enable);

	  FDCAN1_TxFrame.Header.Identifier = 0X200;
	  FDCAN1_TxFrame.Data[0] = (uint8_t)(Control_Info.SendValue[1] >> 8);
	  FDCAN1_TxFrame.Data[1] = (uint8_t)(Control_Info.SendValue[1]);
	  FDCAN1_TxFrame.Data[2] = (uint8_t)(Control_Info.SendValue[2] >> 8);
	  FDCAN1_TxFrame.Data[3] = (uint8_t)(Control_Info.SendValue[2]);
	  HAL_FDCAN_AddMessageToTxFifoQ(FDCAN1_TxFrame.hcan, &FDCAN1_TxFrame.Header, FDCAN1_TxFrame.Data);
	  osDelay(1);
  }
 
}

