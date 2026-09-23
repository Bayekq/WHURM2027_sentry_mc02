/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_can.c
  * @brief          : bsp can functions 
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : Pay attention to enable the fdcan filter
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "fdcan.h"
#include "bsp_can.h"
#include "Motor.h"
#include "Remote_Control.h"
#include "Minipc.h"
#include "Shoot.h"
/**
 * @brief The structure that contains the Information of FDCAN1 and FDCAN2 Receive.
 */
FDCAN_RxFrame_TypeDef FDCAN_RxFIFO0Frame;
FDCAN_RxFrame_TypeDef FDCAN_RxFIFO1Frame;

uint8_t CAN_RX_Buffer[64];
static float bit8TOfloat32_commit(uint8_t change_info[4]);

/**
 * @brief The structure that contains the Information of FDCAN1 Transmit(CLASSIC_CAN).
 */
FDCAN_TxFrame_TypeDef FDCAN1_TxFrame = {
	.hcan = &hfdcan1,
  .Header.IdType = FDCAN_STANDARD_ID, 
  .Header.TxFrameType = FDCAN_DATA_FRAME,
  .Header.DataLength = 8,
	.Header.ErrorStateIndicator =  FDCAN_ESI_ACTIVE,
  .Header.BitRateSwitch = FDCAN_BRS_OFF,
  .Header.FDFormat =  FDCAN_CLASSIC_CAN,           
  .Header.TxEventFifoControl =  FDCAN_NO_TX_EVENTS,  
  .Header.MessageMarker = 0,
};


/**
 * @brief The structure that contains the Information of FDCAN2 Transmit(FDCAN).
 */
FDCAN_TxFrame_TypeDef FDCAN2_TxFrame = {
   .hcan = &hfdcan2,
  .Header.IdType = FDCAN_STANDARD_ID, 
  .Header.TxFrameType = FDCAN_DATA_FRAME,
  .Header.DataLength = 8,
	.Header.ErrorStateIndicator =  FDCAN_ESI_ACTIVE,
  .Header.BitRateSwitch = FDCAN_BRS_OFF,
  .Header.FDFormat =  FDCAN_CLASSIC_CAN,           
  .Header.TxEventFifoControl =  FDCAN_NO_TX_EVENTS,  
  .Header.MessageMarker = 0,
};

/**
 * @brief The structure that contains the Information of FDCAN3 Transmit(CLASSIC_CAN).
 */
FDCAN_TxFrame_TypeDef FDCAN3_TxFrame = {
  .hcan = &hfdcan3,
  .Header.IdType = FDCAN_STANDARD_ID, 
  .Header.TxFrameType = FDCAN_DATA_FRAME,
  .Header.DataLength = 15,
	.Header.ErrorStateIndicator =  FDCAN_ESI_ACTIVE,
  .Header.BitRateSwitch = FDCAN_BRS_ON,
  .Header.FDFormat =  FDCAN_FD_CAN,           
  .Header.TxEventFifoControl =  FDCAN_NO_TX_EVENTS,
	.Header.MessageMarker = 0,
};

/**
  * @brief  Configures the FDCAN Filter. 
            FDCAN1:CLASSIC_CAN  FDCAN2:FDCAN  FDCAN3:CLASSIC_CAN
  * @param  None
  * @retval None
  */
void BSP_FDCAN_Init(void){
 FDCAN_FilterTypeDef FDCAN1_FilterConfig;
	
	FDCAN1_FilterConfig.IdType = FDCAN_STANDARD_ID; // 过滤标准ID，经典CAN只有标准ID
  FDCAN1_FilterConfig.FilterIndex = 0;           //过滤器编号，用几路CAN就依次类推0、1、2....
  FDCAN1_FilterConfig.FilterType = FDCAN_FILTER_MASK; //过滤器Mask模式 关乎到下面ID1、ID2的配置
  FDCAN1_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;//选择哪个FIFO区接收，根据你CubeMX的配置来，FIFO1就改成FDCAN_FILTER_TO_RXFIFO1
  FDCAN1_FilterConfig.FilterID1 = 0x00000000; // 这个都行，只要ID2配置0x00000000就不会过滤调任何ID
  FDCAN1_FilterConfig.FilterID2 = 0x00000000; //理由如上
  
  HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN1_FilterConfig); //将上述配置到CAN1
		
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE); //开启CAN1的全局过滤，就是开启过滤器
 
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);//打开FIFO0区的新数据接收中断，
  
  HAL_FDCAN_Start(&hfdcan1);//使能CAN1
 		
	FDCAN_FilterTypeDef FDCAN2_FilterConfig;
	
	FDCAN2_FilterConfig.IdType = FDCAN_STANDARD_ID;
  FDCAN2_FilterConfig.FilterIndex = 0;
  FDCAN2_FilterConfig.FilterType = FDCAN_FILTER_MASK;
  FDCAN2_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  FDCAN2_FilterConfig.FilterID1 = 0x00000000; 
  FDCAN2_FilterConfig.FilterID2 = 0x00000000; 
  
	HAL_FDCAN_ConfigFilter(&hfdcan2, &FDCAN2_FilterConfig);

  HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

  HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);


	
		HAL_FDCAN_ConfigFilter(&hfdcan2, &FDCAN2_FilterConfig);

  HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

  HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

  HAL_FDCAN_Start(&hfdcan2);
	

	FDCAN_FilterTypeDef FDCAN3_FilterConfig;

	FDCAN3_FilterConfig.IdType = FDCAN_STANDARD_ID;
  FDCAN3_FilterConfig.FilterIndex = 0;
  FDCAN3_FilterConfig.FilterType = FDCAN_FILTER_MASK;
  FDCAN3_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
  FDCAN3_FilterConfig.FilterID1 = 0x00000000; 
  FDCAN3_FilterConfig.FilterID2 = 0x00000000; 
  
	HAL_FDCAN_ConfigFilter(&hfdcan3, &FDCAN3_FilterConfig);

  HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

  HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
	
	HAL_FDCAN_EnableTxDelayCompensation(&hfdcan3);

  HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan3,14,14);

  HAL_FDCAN_Start(&hfdcan3);	
}
/**
  * @brief  Function to transmit the FDCAN message.
  * @param  *FDCAN_TxFrame :the structure that contains the Information of FDCAN
  * @retval None
  */
void USER_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame){
    FDCAN1_TxFrame.Header.Identifier=0x1ff;
	FDCAN2_TxFrame.Header.Identifier=0x01;
    HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame->hcan,&FDCAN_TxFrame->Header,FDCAN_TxFrame->Data);
 
}

/**
  * @brief  Function to converting the FDCAN1 received message to Fifo0.
	* @param  Identifier: Received the identifier.
	* @param  Data: Array that contains the received massage.
  * @retval None
  */
static void FDCAN1_RxFifo0RxHandler(uint32_t *Identifier,uint8_t Data[8]){
	DJI_Motor_Info_Update(Identifier,Data,&DJI_Shoot_Motor[0]);
  DJI_Motor_Info_Update(Identifier,Data,&DJI_Shoot_Motor[1]);
}
/**
  * @brief  Function to converting the FDCAN3 received message to Fifo0.
	* @param  Identifier: Received the identifier.
	* @param  Data: Array that contains the received massage.
  * @retval None
  */
static void FDCAN3_RxFifo1RxHandler(uint32_t *Identifier,uint8_t Data[8]){
 if(*Identifier==0X400){	
		MiniPc_Tx_Auto_Decision.Rfid.Zone_Status = CAN_RX_Buffer[0]; // 补给
		MiniPc_Tx_Auto_Decision.Rfid.Area_Status = CAN_RX_Buffer[1]; //得分
	  MiniPc_Tx_Auto_Decision.Even_Data.Even_Data = CAN_RX_Buffer[2];
	  MiniPc_Tx_Auto_Decision.Even_Data.Mode_Defense = CAN_RX_Buffer[3];
	 
	 	MiniPc_Tx_Auto_Decision.Robot_HP.Current_HP = ((uint16_t) (CAN_RX_Buffer[4]<<8) |  (CAN_RX_Buffer[5]));  //剩余血量
	  MiniPc_Tx_Auto_Decision.Robot_HP.Robot_1_HP = ((uint16_t) (CAN_RX_Buffer[6]<<8) |  (CAN_RX_Buffer[7]));  //对面剩余血量
	  MiniPc_Tx_Auto_Decision.Robot_HP.Robot_3_HP = ((uint16_t) (CAN_RX_Buffer[8]<<8) |  (CAN_RX_Buffer[9]));  //对面剩余血量
	  MiniPc_Tx_Auto_Decision.Robot_HP.Robot_7_HP = ((uint16_t) (CAN_RX_Buffer[10]<<8)|  (CAN_RX_Buffer[11]));  //对面剩余血量
		 
		Shoot_Info.Shoot.initial_speed_test[0] = CAN_RX_Buffer[12];
		Shoot_Info.Shoot.initial_speed_test[1] = CAN_RX_Buffer[13];
		Shoot_Info.Shoot.initial_speed_test[2] = CAN_RX_Buffer[14];
		Shoot_Info.Shoot.initial_speed_test[3] = CAN_RX_Buffer[15];
		Shoot_Info.Shoot.initial_speed= bit8TOfloat32_commit(Shoot_Info.Shoot.initial_speed_test);
	
	}
}


/**
  * @brief  Function to converting the FDCAN2 received message to Fifo1.
	* @param  Identifier: Received the identifier.
	* @param  Data: Array that contains the received massage.
  * @retval None
  */
static void FDCAN2_RxFifo0RxHandler(uint32_t *Identifier,uint8_t Data[8]){
	DM_Motor_Info_Update(Identifier,Data,&DM_Pitch_Motor);
}

/**
  * @brief  Rx FIFO 0 callback.
  * @param  hfdcan pointer to an FDCAN_HandleTypeDef structure that contains
  *         the configuration information for the specified FDCAN.
  * @param  RxFifo0ITs indicates which Rx FIFO 0 interrupts are signaled.
  *         This parameter can be any combination of @arg FDCAN_Rx_Fifo0_Interrupts.
  * @retval None
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{ 
  
	HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &FDCAN_RxFIFO0Frame.Header, FDCAN_RxFIFO0Frame.Data);
	
  if(hfdcan == &hfdcan1){	
	
  FDCAN1_RxFifo0RxHandler(&FDCAN_RxFIFO0Frame.Header.Identifier,FDCAN_RxFIFO0Frame.Data);
	 
	}
	
	if(hfdcan == &hfdcan2){
	 
		FDCAN2_RxFifo0RxHandler(&FDCAN_RxFIFO0Frame.Header.Identifier,FDCAN_RxFIFO0Frame.Data);
	
	}

}
	
	
/**
  * @brief  Rx FIFO 1 callback.
  * @param  hfdcan pointer to an FDCAN_HandleTypeDef structure that contains
  *         the configuration information for the specified FDCAN.
  * @param  RxFifo1ITs indicates which Rx FIFO 1 interrupts are signaled.
  *         This parameter can be any combination of @arg FDCAN_Rx_Fifo1_Interrupts.
  * @retval None
  */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{ 
  
	HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &FDCAN_RxFIFO1Frame.Header, CAN_RX_Buffer);
	FDCAN3_RxFifo1RxHandler(&FDCAN_RxFIFO1Frame.Header.Identifier,CAN_RX_Buffer);
	
	
}
	

void USER_CAN_TxMessage(FDCAN_TxFrame_TypeDef *TxFrame,DJI_Motor_Info_Typedef*DJI_MOTOR,uint8_t data[8])
{

 TxFrame->Header.Identifier = DJI_MOTOR->FDCANFrame.TxIdentifier;
	
	HAL_FDCAN_AddMessageToTxFifoQ(TxFrame->hcan,&TxFrame->Header,data);
}	


//将这个 8 位整型数据转换成 32 位浮点数
static float bit8TOfloat32_commit(uint8_t change_info[4])
{
	union
	{
    float float32;
		uint8_t  byte[4];
	}u32val;

  u32val.byte[0] = change_info[0];
  u32val.byte[1] = change_info[1];
  u32val.byte[2] = change_info[2];
  u32val.byte[3] = change_info[3];

	return u32val.float32;
}