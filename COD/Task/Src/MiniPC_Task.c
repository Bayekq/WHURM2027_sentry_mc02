#include "MiniPC_Task.h"
#include "cmsis_os.h"
#include "MiniPC.h"
#include "usbd_cdc_if.h"
#include "Control_Task.h"
#include "INS_Task.h"
#include "bsp_can.h"

uint8_t Buff[20]={0};
#define MINIPC_SENDLENGTH 26U
#define MINIPC_REFEREE_SYSTEM_SENDLENGTH 32U

float Hp_test;
uint8_t *Hp;
//float Hp_test_1;
//uint8_t *Hp_1;
//float Hp_test_3;
//uint8_t *Hp_3;
//float Hp_test_7;
//uint8_t *Hp_7;

uint8_t MiniPC_TxBuff[MINIPC_SENDLENGTH];
uint8_t MiniPC_Referee_System_TxBuff[MINIPC_REFEREE_SYSTEM_SENDLENGTH];

 MiniPc_Transmit_Vision_Typedef MiniPc_Tx_Vision;
MiniPc_Tx_Auto_Decision_Typedef MiniPc_Tx_Auto_Decision;

 G_Typedef G;

TickType_t MiniPC_Task_SysTick = 0;
/* USER CODE BEGIN Header_MiniPC_Task */
/**
* @brief Function implementing the Start_MiniPC_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MiniPC_Task */
void MiniPC_Task(void const * argument)
{
  /* USER CODE BEGIN MiniPC_Task */
  /* Infinite loop */
  for(;;)
  {
	MiniPC_Task_SysTick=osKernelSysTick();
	//自瞄
		float Vision_Pitch_Angle = -INS_Info.Roll_Angle;
		float Vision_Yaw_Angle = INS_Info.Yaw_Angle;
		float Vision_Roll_Angle = INS_Info.Pitch_Angle;
 		MiniPc_Tx_Vision.Yaw = (uint8_t*)&Vision_Yaw_Angle;
		MiniPc_Tx_Vision.Roll = (uint8_t*)&Vision_Roll_Angle;
		MiniPc_Tx_Vision.Pitch =(uint8_t*)&Vision_Pitch_Angle;
     //决策
	 
			Hp_test = (float)MiniPc_Tx_Auto_Decision.Robot_HP.Current_HP;
			Hp = (uint8_t*)&Hp_test;	
//			Hp_test_1 = (float)MiniPc_Tx_Auto_Decision.Robot_HP.Robot_1_HP;
//			Hp_1 = (uint8_t*)&Hp_test_1;	
//			Hp_test_3 =  (float)MiniPc_Tx_Auto_Decision.Robot_HP.Robot_3_HP;
//			Hp_3 = (uint8_t*)&Hp_test_3;
//							Hp_test_7 = (float)MiniPc_Tx_Auto_Decision.Robot_HP.Robot_7_HP;
//			Hp_7 = (uint8_t*)&Hp_test_7;		
          //自瞄
 			   MiniPC_TxBuff[0]=0xFF; 
				 MiniPC_TxBuff[1]=0;
				 MiniPC_TxBuff[2]=*MiniPc_Tx_Vision.Pitch;
				 MiniPC_TxBuff[3]=*(MiniPc_Tx_Vision.Pitch + 1);
				 MiniPC_TxBuff[4]=*(MiniPc_Tx_Vision.Pitch + 2);
				 MiniPC_TxBuff[5]=*(MiniPc_Tx_Vision.Pitch + 3);
				 MiniPC_TxBuff[6]=*MiniPc_Tx_Vision.Roll;
				 MiniPC_TxBuff[7]=*(MiniPc_Tx_Vision.Roll + 1);
				 MiniPC_TxBuff[8]=*(MiniPc_Tx_Vision.Roll + 2);
				 MiniPC_TxBuff[9]=*(MiniPc_Tx_Vision.Roll + 3);				 
				 MiniPC_TxBuff[10]=*MiniPc_Tx_Vision.Yaw;
				 MiniPC_TxBuff[11]=*(MiniPc_Tx_Vision.Yaw + 1);
				 MiniPC_TxBuff[12]=*(MiniPc_Tx_Vision.Yaw + 2);
				 MiniPC_TxBuff[13]=*(MiniPc_Tx_Vision.Yaw + 3);
				 //决策
			  MiniPC_TxBuff[14]= MiniPc_Tx_Auto_Decision.Rfid.Zone_Status;  //
				MiniPC_TxBuff[15]= MiniPc_Tx_Auto_Decision.Rfid.Area_Status;   //
				MiniPC_TxBuff[16]= MiniPc_Tx_Auto_Decision.Even_Data.Even_Data;
				MiniPC_TxBuff[17]= MiniPc_Tx_Auto_Decision.Even_Data.Mode_Defense;				
				MiniPC_TxBuff[18]= *Hp;
				MiniPC_TxBuff[19]= *(Hp + 1);
				MiniPC_TxBuff[20]= *(Hp + 2);
				MiniPC_TxBuff[21]= *(Hp + 3);
//				MiniPC_TxBuff[22]=* MiniPc_Tx_Vision.Yaw;
//				MiniPC_TxBuff[23]=*(MiniPc_Tx_Vision.Yaw + 1);
//				MiniPC_TxBuff[24]=*(MiniPc_Tx_Vision.Yaw + 2);
//				MiniPC_TxBuff[25]=*(MiniPc_Tx_Vision.Yaw + 3);			 
				MiniPC_TxBuff[24]= 0x00; 
        MiniPC_TxBuff[25]= 0x0d;  
  CDC_Transmit_HS(MiniPC_TxBuff,MINIPC_SENDLENGTH);

    osDelay(1);
  }
  /* USER CODE END MiniPC_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
