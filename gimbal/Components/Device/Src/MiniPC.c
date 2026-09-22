/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : MiniPC.c
  * @brief          : MiniPC interfaces functions 
  * @author         : GarssFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "MiniPC.h"
#include "usbd_cdc_if.h"
#include "Control_Task.h"
#include "stdio.h"
#include "INS_Task.h"
#include "PID.h"


 MiniPc_Receive_Auto_Typedef MiniPc_Auto;
 MiniPc_Receive_Vision_Typedef MiniPc_Rx_Vision;
 
 
 
float xia;
float shang;

 
float Bit8Tofloat32(uint8_t *Change_Info){
  
	union{
	  float   float32;
		uint8_t Bit8[4];
	
	}Value;

  Value.Bit8[0] = Change_Info[0];
  Value.Bit8[1] = Change_Info[1];
  Value.Bit8[2] = Change_Info[2];
  Value.Bit8[3] = Change_Info[3];
	return Value.float32;
}
void MiniPC_Transmit_Info(uint8_t *Buff){


}

//usbd_cdc_if.c -> CDC_Receive_HS
void MiniPC_Recvive_Info(uint8_t* Buff, const uint32_t *Len){
//导航数据
if (Buff[0] == 0xA5)
{
	MiniPc_Auto.Vx_test[0] = Buff[1];
	MiniPc_Auto.Vx_test[1] = Buff[2];
	MiniPc_Auto.Vx_test[2] = Buff[3];
	MiniPc_Auto.Vx_test[3] = Buff[4];
	MiniPc_Auto.Vx = Bit8Tofloat32(MiniPc_Auto.Vx_test);
	MiniPc_Auto.Vy_test[0] = Buff[5];
	MiniPc_Auto.Vy_test[1] = Buff[6];
	MiniPc_Auto.Vy_test[2] = Buff[7];
	MiniPc_Auto.Vy_test[3] = Buff[8];
	MiniPc_Auto.Vy = Bit8Tofloat32(MiniPc_Auto.Vy_test);
	MiniPc_Auto.Vz_test[0] = Buff[9];
	MiniPc_Auto.Vz_test[1] = Buff[10];
	MiniPc_Auto.Vz_test[2] = Buff[11];
	MiniPc_Auto.Vz_test[3] = Buff[12];
	MiniPc_Auto.Vz = Bit8Tofloat32(MiniPc_Auto.Vz_test);
	MiniPc_Auto.Spin_test[0] = Buff[14];
	MiniPc_Auto.Spin_test[1] = Buff[15];
	MiniPc_Auto.Spin_test[2] = Buff[16];
	MiniPc_Auto.Spin_test[3] = Buff[17];
	MiniPc_Auto.Spon_Speed = Bit8Tofloat32(MiniPc_Auto.Spin_test);

	if (MiniPc_Auto.Spon_Speed > 10)
		MiniPc_Auto.Spin_Switch = 1;
	
	else if(MiniPc_Auto.Spon_Speed==0)
		MiniPc_Auto.Spin_Switch=0;
}
	//决策数据  //帧头根据实际情况更改
//if (Buff[0] == 0xAA){
//	MiniPc_Rx_Vision.Pitch_test[0] = Buff[1];
//	MiniPc_Rx_Vision.Pitch_test[1] = Buff[2];
//	MiniPc_Rx_Vision.Pitch_test[2] = Buff[3];
//	MiniPc_Rx_Vision.Pitch_test[3] = Buff[4];
//	MiniPc_Rx_Vision.Pitch = Bit8Tofloat32(MiniPc_Rx_Vision.Pitch_test);
//	MiniPc_Rx_Vision.Yaw_test[0] = Buff[5];
//	MiniPc_Rx_Vision.Yaw_test[1] = Buff[6];
//	MiniPc_Rx_Vision.Yaw_test[2] = Buff[7];
//	MiniPc_Rx_Vision.Yaw_test[3] = Buff[8];
//	MiniPc_Rx_Vision.Yaw = Bit8Tofloat32(MiniPc_Rx_Vision.Yaw_test);
//	MiniPc_Rx_Vision.Distance_test[0] = Buff[9];
//	MiniPc_Rx_Vision.Distance_test[1] = Buff[10];
//	MiniPc_Rx_Vision.Distance_test[2] = Buff[11];
//	MiniPc_Rx_Vision.Distance_test[3] = Buff[12];
//	MiniPc_Rx_Vision.Distance = Bit8Tofloat32(MiniPc_Rx_Vision.Distance_test);
//	//MiniPc_Rx_Vision.Fire_Switch = Buff[13];
//	
//	
//}

	//自瞄数据
	if(Buff[0] == 0xff){	
//	MiniPc_Rx_Vision.Fire_Switch = Buff[1];                                
	MiniPc_Rx_Vision.Pitch_test[0] = Buff[2];
  MiniPc_Rx_Vision.Pitch_test[1] = Buff[3];
	MiniPc_Rx_Vision.Pitch_test[2] = Buff[4];
  MiniPc_Rx_Vision.Pitch_test[3] = Buff[5];
  MiniPc_Rx_Vision.Pitch = Bit8Tofloat32(MiniPc_Rx_Vision.Pitch_test);
		
		
  MiniPc_Rx_Vision.Yaw_test[0] = Buff[6];
	MiniPc_Rx_Vision.Yaw_test[1] = Buff[7];
  MiniPc_Rx_Vision.Yaw_test[2] = Buff[8];
  MiniPc_Rx_Vision.Yaw_test[3] = Buff[9];
  MiniPc_Rx_Vision.Yaw = Bit8Tofloat32(MiniPc_Rx_Vision.Yaw_test);	
		
	MiniPc_Rx_Vision.Distance_test[0]= Buff[10];
	MiniPc_Rx_Vision.Distance_test[1]= Buff[11];
	MiniPc_Rx_Vision.Distance_test[2]= Buff[12];
	MiniPc_Rx_Vision.Distance_test[3]= Buff[13];
	MiniPc_Rx_Vision.Distance =  Bit8Tofloat32(MiniPc_Rx_Vision.Distance_test);
	//控制延时前的Yaw Pitch
	MiniPc_Rx_Vision.Fire_Pitch_test[0] = Buff[14];
  MiniPc_Rx_Vision.Fire_Pitch_test[1] = Buff[15];
	MiniPc_Rx_Vision.Fire_Pitch_test[2] = Buff[16];
  MiniPc_Rx_Vision.Fire_Pitch_test[3] = Buff[17];
  MiniPc_Rx_Vision.Fire_Pitch = Bit8Tofloat32(MiniPc_Rx_Vision.Fire_Pitch_test);
	
  MiniPc_Rx_Vision.Fire_Yaw_test[0] = Buff[18];
	MiniPc_Rx_Vision.Fire_Yaw_test[1] = Buff[19];
  MiniPc_Rx_Vision.Fire_Yaw_test[2] = Buff[20];
  MiniPc_Rx_Vision.Fire_Yaw_test[3] = Buff[21];
  MiniPc_Rx_Vision.Fire_Yaw = Bit8Tofloat32(MiniPc_Rx_Vision.Fire_Yaw_test);			
		
	}
if(MiniPc_Rx_Vision.Distance!=-1){
	MiniPc_Rx_Vision.Vision_Grap=1;   //瞄准 1
	Control_Info.Symbol.Auto_to_Vision = 1;
		}
else if(MiniPc_Rx_Vision.Distance==-1){
	MiniPc_Rx_Vision.Vision_Grap=0;  //不瞄准
	Control_Info.Symbol.Auto_to_Vision = 0;
		}
else{
	MiniPc_Rx_Vision.Vision_Grap=0;  //不瞄准
	Control_Info.Symbol.Auto_to_Vision = 0;
	}

	if(MiniPc_Rx_Vision.Distance<=2.2) MiniPc_Rx_Vision.Fire_Switch_Spin =1;
	else MiniPc_Rx_Vision.Fire_Switch_Spin =0;
}

