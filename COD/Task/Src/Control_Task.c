/**
  ******************************************************************************
  * @file           : Control_Task.c
  * @brief          : Control task
  * @author         : GrassFan Wang
  * @date           : 2025/07/31
  * @version        : v2.2
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "Control_Task.h"
#include "bsp_uart.h"
#include "Remote_Control.h"
#include "PID.h"
#include "Motor.h"
#include "Minipc.h"
#include "INS_Task.h"
#include "Shoot.h"
#include "config.h"
#include "arm_math.h"

#define Gimbal_Weight    1.018   //有枪管，无图传质量，单位：Kg
#define Gimbal_Moment_Arm     0.05762 //力臂长度，单位：m 
#define Gimbal_Offset_Angle   0.4353367387   //质心相对水平面偏移角度，24.37单位：度，弧度为：0.6883578
																
static void Control_Init(Control_Info_Typedef *Control_Info);
static void Gimbal_Measure_Update(Control_Info_Typedef *Control_Info);
static void Gimbal_Target_Update(Control_Info_Typedef *Control_Info);
static void Gimbal_Info_Update(Control_Info_Typedef *Control_Info);
static void Control_Mode(Control_Info_Typedef *Control_Info);

float aba;

Gimbal_Info_Typedef Gimbal_Info;
Control_Info_Typedef Control_Info;
PID_Info_TypeDef PID_Pitch_KeyBoard[2];
PID_Info_TypeDef PID_Pitch_Vision[2];
PID_Info_TypeDef PID_Auto[2];																
															
//                                  KP   KI   KD  Alpha Deadband  I_MAX   Output_MAX
float PID_Pitch_KeyBoard_Angle_Param[7]  ={-0.31f,0.f,0.f,0.0f,0.0f,0.f,4.f};
float PID_Pitch_KeyBoard_Velocity_Param[7] = {0.30f,0.01f,0.0f,0.0f,0.0f,1.f,8.0f};

float PID_Pitch_Vision_Angle_Param[7]  ={-0.51f,0.f,0.f,0.0f,0.0f,1.f,8.f};  //0.325
float PID_Pitch_Vision_Velocity_Param[7] ={0.305f,0.001f,0.0f,0.1f,0.0f,1.f,8.0f};  //0.69  0.82


float PID_Auto_Angle_Param[7]  ={-0.22f,0.f,0.f,0.0f,0.0f,0.f,2.f};
float PID_Auto_Velocity_Param[7] ={0.22f,0.f,0.1f,0.0f,0.0f,1.f,8.0f};

PID_Info_TypeDef Chassis_PID;

  TickType_t Control_Task_SysTick = 0;
  TickType_t Control_Task_SysTick_Last = 0;
  TickType_t Auto_Time = 0;

void Control_Task(void const * argument)
{
  /* USER CODE BEGIN Control_Task */
	Control_Init(&Control_Info);
 /* Infinite loop */
	for(;;)
  {
		Control_Task_SysTick = osKernelSysTick();
	  Gimbal_Measure_Update(&Control_Info);
		Gimbal_Target_Update(&Control_Info);
    Gimbal_Info_Update(&Control_Info);
    Control_Mode(&Control_Info);
	 
 // USART_Vofa_Justfloat_Transmit(-MiniPc_Rx_Vision.Pitch , Control_Info.Gimbal.Measure.Pitch_Angle,PID_Pitch_Vision[0].Err[0]);	

		osDelay(1);
  }
}
  /* USER CODE END Control_Task */

static void Control_Init(Control_Info_Typedef *Control_Info){
	PID_Init(&PID_Pitch_KeyBoard[0],PID_POSITION,PID_Pitch_KeyBoard_Angle_Param);
	PID_Init(&PID_Pitch_KeyBoard[1],PID_POSITION,PID_Pitch_KeyBoard_Velocity_Param);
	PID_Init (&PID_Pitch_Vision[0],PID_POSITION,PID_Pitch_Vision_Angle_Param);
	PID_Init (&PID_Pitch_Vision[1],PID_POSITION,PID_Pitch_Vision_Velocity_Param);
	PID_Init (&PID_Auto[0],PID_POSITION,PID_Auto_Angle_Param);
	PID_Init (&PID_Auto[1],PID_POSITION,PID_Auto_Velocity_Param);
	Control_Info->Gimbal.Target.Auto_Pitch_Angle_err=0.1f;
  Control_Info->Symbol.Power_Up_Retain  = 0;
	Gimbal_Info.Target .Pitch_PID_K =1.2f;
	Control_Info->Symbol.Auto_to_Vision = 0;
}

static void Control_Mode(Control_Info_Typedef *Control_Info){
	Control_Info->Gimbal_Last_Mode = Control_Info->Control_Mode;
	// 模式选择
if (remote_ctrl.rc_lost == 1) {
    Control_Info->Control_Mode = Auto;  
} 

//if ((remote_ctrl.rc_lost == 1)&&()) {
//    Control_Info->Control_Mode = Off;
//}

else {
    switch (remote_ctrl.rc.s[0]){
        case 1: // 第一位为1的情况
            switch (remote_ctrl.rc.s[1]) {
             case 1: Control_Info->Control_Mode = Auto; break;   // 导航加自瞄
             case 2: Control_Info->Control_Mode = Chassis_Auto; break;   //底盘导航 (包括P轴)   
				     case 3: Control_Info->Control_Mode = Vision; break;     //自瞄
                default: 
				Control_Info->Control_Mode = Off; break; 
            }
            break;
            
        case 2: // 第一位为2的情况
            switch (remote_ctrl.rc.s[1]) {
                case 1: Control_Info->Control_Mode = Shoot; break;
                case 2: Control_Info->Control_Mode = Off; break;  //底盘和云台都失能
      			    case 3: Control_Info->Control_Mode = Spin_Vision; break; //暂定
                default: 
				Control_Info->Control_Mode = Off; break; 
            }
            break;
            
        case 3: // 第一位为3的情况
            switch (remote_ctrl.rc.s[1]) {
                case 1: Control_Info->Control_Mode = Gimbal_Normal; break;  //底盘小陀螺   云台正常
                case 2: Control_Info->Control_Mode = Gimbal_Normal; break;  //底盘失能     云台正常
				        case 3: Control_Info->Control_Mode = Gimbal_Normal; break;  //底盘跟随     云台正常
                default: 
				Control_Info->Control_Mode = Off; break; 
            }
            break; 
        default:
            Control_Info->Control_Mode = Off; break; 
    }
}
}

static void Gimbal_Measure_Update(Control_Info_Typedef *Control_Info){
   Control_Info->Gimbal.Measure.Pitch_Angle = -INS_Info.Roll_Angle;
   Control_Info->Gimbal.Measure.Pitch_Gyro  = DM_Pitch_Motor.Data.Velocity; //  INS_Info.Gyro[1];
   Control_Info->Gimbal.Measure.Left_shoot = DJI_Shoot_Motor[0].Data.Velocity;
	 Control_Info->Gimbal.Measure.Right_Shoot = DJI_Shoot_Motor[1].Data.Velocity;
}
float text;
static void Gimbal_Target_Update(Control_Info_Typedef *Control_Info){
	//视觉
	if( Control_Info->Control_Mode==Vision){
		if(MiniPc_Rx_Vision.Vision_Grap==1)
		 Control_Info->Gimbal.Target.Vision_Pitch_Angle = MiniPc_Rx_Vision.Pitch;

		else if (MiniPc_Rx_Vision.Vision_Grap==0)
		 Control_Info->Gimbal.Target.Vision_Pitch_Angle+=remote_ctrl.rc.ch[1] * 0.0002f;	
	}
	
		//视觉
	if( Control_Info->Control_Mode==Spin_Vision){
		if(MiniPc_Rx_Vision.Vision_Grap==1)
		 Control_Info->Gimbal.Target.Vision_Pitch_Angle = MiniPc_Rx_Vision.Pitch;
					
		else if (MiniPc_Rx_Vision.Vision_Grap==0)
		 Control_Info->Gimbal.Target.Vision_Pitch_Angle+=remote_ctrl.rc.ch[1] * 0.0002f;
	}
	//导航
	if(( Control_Info->Control_Mode == Auto)||(Control_Info->Control_Mode == Chassis_Auto)){
		if((Control_Info->Symbol.Auto_to_Vision==1)&&(MiniPc_Rx_Vision.Vision_Grap==1)){
		 Control_Info->Gimbal.Target.Vision_Pitch_Angle = MiniPc_Rx_Vision.Pitch;
		
	}
			else{
		if((Control_Info->Gimbal.Target.Auto_Pitch_Angle) <= -23.0f)
    Control_Info->Gimbal.Target.Auto_Pitch_Angle_err+=0.05;
		else if((Control_Info->Gimbal.Target.Auto_Pitch_Angle) >= 18.0f)
    Control_Info->Gimbal.Target.Auto_Pitch_Angle_err-=0.05;
	Control_Info->Gimbal.Target.Auto_Pitch_Angle+= Control_Info->Gimbal.Target.Auto_Pitch_Angle_err;
	}
		
	}
	if(( Control_Info->Control_Mode == Gimbal_Normal)||(Control_Info->Control_Mode == Shoot)){
    if (Control_Info->Symbol.Power_Up_Retain == 1){ //上电保持原来的位置
  Control_Info->Gimbal.Target.Pitch_Angle+= remote_ctrl.rc.ch[1] * 0.0002f;
    }
		else{
		Control_Info->Gimbal.Target.Pitch_Angle = INS_Info.Roll_Angle;
    Control_Info->Symbol.Power_Up_Retain = 1; 
    }
	}
	//pitch轴限幅，防止超过机械限位
  VAL_LIMIT(Control_Info->Gimbal.Target.Pitch_Angle,-23,18); 
	
//	if(MiniPc_Rx_Vision.Distance<=0.8){
//		if(-MiniPc_Rx_Vision.Pitch <-20)  MiniPc_Rx_Vision.Pitch =-20;
//		else if(-MiniPc_Rx_Vision.Pitch >=20)  MiniPc_Rx_Vision.Pitch =20;
//			
//	}
//	
	
}
static void Gimbal_Info_Update(Control_Info_Typedef *Control_Info){
	   	PID_Calculate(&PID_Pitch_KeyBoard[0] , Control_Info->Gimbal.Target.Pitch_Angle, Control_Info->Gimbal.Measure.Pitch_Angle);
       PID_Calculate(&PID_Pitch_KeyBoard[1],PID_Pitch_KeyBoard[0].Output,Control_Info->Gimbal.Measure.Pitch_Gyro);
	
	 Gimbal_Info.Target.Last_Pitch_PID_Output = PID_Pitch_Vision[0].Output;
   Control_Info->Gimbal.Output.Compensation =0; //-0.29099; //(Gimbal_Weight * GravityAccel * Gimbal_Moment_Arm * cos(Gimbal_Offset_Angle + INS_Info.Roll_Angle*DegreesToRadians));// t=mgrcos(β+γ)；
	
       PID_Calculate(&PID_Pitch_Vision[0],  Control_Info->Gimbal.Target.Vision_Pitch_Angle , Control_Info->Gimbal.Measure.Pitch_Angle);
       PID_Calculate(&PID_Pitch_Vision[1], PID_Pitch_Vision[0].Output-aba, Control_Info->Gimbal.Measure.Pitch_Gyro);
	
	   aba =( PID_Pitch_Vision[0].Output - Gimbal_Info.Target.Last_Pitch_PID_Output) * Gimbal_Info.Target .Pitch_PID_K;
	
		   PID_Calculate(&PID_Auto[0] ,Control_Info->Gimbal.Target.Auto_Pitch_Angle,Control_Info->Gimbal.Measure.Pitch_Angle);
       PID_Calculate(&PID_Auto[1],PID_Auto[0].Output ,Control_Info->Gimbal.Measure.Pitch_Gyro);
	
	//输出
if(Control_Info->Control_Mode==Off){
Control_Info->Gimbal.Output.Pitch = 0;	
  Control_Info->SendValue[1] = 0;
  Control_Info->SendValue[2] = 0;
	Control_Info->Symbol.Power_Up_Retain = 0;
  }

  if(Control_Info->Control_Mode==Shoot){
Control_Info->Gimbal.Output.Pitch =( PID_Pitch_KeyBoard[1].Output)+ Control_Info->Gimbal.Output.Compensation;
    }

   if(Control_Info->Control_Mode==Auto){		 
		 if(Control_Info->Symbol.Auto_to_Vision == 1)	Control_Info->Gimbal.Output.Pitch= PID_Pitch_Vision[1].Output+ Control_Info->Gimbal.Output.Compensation;
	else Control_Info->Gimbal.Output.Pitch =PID_Auto[1].Output+ Control_Info->Gimbal.Output.Compensation;
      }
	 
	    if(Control_Info->Control_Mode==Chassis_Auto){
	 if(Control_Info->Symbol.Auto_to_Vision == 1)	Control_Info->Gimbal.Output.Pitch= PID_Pitch_Vision[1].Output+ Control_Info->Gimbal.Output.Compensation;
	else Control_Info->Gimbal.Output.Pitch =PID_Auto[1].Output+ Control_Info->Gimbal.Output.Compensation;
      }
	 
	    if((Control_Info->Control_Mode == Vision) ||(Control_Info->Control_Mode == Spin_Vision)){
				Control_Info->Gimbal.Output.Pitch= PID_Pitch_Vision[1].Output+ Control_Info->Gimbal.Output.Compensation;
      }			
	if(Control_Info->Control_Mode == Gimbal_Normal){
Control_Info->Gimbal.Output.Pitch = PID_Pitch_KeyBoard[1].Output + Control_Info->Gimbal.Output.Compensation;		
	}	
	
}






	