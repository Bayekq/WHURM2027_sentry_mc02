
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


static void Control_Init(Control_Info_Typedef *Control_Info);
static void Gimbal_Measure_Update(Control_Info_Typedef *Control_Info);
static void Gimbal_Target_Update(Control_Info_Typedef *Control_Info);
static void Gimbal_Info_Update(Control_Info_Typedef *Control_Info);
static void Control_Fire(Control_Info_Typedef *Control_Info);
static void Heat_Detection(void);
static float SpeedAdapt(float real_S, float min_S, float max_S, float up_num, float down_num);
static bool IsOnTarget(const double cur_yaw,const double cur_pitch,const double target_yaw,const double target_pitch,const double distance);
PID_Info_TypeDef PID_Shoot[2];

Shoot_Info_Typedef Shoot_Info;

bool  pitch_ok;
bool  yaw_ok;
double shooting_range_yaw;
double shooting_range_pitch;

//                                  KP   KI   KD  Alpha Deadband  I_MAX   Output_MAX
float PID_Shoot_Right_Param[7]= {16.0f,0.0f,1.f,0.1f,0.0f,100.0f,10000.0f};
float PID_Shoot_Left_Param[7]= {16.0f,0.0f,1.f,0.1,0.0f,100.0f,10000.0f};

/* USER CODE BEGIN Header_Shoot_Task */
/**
* @brief Function implementing the Start_Shoot_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Shoot_Task */
     TickType_t systick = 0;

 void Shoot_Task(void const * argument)
{
  /* USER CODE BEGIN Shoot_Task */
		Control_Init(&Control_Info);
  /* Infinite loop */
  for(;;)
  {		
		systick = osKernelSysTick();	
	Gimbal_Measure_Update(&Control_Info);
	Gimbal_Target_Update(&Control_Info);
	Heat_Detection();	
    Gimbal_Info_Update(&Control_Info);
 Control_Fire(&Control_Info);
		
//	if (Shoot_Info.Shoot.initial_speed!= Control_Info.Gimbal.Target.Shoot_Speed_last)
//    USART_Vofa_Justfloat_Transmit(Shoot_Info.Shoot.initial_speed ,Control_Info.Gimbal.Target.Shoot_Speed,DJI_Shoot_Motor[0].Data.Velocity);	
    osDelay(1);
  }
  /* USER CODE END Shoot_Task */
}



static void Control_Init(Control_Info_Typedef *Control_Info){

 PID_Init(&PID_Shoot[0],PID_POSITION,PID_Shoot_Right_Param);
  PID_Init(&PID_Shoot[1],PID_POSITION,PID_Shoot_Left_Param);		
  Shoot_Info.Shoot_HeatCtrl.Qcd =30.0f;	
  Shoot_Info.Shoot_HeatCtrl.Qmax =260.0f;
}

static void Control_Fire(Control_Info_Typedef *Control_Info){
	
	if(MiniPc_Rx_Vision.Vision_Grap==1)
MiniPc_Rx_Vision.Fire_Switch = IsOnTarget(INS_Info.Yaw_Angle*DegreesToRadians,-INS_Info.Roll_Angle*DegreesToRadians,MiniPc_Rx_Vision.Fire_Yaw*DegreesToRadians, MiniPc_Rx_Vision.Fire_Pitch*DegreesToRadians,MiniPc_Rx_Vision.Distance);
  else MiniPc_Rx_Vision.Fire_Switch=0;
		
}

static void Gimbal_Measure_Update(Control_Info_Typedef *Control_Info){

   Control_Info->Gimbal.Measure.Left_shoot = DJI_Shoot_Motor[0].Data.Velocity;
	 Control_Info->Gimbal.Measure.Right_Shoot = DJI_Shoot_Motor[1].Data.Velocity;
}

static void Gimbal_Target_Update(Control_Info_Typedef *Control_Info){
	
  //保持弹频1
if(systick%121==0){
if (Shoot_Info.Shoot.initial_speed!= Control_Info->Gimbal.Target.Shoot_Speed_last){   //Shoot_Info.Shoot.initial_speed =21.8
Shoot_Info.Shoot.Fire_Trigger += SpeedAdapt(Shoot_Info.Shoot.initial_speed, 22.5,23.0,3,6);
Control_Info->Gimbal.Target.Shoot_Speed += Shoot_Info.Shoot.Fire_Trigger;
}
else{
Shoot_Info.Shoot.Fire_Trigger  = 0;
}
 Control_Info->Gimbal.Target.Shoot_Speed_last= Shoot_Info.Shoot.initial_speed;
}
  //限制摩擦轮转速在5900到6500之间
	VAL_LIMIT((Control_Info->Gimbal.Target.Shoot_Speed),6030,6080);
	Control_Info->Gimbal.Target.Left_Shoot=  -Control_Info->Gimbal.Target.Shoot_Speed;
  Control_Info->Gimbal.Target.Right_Shoot= Control_Info->Gimbal.Target.Shoot_Speed;
 
}
static void Gimbal_Info_Update(Control_Info_Typedef *Control_Info){
       PID_Calculate(&PID_Shoot[0] , Control_Info->Gimbal.Target.Left_Shoot , Control_Info->Gimbal.Measure.Left_shoot);
	     PID_Calculate(&PID_Shoot[1] , Control_Info->Gimbal.Target.Right_Shoot, Control_Info->Gimbal.Measure.Right_Shoot);

  if(Control_Info->Control_Mode==Shoot){
	Control_Info->SendValue[1] = PID_Shoot[0].Output;
  Control_Info->SendValue[2] = PID_Shoot[1].Output;
    }
	else if((Control_Info->Control_Mode == Vision)||(Control_Info->Control_Mode == Spin_Vision)){
	Control_Info->SendValue[1] = PID_Shoot[0].Output;
  Control_Info->SendValue[2] = PID_Shoot[1].Output;
	}
	
	else if(Control_Info->Control_Mode == Auto){
	Control_Info->SendValue[1] = PID_Shoot[0].Output;
  Control_Info->SendValue[2] = PID_Shoot[1].Output;
	}
 else if(Control_Info->Control_Mode==Off){
		if((Control_Info->Gimbal.Measure.Left_shoot>=1000)||(Control_Info->Gimbal.Measure.Right_Shoot>=1000)){
     Control_Info->SendValue[1] = PID_Calculate(&PID_Shoot[0] , 1000, Control_Info->Gimbal.Measure.Left_shoot);
	   Control_Info->SendValue[2] = PID_Calculate(&PID_Shoot[1] , 1000, Control_Info->Gimbal.Measure.Right_Shoot);
		}
		else{
			Control_Info->SendValue[1] = 0.0f;
			Control_Info->SendValue[2] = 0.0f;
		}
}
  else{
		if((Control_Info->Gimbal.Measure.Left_shoot>=1000)||(Control_Info->Gimbal.Measure.Right_Shoot>=1000)){
     Control_Info->SendValue[1] = PID_Calculate(&PID_Shoot[0] , 1000, Control_Info->Gimbal.Measure.Left_shoot);
	   Control_Info->SendValue[2] = PID_Calculate(&PID_Shoot[1] , 1000, Control_Info->Gimbal.Measure.Right_Shoot);
		}
		else{
			Control_Info->SendValue[1] = 0.0f;
			Control_Info->SendValue[2] = 0.0f;
		}
}	

}

static float SpeedAdapt(float real_S, float min_S, float max_S, float up_num, float down_num)
{
    float res = 0;
    static uint8_t SpeedErr_cnt = 0;

    if (real_S < min_S && real_S > 8)

        SpeedErr_cnt++;

    else if (real_S >= min_S && real_S <= max_S)

        SpeedErr_cnt = 0;

    if (SpeedErr_cnt == 1)
    {
        SpeedErr_cnt = 0;
        res += up_num;
    }
    if (real_S > max_S)
        res -= down_num;

    return res;
}


//火控
static void  Heat_Detection(){
if(Shoot_Info.Shoot_HeatCtrl.Qnow<0){
	Shoot_Info.Shoot_HeatCtrl.Time = 0.0f;
	Shoot_Info.Shoot_HeatCtrl.Qnow = 0.0f;
}	
else if(Shoot_Info.Shoot_HeatCtrl.Qnow>=0){
Shoot_Info.Shoot_HeatCtrl.Qnow-=(Shoot_Info.Shoot_HeatCtrl.Qcd* 0.001f);
}
if(DJI_Shoot_Motor[0].Data.Velocity<-5200&&DJI_Shoot_Motor[1].Data.Velocity>5200){		
if(DJI_Shoot_Motor[0].Data.Current<-2000&&DJI_Shoot_Motor[1].Data.Current>2000){
	if (remote_ctrl.rc.ch[4] > 200){
	 Shoot_Info.Shoot_HeatCtrl.Time++;
	if( Shoot_Info.Shoot_HeatCtrl.Shoot_Heat_Time ==20)
          Shoot_Info.Shoot_HeatCtrl.Qnow+=10;
		  Shoot_Info.Shoot_HeatCtrl.Shoot_Count+=1;
	Shoot_Info.Shoot_HeatCtrl.Shoot_Heat_Time =0;
	 if (Shoot_Info.Shoot_HeatCtrl.Qnow > Shoot_Info.Shoot_HeatCtrl.Qmax)  Shoot_Info.Shoot_HeatCtrl.Qnow = Shoot_Info.Shoot_HeatCtrl.Qmax;
	}
}
}
if(Control_Info.Control_Mode!=Control_Info.Gimbal_Last_Mode){
 Shoot_Info.Shoot_HeatCtrl.Shoot_Heat_Time = 0;
}

Shoot_Info.Shoot_HeatCtrl.Qres = Shoot_Info.Shoot_HeatCtrl.Qmax - Shoot_Info.Shoot_HeatCtrl.Qnow;
}


static bool IsOnTarget(const double cur_yaw,const double cur_pitch,const double target_yaw,const double target_pitch,const double distance) {                         
  // Judge whether to shoot
   shooting_range_yaw = fabs(atan2(0.165 / 2, distance));  //0.135
   shooting_range_pitch = fabs(atan2(3.0 / 2, distance));    //3

   pitch_ok =  fabs(cur_pitch - target_pitch) < shooting_range_pitch;
   yaw_ok = fabs(cur_yaw - target_yaw) < shooting_range_yaw;

  if (yaw_ok && pitch_ok) {
       
    return true;
  }
	  return false;
}
