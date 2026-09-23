/**
  ******************************************************************************
  * @file           : Control_Task.c
  * @brief          : Control task
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"


/**
 * @brief typedef structure that contains the information of chassis control
*/



typedef struct
{
float Yaw_Err;
	struct{
		float Last_Pitch_Angle;
    float Pitch_Angle;
		float Pitch_Gyro;
		float Last_Pitch_PID_Output;
		float Right_Shoot;
		float Left_Shoot;
		float Shoot_Speed;
		float Shoot_Speed_last;
		float Vision_Pitch_Angle;
		float Auto_Pitch_Angle;
	float	Auto_Pitch_Angle_err;
		float Pitch_PID_K;
	}Target;

	struct{
    float Pitch_Angle;
		float Pitch_Gyro;
		float Right_Shoot;
		float Left_shoot;

	}Measure;

	struct{
		float Pitch;
		float Pitch_Test;
		float Right_Shoot;
		float Left_Shoot;
    float Compensation;

	
	}Output;
	
	
  	struct{
		float Min;
		float Max;
	}Limit_Pitch;

}Gimbal_Info_Typedef;

typedef enum
{
    Off,
	Vision,
	Gimbal_Normal,
	Shoot,
	Auto,
	Chassis_Auto,
	Spin_Vision,
 Gimbal_Last_Mode,
} Control_Mode_Typedef;
	

typedef struct
{

  int16_t SendValue[5];
  Gimbal_Info_Typedef Gimbal;
  Control_Mode_Typedef Control_Mode;
  Control_Mode_Typedef Gimbal_Last_Mode; 
struct{
  float Power_Up_Retain;
	int Auto_to_Vision;
	}Symbol;

}Control_Info_Typedef;

/* Exported variables ----------------------------------------------------------*/
extern Control_Info_Typedef Control_Info;

#endif //CONTROL_TASK_H
