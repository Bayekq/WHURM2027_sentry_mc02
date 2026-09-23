#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H

#include "Shoot.h"
typedef struct
{
struct {
	float Shoot_Count;//连续射击的单位数量
	int Shoot_Heat_Time; // 射击热积累时间
	float Qmax;
	float Qcd;
	float Qnow;
	float Qres; 
	float Time;  //射击时间
	}Shoot_HeatCtrl;

	struct{
  float Pitch_Angle;
	float Fire_Trigger;
	float initial_speed;
  uint8_t initial_speed_test[4];
		
	}Shoot;

} Shoot_Info_Typedef;



extern Shoot_Info_Typedef Shoot_Info;

#endif