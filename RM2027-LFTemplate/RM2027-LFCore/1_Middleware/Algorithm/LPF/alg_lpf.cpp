/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : LPF.c
  * @brief          : lowpass filter
  * @author         : GrassFan Wang
  * @date           : 2025/12/28
  * @version        : v1.0
  ******************************************************************************
  * @attention      : To be perfected
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "alg_lpf.h"
#include "string.h"

float sign(float input){
   return (input>0.0f) - (input<0.0f);
}

/**
  * @brief 初始化一阶低通滤波器.
  * @param LPF: 一阶低通滤波器结构体.
  * @param Alpha: 滤波器系数.
  * @param Frame_Period: 采样周期.
  * @retval 无.
  */
void LowPassFilter1p_Init(LowPassFilter1p_Info_TypeDef *LPF,float Alpha)
{
  LPF->Alpha = Alpha;
  LPF->Input = 0;
  LPF->Output = 0;
}

/**
  * @brief 更新一阶低通滤波器输出.
  * @param Input: 当前输入.
  * @retval 滤波器输出.
  */
float LowPassFilter1p_Update(LowPassFilter1p_Info_TypeDef *LPF,float Input)
{
  LPF->Input = Input;

  if(LPF->Initialized == false)
  {
    LPF->Output = LPF->Input;
    LPF->Initialized = true;
  }

  LPF->Output = LPF->Alpha * LPF->Output + (1.f - LPF->Alpha) * LPF->Input;

  return LPF->Output;
}

/**
  * @brief 初始化二阶低通滤波器.
  * @param Alpha[3]: 滤波器系数[3].
  * @retval 无.
  */
void LowPassFilter2p_Init(LowPassFilter2p_Info_TypeDef *LPF,float Alpha[3])
{
  memcpy(LPF->Alpha,Alpha,sizeof(LPF->Alpha));
  LPF->Input = 0;
  memset(LPF->Output,0,sizeof(LPF->Output));
}

/**
  * @brief 更新二阶低通滤波器输出.
  * @param Input: 当前输入.
  * @retval 滤波器输出.
  */
float LowPassFilter2p_Update(LowPassFilter2p_Info_TypeDef *LPF,float Input)
{
  LPF->Input = Input;

  if(LPF->Initialized == false)
  {
    LPF->Output[0] = LPF->Input;
    LPF->Output[1] = LPF->Input;
    LPF->Output[2] = LPF->Input;
    LPF->Initialized = true;
  }

  LPF->Output[0] = LPF->Output[1];
  LPF->Output[1] = LPF->Output[2];
  LPF->Output[2] = LPF->Alpha[0] * LPF->Output[1] + LPF->Alpha[1] * LPF->Output[0] + LPF->Alpha[2] * LPF->Input;

  return LPF->Output[2];
}