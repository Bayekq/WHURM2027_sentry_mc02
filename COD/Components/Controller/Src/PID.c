
/* Includes ------------------------------------------------------------------*/
#include "PID.h"
/* Includes ------------------------------------------------------------------*/

/**
 * @brief 初始化PID参数.
 * @Param PID: 指向PID_Info_TypeDef结构的指针，包含PID控制器的信息.
 * @Param Param: 指向PID参数的浮点数指针，包含PID参数信息.
 * @retval PID错误状态.
 */
static PID_Status_e PID_Param_Init(PID_Info_TypeDef *PID,float Param[PID_PARAMETER_NUM])
{
    //判断PID类型和参数是否为空 若为空 返回PID_FAILED_INIT（初始化失败）
    if(PID->Type == PID_Type_None || Param == NULL)
    {
      return PID_FAILED_INIT;
    }
    
    //初始化PID参数
    PID->Param.KP = Param[0];
    PID->Param.KI = Param[1];
    PID->Param.KD = Param[2];
    PID->Param.Alpha = Param[3];
    if( PID->Param.Alpha > 0.f &&  PID->Param.Alpha < 1.f) 
		LowPassFilter1p_Init(&PID->Dout_LPF,PID->Param.Alpha);
		
		PID->Param.Deadband = Param[4];
    PID->Param.LimitIntegral = Param[5];
    PID->Param.LimitOutput = Param[6];

    //清除PID错误计数
    PID->ERRORHandler.ErrorCount = 0;

		//返回PID_ERROR_NONE（无错误状态）
    return PID_ERROR_NONE;
}
//------------------------------------------------------------------------------


/**
 * @brief 清除PID计算值，将所有输出赋0.
 * @Param PID: 指向PID_Info_TypeDef结构的指针，包含PID控制器的信息.
 * @retval 无.
 */
static void PID_Calc_Clear(PID_Info_TypeDef *PID)
{
	//将所有输出赋0
	memset(PID->Err,0,sizeof(PID->Err));
	PID->Integral = 0;
		
	PID->Pout = 0;
	PID->Iout = 0;
	PID->Dout = 0;
	PID->Output = 0;
}
//------------------------------------------------------------------------------


/**
 * @brief 初始化PID控制器.
 * @Param PID: 指向PID_Info_TypeDef结构的指针，包含PID控制器的信息.
 * @Param Type: PID控制器类型.
 * @Param Param:指向PID参数的浮点数指针，包含PID参数信息.
 * @retval PID错误状态.
 */
void PID_Init(PID_Info_TypeDef *PID,PID_Type_e Type,float Param[PID_PARAMETER_NUM])
{
	
		PID->Type = Type;

		PID->PID_Calc_Clear = PID_Calc_Clear;
    PID->PID_Param_Init = PID_Param_Init;

		PID->PID_Calc_Clear(PID);
    PID->ERRORHandler.Status = PID->PID_Param_Init(PID, Param);
}
//------------------------------------------------------------------------------


/**
  * @brief 判断PID错误状态
  * @Param PID: 指向PID_Info_TypeDef结构的指针，包含PID控制器的信息.
  * @retval 无.
  */
static void PID_ErrorHandle(PID_Info_TypeDef *PID)
{
		/* Judge NAN/INF */
		if(isnan(PID->Output) == true || isinf(PID->Output)==true)
		{
				PID->ERRORHandler.Status = PID_CALC_NANINF;
		}
}
//------------------------------------------------------------------------------

/**
  * @brief  PID控制器计算函数
  * @Param  *PID PID: 指向PID_Info_TypeDef结构的指针，包含PID控制器的信息.
  * @Param  Target：PID控制器的期望值
  * @Param  Measure PID控制器的实际值
  * @retval PID总输出
  */
float PID_Calculate(PID_Info_TypeDef *PID, float Target,float Measure)
{		
  /* 检查PID控制器错误状态 */
  PID_ErrorHandle(PID);
  if(PID->ERRORHandler.Status != PID_ERROR_NONE)
  {
    PID->PID_Calc_Clear(PID);
    return 0;
  }
  
  /* 更新PID期望值和实际值 */
  PID->Target =  Target;
  PID->Measure = Measure;

  /* 更新误差值 */
	PID->Err[2] = PID->Err[1];
	PID->Err[1] = PID->Err[0];
	PID->Err[0] = PID->Target - PID->Measure;
		
	/*  死区检查 误差值的绝对值小于死区 PID控制器停止计算 */
  if(fabsf(PID->Err[0]) >= PID->Param.Deadband)
  {
		/* 位置式PID */
		if(PID->Type == PID_POSITION)
		{
      /* 计算PID积分项 误差的累加 */
      if(PID->Param.KI != 0)
        PID->Integral += PID->Err[0];
      else
        PID->Integral = 0;

			/* 积分限幅 */
      VAL_LIMIT(PID->Integral,-PID->Param.LimitIntegral,PID->Param.LimitIntegral);
      
      /* 更新PID控制器的比例输出 积分输出 微分输出 */
      PID->Pout = PID->Param.KP * PID->Err[0];
      PID->Iout = PID->Param.KI * PID->Integral;
      PID->Dout = PID->Param.KD * (PID->Err[0] - PID->Err[1]);
			
			/* 微分输出的一阶低通滤波 */
      if( PID->Param.Alpha > 0.f &&  PID->Param.Alpha < 1.f){
			
				 PID->Dout_LPF.Alpha = PID->Param.Alpha;
			   PID->Dout = LowPassFilter1p_Update(&PID->Dout_LPF, PID->Dout);
			
			}
			
      /* 更新PID总输出 */
      PID->Output = PID->Pout + PID->Iout + PID->Dout;
      VAL_LIMIT(PID->Output,-PID->Param.LimitOutput,PID->Param.LimitOutput);
		}
		
		/* 增量式PID */
		else if(PID->Type == PID_VELOCITY)
		{
      /* 更新PID控制器的比例输出 积分输出 微分输出 */
      PID->Pout = PID->Param.KP * (PID->Err[0] - PID->Err[1]);
      PID->Iout = PID->Param.KI * (PID->Err[0]);
      PID->Dout = PID->Param.KD * (PID->Err[0] - 2.f*PID->Err[1] + PID->Err[2]);
			
			/* 更新PID控制器的比例输出 积分输出 微分输出 */
      if( PID->Param.Alpha > 0.f &&  PID->Param.Alpha < 1.f){
			
				 PID->Dout_LPF.Alpha = PID->Param.Alpha;
			   PID->Dout = LowPassFilter1p_Update(&PID->Dout_LPF, PID->Dout);
			
			}
      /* 更新PID总输出 */
      PID->Output += PID->Pout + PID->Iout + PID->Dout;
      VAL_LIMIT(PID->Output,-PID->Param.LimitOutput,PID->Param.LimitOutput);
		}
  }

	/* 返回PID总输出*/
  return PID->Output;
}
//------------------------------------------------------------------------------

