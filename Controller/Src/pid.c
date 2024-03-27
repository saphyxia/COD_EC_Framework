/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : pid.c
  * Description        : Implementation of pid
  ******************************************************************************
  * @author         : YuanBin Yan
  * @date           : 2024/02/22
  * @version        : 1.2.2
  * @attention      : none
  *
  * Copyright 2024 COD USTL.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "pid.h"

/**
 * @brief Initialize PID Parameters.
 * @param pid: pointer to PID_Info_TypeDef structure that
 *         contains the information of PID controller.
 * @param para: pointer to a floating-point array that
 *         contains the parameters for the PID controller.
 * @retval pid error status
 */
static uint8_t PID_Param_Init(PID_Info_TypeDef *pid,float para[PID_PARAMETER_NUM])
{
  /* check the type of pid and Null pointer */
  if(pid->type == PID_Type_None || para == NULL)
  {
    return 1;
  }

  /* Initialize the pid Parameters ------------------*/
  pid->param.kp = para[0];
  pid->param.ki = para[1];
  pid->param.kd = para[2];
  pid->param.Deadband = para[3];
  pid->param.MaxIntegral = para[4];
  pid->param.MaxOutput = para[5];

  /* Initialize the error count */
  pid->ERRORHandler.ErrorCount = 0;

  return 0;
}
//------------------------------------------------------------------------------


/**
 * @brief Clear Pid Calculation.
 * @param pid: pointer to PID_Info_TypeDef structure that
 *         contains the information of PID controller.
 * @retval none
 */
static void PID_Clear(PID_Info_TypeDef *pid)
{
	pid->err[0]=0;
	pid->err[1]=0;
	pid->err[2]=0;

	pid->integral = 0;
		
	pid->Pout = 0;
	pid->Iout = 0;
	pid->Dout = 0;
	pid->Output = 0;
}
//------------------------------------------------------------------------------

/**
 * @brief Initializes PID Controller.
 * @param pid: pointer to PID_Info_TypeDef structure that
 *         contains the information of PID controller.
 * @param type: type of pid controller
 * @param para: pointer to a floating-point array that
 *         contains the parameters for the PID controller.
 * @retval none
 */
void PID_Init(PID_Info_TypeDef *pid,PID_Type_e type,float para[PID_PARAMETER_NUM])
{
  pid->type = type;

  pid->Clear = PID_Clear;
  pid->Param_Init = PID_Param_Init;

  pid->Clear(pid);
  pid->ERRORHandler.INIT_FAILED = pid->Param_Init(pid, para);
}
//------------------------------------------------------------------------------

/**
  * @brief  detect the pid error status
  * @param pid: pointer to a PID_Info_TypeDef structure which
  *         contains the information of pid controller.
  * @retval None
  */
static void PID_ErrorHandle(PID_Info_TypeDef *pid)
{
  /* check NAN INF */
  if(isnan(pid->Output) == true || isinf(pid->Output) == true)
  {
    pid->ERRORHandler.RET_NAN_INF = 1;
  }
  else
  {
    pid->ERRORHandler.RET_NAN_INF = 0;
  }
}
//------------------------------------------------------------------------------

/**
  * @brief  Caculate the PID Controller
  * @param  *pid pointer to a PID_TypeDef_t structure that contains
  *              the configuration information for the specified PID. 
  * @param  Target  Target for the pid controller
  * @param  Measure Measure for the pid controller
  * @retval the Pid Output
  */
float f_PID_Calculate(PID_Info_TypeDef *pid, float target,float measure)
{		
  /* update error status */
  PID_ErrorHandle(pid);

  if(pid->ERRORHandler.INIT_FAILED != 0 || pid->ERRORHandler.RET_NAN_INF != 0)
  {
    pid->Clear(pid);
    return 0;
  }
  
  /* update the target/measure */
  pid->target = target;
  pid->measure = measure;

  /* update the errors */
	pid->err[2] = pid->err[1];
	pid->err[1] = pid->err[0];
	pid->err[0] = pid->target - pid->measure;

  if(fabsf(pid->err[0]) > pid->param.Deadband)
  {
		/* update the output */
		if(pid->type == PID_POSITION)
		{
      /* Update the Integral */
      if(pid->param.ki != 0)
        pid->integral += pid->err[0];
      else
        pid->integral = 0;

      VAL_LIMIT(pid->integral,-pid->param.MaxIntegral,pid->param.MaxIntegral);
      
      /* Update the Proportional Output,Integral Output and Derivative Output */
      pid->Pout = pid->param.kp * pid->err[0];
      pid->Iout = pid->param.ki * pid->integral;
      pid->Dout = pid->param.kd * (pid->err[0] - pid->err[1]);
      
      /* update the output */
      pid->Output = pid->Pout + pid->Iout + pid->Dout;
      VAL_LIMIT(pid->Output,-pid->param.MaxOutput,pid->param.MaxOutput);
		}
		else if(pid->type == PID_VELOCITY)
		{
      /* Update the Proportional Output,Integral Output and Derivative Output */
      pid->Pout = pid->param.kp * (pid->err[0] - pid->err[1]);
      pid->Iout = pid->param.ki * (pid->err[0]);
      pid->Dout = pid->param.kd * (pid->err[0] - 2.f*pid->err[1] + pid->err[2]);

      /* update the output */
      pid->Output += pid->Pout + pid->Iout + pid->Dout;
      VAL_LIMIT(pid->Output,-pid->param.MaxOutput,pid->param.MaxOutput);
		}
    else 
    {
      pid->Output = 0;
    }
  }

  return pid->Output;
}
//------------------------------------------------------------------------------
