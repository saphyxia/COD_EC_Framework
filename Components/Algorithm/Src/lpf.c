/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : lowpass_filter.c
  * @brief          : lowpass filter 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : To be perfected
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "lpf.h"
#include "string.h"

/**
  * @brief first order low-pass filter init
  * @retval none
  */
void LPF_First_Order_Init(LPF_FIRST_ORDER_t *lpf,float alpha,float frame_period)
{
  lpf->alpha = alpha;
  lpf->frame_period = frame_period;
  lpf->input = 0;
  lpf->output = 0;
}
/**
  * @brief first order low-pass filter calculate
  * @retval filter output
  */
float LPF_First_Order_Calc(LPF_FIRST_ORDER_t *lpf,float input)
{
  lpf->input = input;

  lpf->output = lpf->alpha/(lpf->alpha+lpf->frame_period)*lpf->output 
              + lpf->frame_period/(lpf->alpha+lpf->frame_period)*lpf->input;

  return lpf->output;
}
/**
  * @brief second order low-pass filter init
  * @retval none
  */
void LPF_Second_Order_Init(LPF_SECOND_ORDER_t *lpf,float alpha[3])
{
  memcpy(lpf->alpha,alpha,sizeof(lpf->alpha));
  lpf->input = 0;
  memset(lpf->output,0,sizeof(lpf->output));
}
/**
  * @brief second order low-pass filter calculate
  * @retval filter output
  */
float LPF_Second_Order_Calc(LPF_SECOND_ORDER_t *lpf,float input)
{
	lpf->input = input;
	lpf->output[0] = lpf->output[1];
	lpf->output[1] = lpf->output[2];
  lpf->output[2] = lpf->alpha[0]*lpf->output[1] + lpf->alpha[1]*lpf->output[0] + lpf->alpha[2]*lpf->input;

	return lpf->output[2];
}

