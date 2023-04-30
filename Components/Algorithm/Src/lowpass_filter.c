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
#include "lowpass_filter.h"

/**
  * @brief first order low-pass filter init
  * @retval none
  */
void lpf_first_order_init(lpf_first_order_t *lpf,float alpha,float frame_period)
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
float lpf_first_order_calc(lpf_first_order_t *lpf,float input)
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
void lpf_second_order_init(lpf_second_order_t *lpf,float alpha[3])
{
  memcpy(lpf->alpha,alpha,sizeof(alpha));
  lpf->input = 0;
  memset(lpf->output,0,sizeof(lpf->output));
}
/**
  * @brief second order low-pass filter calculate
  * @retval filter output
  */
float lpf_second_order_calc(lpf_second_order_t *lpf,float input)
{
	lpf->input = input;
	lpf->output[0] = lpf->output[1];
	lpf->output[1] = lpf->output[2];
  lpf->output[2] = lpf->alpha[0]*lpf->output[1] + lpf->alpha[1]*lpf->output[0] + lpf->alpha[2]*lpf->input;

	return lpf->output[2];
}

