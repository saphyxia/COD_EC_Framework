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
  * @brief Initializes the first order lowpass filter according to the specified parameters in the
  *         LPF_First_Order_TypeDef.
  * @param lpf: pointer to an LPF_First_Order_TypeDef structure that
  *         contains the information  for the first order lowpass filter.
  * @param alpha: filter coefficient
  * @param frame_period: frame perood
  * @retval none
  */
void LPF_First_Order_Init(LPF_First_Order_TypeDef *lpf,float alpha,float frame_period)
{
  lpf->alpha = alpha;
  lpf->frame_period = frame_period;
  lpf->input = 0;
  lpf->output = 0;
}

/**
  * @brief Update the first order lowpass filter according to the specified parameters in the
  *         LPF_First_Order_TypeDef.
  * @param kf: pointer to an LPF_First_Order_TypeDef structure that
  *         contains the information  for the first order lowpass filter.
  * @param input: the filter input
  * @retval first order lowpass filter output
  */
float LPF_First_Order_Update(LPF_First_Order_TypeDef *lpf,float input)
{
  lpf->input = input;

  lpf->output = lpf->alpha / (lpf->alpha + lpf->frame_period) * lpf->output 
              + lpf->frame_period / (lpf->alpha + lpf->frame_period) * lpf->input;

  return lpf->output;
}

/**
  * @brief Initializes the Second order lowpass filter according to the specified parameters in the
  *         LPF_Second_Order_TypeDef.
  * @param lpf: pointer to an LPF_Second_Order_TypeDef structure that
  *         contains the information  for the Second order lowpass filter.
  * @param alpha: filter coefficient
  * @param frame_period: frame perood
  * @retval none
  */
void LPF_Second_Order_Init(LPF_Second_Order_TypeDef *lpf,float alpha[3])
{
  memcpy(lpf->alpha,alpha,sizeof(lpf->alpha));
  lpf->input = 0;
  memset(lpf->output,0,sizeof(lpf->output));
}

/**
  * @brief Update the Second order lowpass filter according to the specified parameters in the
  *         LPF_Second_Order_TypeDef.
  * @param kf: pointer to an LPF_Second_Order_TypeDef structure that
  *         contains the information  for the Second order lowpass filter.
  * @param input: the filter input
  * @retval Second order lowpass filter output
  */
float LPF_Second_Order_Update(LPF_Second_Order_TypeDef *lpf,float input)
{
	lpf->input = input;
  
	lpf->output[0] = lpf->output[1];
	lpf->output[1] = lpf->output[2];
  lpf->output[2] = lpf->alpha[0] * lpf->output[0] + lpf->alpha[1] * lpf->output[1] + lpf->alpha[2] * lpf->input;

	return lpf->output[2];
}

