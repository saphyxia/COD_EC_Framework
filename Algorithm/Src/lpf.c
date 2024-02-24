/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : lpf.c
  * Description        : Code for Low-pass filter
  ******************************************************************************
  * @author         : YuanBin Yan
  * @date           : 2024/02/24
  * @version        : 1.2.2
  * @attention      : none
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "lpf.h"

/**
  * @brief Initialize the first order lowpass filter.
  * @param flpf: point to FirstOrderLowpass_Typedef structure that
  *         contains the informations of first order lowpass filter.
  * @param fc: cut-off frequency
  * @param fs: sampling frequency
  * @param init_output: initialized value of filter output
  * @retval none
  */
void FirstOrderLowpass_Init(FirstOrderLowpass_Typedef *flpf,float fc,float fs,float init_output)
{
  flpf->alpha = 1.f/(1.f + tan(PI*fc/fs));
  flpf->output_prev = init_output;
}
//------------------------------------------------------------------------------

/**
  * @brief Update the first order lowpass filter.
  * @param flpf: point to FirstOrderLowpass_Typedef structure that
  *         contains the informations of first order lowpass filter.
  * @param input: value of filter input
  * @retval filter output
  */
float FirstOrderLowpass_Update(FirstOrderLowpass_Typedef *flpf,float input)
{
  float res = 0.f;

  res = flpf->alpha * flpf->output_prev + (1.f-flpf->alpha) * input;

  return res;
}
//------------------------------------------------------------------------------

/**
  * @brief Initialize the second order lowpass filter.
  * @param Slpf: point to SecondOrderLowpass_Typedef structure that
  *         contains the informations of second order lowpass filter.
  * @param alpha: filter parameters
  * @param init_output: initialized value of filter output
  * @retval none
  */
void SecondOrderLowpass_Init(SecondOrderLowpass_Typedef *Slpf,float alpha[3],float init_output)
{
  Slpf->alpha[0] = alpha[0];
  Slpf->alpha[1] = alpha[1];
  Slpf->alpha[2] = alpha[2];

  Slpf->output_prev1 = init_output;
  Slpf->output_prev2 = init_output;
}
//------------------------------------------------------------------------------

/**
  * @brief Update the second order lowpass filter.
  * @param Slpf: point to SecondOrderLowpass_Typedef structure that
  *         contains the informations of second order lowpass filter.
  * @param input: value of filter input
  * @retval filter output
  */
float SecondOrderLowpass_Update(SecondOrderLowpass_Typedef *Slpf,float input)
{
  float res = 0.f;

  res = Slpf->alpha[2] * Slpf->output_prev2  + Slpf->alpha[1] * Slpf->output_prev1 + Slpf->alpha[0] * input;
  Slpf->output_prev2 = Slpf->output_prev1;
  Slpf->output_prev1 = res;

  return res;
}
//------------------------------------------------------------------------------