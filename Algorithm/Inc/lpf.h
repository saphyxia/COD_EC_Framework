#ifndef __LPF_H
#define __LPF_H
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : lpf.h
  * @brief          : Prototypes of low-pass filter.
  * 
  ******************************************************************************
  * @attention      : none
  *
  * Copyright 2024 COD USTL.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "arm_math.h"

/* Exported typedef ----------------------------------------------------------*/

typedef struct
{
  // alpha = 1.f/(1.f + tan(PI*fc/fs))
  // fc: cut-off frequency
  // fs: sampling frequency
  float alpha;
  float output_prev;  // last output 
}FirstOrderLowpass_Typedef;

typedef struct
{
  float alpha[3];
  float output_prev1;  // previous output
  float output_prev2;  // penultimate output
}SecondOrderLowpass_Typedef;

/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief Initialize the first order lowpass filter.
  * @param flpf: point to FirstOrderLowpass_Typedef structure that
  *         contains the informations of first order lowpass filter.
  * @param fc: cut-off frequency
  * @param fs: sampling frequency
  * @param init_output: initialized value of filter output
  * @retval none
  */
extern void FirstOrderLowpass_Init(FirstOrderLowpass_Typedef *flpf,float fc,float fs,float init_output);
//------------------------------------------------------------------------------

/**
  * @brief Update the first order lowpass filter.
  * @param flpf: point to FirstOrderLowpass_Typedef structure that
  *         contains the informations of first order lowpass filter.
  * @param input: value of filter input
  * @retval filter output
  */
extern float FirstOrderLowpass_Update(FirstOrderLowpass_Typedef *flpf,float input);
//------------------------------------------------------------------------------

/**
  * @brief Initialize the second order lowpass filter.
  * @param Slpf: point to SecondOrderLowpass_Typedef structure that
  *         contains the informations of second order lowpass filter.
  * @param alpha: filter parameters
  * @param init_output: initialized value of filter output
  * @retval none
  */
extern void SecondOrderLowpass_Init(SecondOrderLowpass_Typedef *Slpf,float alpha[3],float init_output);
//------------------------------------------------------------------------------

/**
  * @brief Update the second order lowpass filter.
  * @param Slpf: point to SecondOrderLowpass_Typedef structure that
  *         contains the informations of second order lowpass filter.
  * @param input: value of filter input
  * @retval filter output
  */
extern float SecondOrderLowpass_Update(SecondOrderLowpass_Typedef *Slpf,float input);
//------------------------------------------------------------------------------


#endif