#ifndef __BSP_TIM_H
#define __BSP_TIM_H
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_tim.h
  * @brief          : Prototypes of PWM signal generation.
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


/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief  Starts the PWM signal generation.
  * @param  None
  * @retval None
  */
extern void TIM_PWM_Init(void);
//------------------------------------------------------------------------------

/**
  * @brief  Set the Heat_Power TIM Capture Compare Register value.
  * @param  compare: specifies the Capture Compare register new value.
  * @retval None
  */
extern void Heat_Power_Control(uint16_t compare);
//------------------------------------------------------------------------------

#endif