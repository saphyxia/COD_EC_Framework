#ifndef __TIMER_H
#define __TIMER_H
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : timer.h
  * @brief          : Header for timer.c file.
  * 
  ******************************************************************************
  * @attention      : none
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"


/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief  microsecond delay
  * @param  us: delay tick value 
  * @retval none
  */
extern void Delay_us(uint32_t us);
//------------------------------------------------------------------------------

/**
  * @brief  millisecond delay
  * @param  ms: delay tick value
  * @retval none
  */
extern void Delay_ms(uint32_t ms);
//------------------------------------------------------------------------------

/**
  * @brief  Starts the PWM signal generation.
  * @param  None
  * @retval None
  */
extern void TIM_PWM_Start(void);
//------------------------------------------------------------------------------

/**
  * @brief  Set the Heat_Power TIM Capture Compare Register value.
  * @param  compare: specifies the Capture Compare register new value.
  * @retval None
  */
extern void Heat_Power_Control(uint16_t compare);
//------------------------------------------------------------------------------

#endif