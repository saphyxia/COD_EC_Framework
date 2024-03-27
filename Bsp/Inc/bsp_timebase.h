#ifndef __BSP_TIMBASE_H
#define __BSP_TIMBASE_H
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_timebase.h
  * @brief          : Prototypes of delay based on hal time base. 
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

#endif