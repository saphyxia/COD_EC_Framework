/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : bsp_timebase.c
  * Description        : Implementation of delay based on hal time base. 
  ******************************************************************************
  * @author         : YuanBin Yan
  * @date           : 2024/02/23
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
#include "bsp_timebase.h"
#include "stm32f4xx.h"

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief    Provides a haltick value in microsecond.
  * @param    none
  * @details  the cubemx recommended to switch the system time base to a timer other than SysTick, 
  *           so there will be two sets of time bases in the system, 
  *			  1.SysTick for RTOS 
  * 		  2.HalTick for HAL
  *           SysTick of the cortex-m4 kernel (SysTick->VAL will be updated after starting the task scheduler)
  *           HalTick uses TIM2 in this project (TIM2->CNT can provide microsecond delay)
  *           functions void Delay_us(void) and void Delay_ms(void) will not cause task scheduling (blocking type)
  * @retval   tick value
  */
static uint32_t HAL_usTick(void)
{
  register uint32_t haltick = 0;
  register uint32_t ms = 0, us= 0;

  ms = HAL_GetTick();
  // use the TIM2 as the HAL TimeBase
  // Freq:1MHz => 1Tick = 1us
  // Period:1ms
  us = TIM2->CNT;

  haltick = ms*1000 + us;

  return haltick;
}
//------------------------------------------------------------------------------

/**
  * @brief  microsecond delay
  * @param  us: tick value 
  * @retval none
  */
void Delay_us(uint32_t us)
{
  uint32_t now = HAL_usTick();

  while((HAL_usTick() - now) < us);
}
//------------------------------------------------------------------------------


/**
  * @brief  millisecond delay
  * @param  ms: tick value
  * @retval none
  */
void Delay_ms(uint32_t ms)
{
  uint32_t now = HAL_GetTick();

  while((HAL_GetTick()-now) < ms);
}
//------------------------------------------------------------------------------
