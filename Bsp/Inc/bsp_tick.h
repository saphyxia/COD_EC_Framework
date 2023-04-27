/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_tick.c
  * @brief          : HAL delay functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : HAL clock source is TIM2
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BSP_TICK_H
#define BSP_TICK_H


/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"


/* Exported functions prototypes ---------------------------------------------*/
extern void Delay_us(uint32_t us);
extern void Delay_ms(uint32_t ms);

#endif //BSP_TICK_H
