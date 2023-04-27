/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_tick.c
  * @brief          : HAL delay functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : use the TIM2 as the HAL TimeBase
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "bsp_tick.h"

/**
 * the cubemx recommended to switch the system time base to a timer other than SysTick, 
 * so there will be two sets of time bases in the system, 1.SysTick for RTOS 2.HalTick for HAL
 * SysTick uses the SysTick of the cortex-m4 kernel (SysTick->VAL will be updated after starting the task scheduler)
 * HalTick uses TIM2 in this project (TIM2->CNT can provide microsecond delay)
 * # delay_us and delay_ms will not cause task scheduling (blocking type)
 * */
static uint32_t Micros(void)
{
    uint32_t haltick = 0;
    register uint32_t ms = 0, us= 0;

    ms = HAL_GetTick();
    // use the TIM2 as the HAL TimeBase
    // Freq:1MHz => 1Tick = 1us
    // Period:1ms
    us = TIM2->CNT;

    haltick = ms*1000 + us;

    return haltick;
}

// microsecond delay
void Delay_us(uint32_t us)
{
    uint32_t now = micros();

    while((micros() - now) < us);
}

// millisecond delay
void Delay_ms(uint32_t ms)
{
    uint32_t now = HAL_GetTick();

    while((HAL_GetTick()-now) < ms);
}

