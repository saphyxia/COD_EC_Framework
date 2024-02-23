/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : timer.c
  * Description        : Code for time base and PWM
  ******************************************************************************
  * @author         : YuanBin Yan
  * @date           : 2024/02/23
  * @version        : 1.2.2
  * @attention      : none
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "timer.h"
#include "stm32f4xx.h"
#include "tim.h"

/* Private function prototypes -----------------------------------------------*/

/**
  * @brief    Provides a tick value in microsecond.
  * @param    none
  * @details  the cubemx recommended to switch the system time base to a timer other than SysTick, 
  *           so there will be two sets of time bases in the system, 
	*					  1.SysTick for RTOS 
	* 				  2.HalTick for HAL
  *           SysTick of the cortex-m4 kernel (SysTick->VAL will be updated after starting the task scheduler)
  *           HalTick uses TIM2 in this project (TIM2->CNT can provide microsecond delay)
  *           functions void Delay_us(void) and void Delay_ms(void) will not cause task scheduling (blocking type)
  * @retval   tick value
  */
static uint32_t Get_Microsecond(void)
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
  * @param  us: delay tick value 
  * @retval none
  */
void Delay_us(uint32_t us)
{
  uint32_t now = Get_Microsecond();

  while((Get_Microsecond() - now) < us);
}
//------------------------------------------------------------------------------


/**
  * @brief  millisecond delay
  * @param  ms: delay tick value
  * @retval none
  */
void Delay_ms(uint32_t ms)
{
  uint32_t now = HAL_GetTick();

  while((HAL_GetTick()-now) < ms);
}
//------------------------------------------------------------------------------

/**
  * @brief  Starts the PWM signal generation.
  * @param  None
  * @retval None
  */
void TIM_PWM_Start(void)
{
	//Heat_Power PWM Start
	HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
}
//------------------------------------------------------------------------------

/**
  * @brief  Set the TIM Capture Compare Register value.
  * @param  htim TIM PWM handle
  * @param  Channel TIM Channels to be configured
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
  *            @arg TIM_CHANNEL_4: TIM Channel 4 selected
  * @param  compare specifies the Capture Compare register new value.
  * @retval None
  */
static void User_Tim_SetCompare(TIM_HandleTypeDef *htim,uint32_t Channel,uint16_t compare)
{
  switch (Channel)
  {
    case TIM_CHANNEL_1:
      htim->Instance->CCR1 = compare;
    break;

    case TIM_CHANNEL_2:
      htim->Instance->CCR2 = compare;
    break;

    case TIM_CHANNEL_3:
      htim->Instance->CCR3 = compare;
    break;

    case TIM_CHANNEL_4:
      htim->Instance->CCR4 = compare;
    break;

    default:break;
  }
}
//------------------------------------------------------------------------------

/**
  * @brief  Set the BMI088 Heat_Power TIM Capture Compare Register value.
  * @param  compare: specifies the Capture Compare register new value.
  * @retval None
  */
void Heat_Power_Control(uint16_t compare)
{
  User_Tim_SetCompare(&htim10,TIM_CHANNEL_1,compare);
}
//------------------------------------------------------------------------------



