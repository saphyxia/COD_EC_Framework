/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : bsp_tim.c
  * Description        : Implementation of PWM signal generation
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
#include "bsp_tim.h"
#include "tim.h"

/* Private function prototypes -----------------------------------------------*/

/**
  * @brief  Init the PWM Channel.
  * @param  None
  * @retval None
  */
void BSP_PWM_Init(void)
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



