/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_mcu.c
  * @brief          : MCU peripheral initialization functions
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : none
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "bsp_mcu.h"
#include "bsp_can.h"
#include "bsp_tim.h"
#include "bsp_uart.h"
#include "bmi088.h"
#include "INS_Task.h"

/**
  * @brief Initializes the MCU.
  */
void MCU_Init(void)
{
  /* ----------------------- BSP Init ----------------------- */
  Bsp_Tim_Init();
  /* ----------------------- Device Init ----------------------- */
  BMI088_Init();
}
//-------------------------------------------------------------------------------------------------------
