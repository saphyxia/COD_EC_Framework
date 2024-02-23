/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : IMU_Task.c
  * Description        : Code for IMU task
  *                      using Mahony algorithm and Extended Kalman Filter
  ******************************************************************************
  * @author         : YuanBin Yan
  * @date           : 2024/02/22
  * @version        : 1.2.2
  * @attention      : 1. fix the usage error of osDelayUntil
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "IMU_Task.h"
#include "bmi088.h"

BMI088_Info_Typedef BMI088_Info;

/* USER CODE BEGIN Header_IMU_Task */
/**
* @brief Function implementing the IMUTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IMU_Task */
void IMU_Task(void const * argument)
{
  /* USER CODE BEGIN IMU_Task */
	
  // Holds the time at which the task was last unblocked.
  TickType_t ticks = 0;

  // Initialize the time.
  // Will be update in function osDelayUntil.
  ticks = osKernelSysTick();
	
  /* Infinite loop */
  for(;;)
  {
		// update bmi088 informations
		BMI088_Info_Update(&BMI088_Info);
		
    // Delay the task until 1 ms
    osDelayUntil(&ticks,1);
  }
  /* USER CODE END IMU_Task */
}
//------------------------------------------------------------------------------

