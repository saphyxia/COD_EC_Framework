/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Vision_Task.h
  * @brief          : Vision task
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef VISION_TASK_H
#define VISION_TASK_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"
#include "math.h"

/* Exported types ------------------------------------------------------------*/
/**
 * @brief typedef structure that contains the information for the Vision.
 */
typedef struct
{
  bool IF_Aiming_Enable;  /*!< vision aiming enable flag */

  bool IF_Fire_Accept;    /*!< fire acception */
  float Fire_Yaw_Threshold;   /*!< fire judgement threshold */

  float target_Pitch;   /*!< target pitch angle in radians */
  float target_Yaw;     /*!< target yaw angle in radians */

  float yawerror;      /*!< target pitch angle in radians */
}Vision_Info_Typedef;


#endif
