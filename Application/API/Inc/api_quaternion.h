/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : api_quaternion.h
  * @brief          : quaternion fusion api
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef API_QUATERNION_H
#define API_QUATERNION_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "kalman.h"
#include "lpf.h"


/* Exported types ------------------------------------------------------------*/
/**
 * @brief typedef structure that contains the Information of Quaternion.
 */
typedef struct 
{
  bool init; /*!< Initialized flag */

  float quat[4];       /*!< the data of quaternion */
  float deviate[3];    /*!< the deviate of Gyro */


}Quaternion_Info_Typedef;

/* Exported functions prototypes ---------------------------------------------*/

#endif //API_QUATERNION_H


