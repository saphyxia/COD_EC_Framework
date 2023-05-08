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
 * @brief typedef enum that contains the type of CoordinateSystem.
 */
typedef enum
{
  CoordinateSystem_NWU = 0x00U,     /*!< North-West-Up World Coordinate System */
  CoordinateSystem_ENU = 0x01U,     /*!< East-North-Up Coordinate System */
  CoordinateSystem_NED = 0x02U,     /*!< North-East-Down Coordinate System */
  CoordinateSystem_Type_NUM,
}CoordinateSystem_Type_e;

/**
 * @brief typedef structure that contains the Settings of Quaternion.
 */
typedef struct 
{
  float alpha[3];     /*!< LPF coefficients */

  uint8_t xhatSize;   /*!<  state vector dimension */
  uint8_t uSize;      /*!<  control vector dimension */
  uint8_t zSize;      /*!<  measurement vector dimension */

  CoordinateSystem_Type_e CoordinateSystem;  /*!< the type of CoordinateSystem */
}Quaternion_Settings_Typedef;

/**
 * @brief typedef structure that contains the Information of Quaternion.
 */
typedef struct 
{
  Quaternion_Settings_Typedef settings;   /*!< the Settings of Quaternion */

  float q[4];   /*!< the array of Quaternion */
  float deviate[3];   /*!< the deviate of Gyro */

  LowPassFilter2p_Info_TypeDef accel_LPF2p;  /*!< the second order lowpass filter. */

  KalmanFilter_Info_TypeDef Quaternion_EKF;   /*!< Quaternion external kalman filter */

}Quaternion_Info_Typedef;

/* Exported functions prototypes ---------------------------------------------*/

#endif //API_QUATERNION_H


