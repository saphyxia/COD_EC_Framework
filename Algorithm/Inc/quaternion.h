#ifndef __QUATERNION_H
#define __QUATERNION_H
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : quaternion.h
  * @brief          : Header for quaternion.c file.
  * 
  ******************************************************************************
  * @attention      : none
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "kalman.h"

#define GravityAccel 9.8035f

/* Exported typedef ----------------------------------------------------------*/
/**
 * @brief structure that contains the Informations of quaternion.
 */
typedef struct 
{
  bool init; /*!< Initialize flag */

  float quat[4];       /*!< quaternion value */
  float offsets[3];    /*!< offsets of Gyro */
  matrix relation;     /*!< relation matrix */

  float Q1,Q2,R;       /*!< data of process and measurement noise */
  float *pdata_A;      /*!< point to data of state transition */
  float *pdata_P;      /*!< point to data of posteriori covariance */
  Kalman_Info_TypeDef QuatEKF;  /*!< Extended Kalman Filter */

  float accel[3];      /*!< data of accel */
  float gyro[3];       /*!< data of gyro */
  float accelInvNorm;  /*!< inverse of accel norm */
  float gyroInvNorm;   /*!< inverse of gyro norm */
  float halfgyrodt[3]; /*!< 0.5f*gyro*dt */
  float angle[3];      /*!< angle in radians: */
}Quat_Info_Typedef;

/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief Initializes the Quaternion EKF.
  * @param quat: point to a Quat_Info_Typedef structure that
  *         contains the informations of Quaternion EK
  * @param Q1/Q2: process noise
  * @param R: measurement noise
  * @param pdata_A: point to the data of state transition
  * @param pdata_P: point to the data of posteriori covariance
  */
extern void QuatEKF_Init(Quat_Info_Typedef *quat,float Q1,float Q2,float R,float *pdata_A,float *pdata_P);

/**
  * @brief  Update the Extended Kalman Filter
  * @param quat: point to a Quat_Info_Typedef structure that
  *         contains the informations of Quaternion EKF
  * @param gyro: point to the accel measurement
  * @param accel: point to the gyro measurement
  * @param dt: system latency
  */
extern void QuatEKF_Update(Quat_Info_Typedef *quat,float gyro[3],float accel[3],float dt);
//------------------------------------------------------------------------------

#endif
