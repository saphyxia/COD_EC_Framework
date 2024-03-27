#ifndef __IMU_TASK_H
#define __IMU_TASK_H
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : IMU_Task.h
  * @brief          : Prototypes of attitude algorithm
  *                   based on Mahony and Extended Kalman Filter.
  * 
  ******************************************************************************
  * @attention      : none
  *
  * Copyright 2024 COD USTL.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

/**
 * @brief radian to degrees , 180.f/PI
 */
#define RadiansToDegrees 57.295779513f

/* Exported types ------------------------------------------------------------*/

/**
 * @brief index of angle
 */
typedef enum
{
  IMU_ANGLE_INDEX_YAW = 0U,
  IMU_ANGLE_INDEX_PITCH = 1U,
  IMU_ANGLE_INDEX_ROLL = 2U,
}IMU_ANGLE_INDEX_e;

/**
 * @brief index of accel/gyro
 */
typedef enum
{
  IMU_ACCEL_GYRO_INDEX_ROLL = 0U,
  IMU_ACCEL_GYRO_INDEX_PITCH = 1U,
  IMU_ACCEL_GYRO_INDEX_YAW = 2U,
}IMU_ACCEL_GYRO_INDEX_e;

/**
 * @brief Instance structure that contains the informatiosn of IMU.
 */
typedef struct 
{
	float pit_angle;
	float yaw_angle;
	float yaw_tolangle;
	float rol_angle;

  float pit_gyro;
  float yaw_gyro;
  float rol_gyro;

  float angle[3];
	float gyro[3];	
	float accel[3];
	
	float last_yawangle;
	int16_t YawRoundCount;
}IMU_Info_Typedef;

/* Exported functions prototypes ---------------------------------------------*/

#endif

