#ifndef __MOTOR_H
#define __MOTOR_H
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : motor.h
  * @brief          : Prototypes of motor communication.
  * 
  ******************************************************************************
  * @attention      : none
  *
  * Copyright 2024 COD USTL.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"
#include "stdlib.h"
#include "math.h"

/* Exported types ------------------------------------------------------------*/
/**
 * @brief enum the type of DJI Motor.
 */
typedef enum{
    DJI_GM6020,
    DJI_M3508,
    DJI_M2006,
    DJI_MOTOR_TYPE_NUM,
}DJI_Motor_Type_e;

/**
 * @brief typedef structure that contains the General information for the Motor Device.
 */
typedef struct 
{
  bool Initlized;   /*!< init flag */

  DJI_Motor_Type_e type;

  uint32_t RxStdId;       /*!< CAN receive identifier */

  int16_t  current;        /*!< electric current */
  int16_t  velocity;       /*!< rotate velocity */
  int16_t  encoder;        /*!< encoder angle */
  int16_t  encoder_prev;   /*!< previous encoder angle */
  float    angle;          /*!< angle in degree */
  uint8_t  temperature;    /*!< Temperature */
}DJI_Motor_Info_Typedef;

/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief  Update the DJI motor Information
  * @param  rxBuf  point to the can receive message
  * @param  DJI_Motor point to DJI_Motor_Info_t structure 
  *                   that contains the informations of DJI motor
  * @retval None
  */
extern void DJI_Motor_Info_Update(uint8_t *rxBuf,DJI_Motor_Info_Typedef *DJI_Motor);
//------------------------------------------------------------------------------

#endif