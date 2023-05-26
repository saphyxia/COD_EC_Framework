/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : motor.c
  * @brief          : motor interfaces functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : to be tested
  ******************************************************************************
  */
/* USER CODE END Header */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DEVICE_MOTOR_H
#define DEVICE_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "config.h"
#include "stm32f4xx.h"
#include "pid.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief typedef enum that contains the Frame Identifier for DJI Motor Device.
 */
typedef enum
{
  DJI_TxFrame_HIGH = 0x1ffU,
  DJI_TxFrame_LOW = 0x200U,
  DJI_RxFrame_MIDDLE = 0x204U,
  DJI_MotorFrameId_NUM,
}DJI_MotorFrameId_e;

/**
 * @brief typedef enum that contains the Error status for Motor Device.
 */
typedef enum
{
  MOTOR_ERROR_NONE = 0x00U,   /*!< no error */
  MOTOR_CAN_OFFLINE = 0x01U,    /*!< CAN transfer failed */
  MOTOR_OVER_TEMPERATURE = 0x02U,   /*!< abnormal motor temperature */
}Motor_Status_e;

/**
 * @brief typedef enum that contains the type of RMD Motor Device.
 */
typedef enum{
	  RMD_L9025,
    RMD_MOTOR_TYPE_NUM,
}RMD_Motor_Type_e;

/**
 * @brief typedef enum that contains the type of DJI Motor Device.
 */
typedef enum{
    DJI_GM6020,
    DJI_M3508,
    DJI_M2006,
    DJI_MOTOR_TYPE_NUM,
}DJI_Motor_Type_e;

/**
 * @brief typedef structure that contains the information for the Motor Error handler.
 */
typedef struct 
{
  uint16_t ErrorCount;    /*!< Error status judgment count */
  Motor_Status_e Status;   /*!< Error status */
}Motor_ErrorrHandler_Typedef;

/**
 * @brief typedef structure that contains the information for the Motor CAN Transfer.
 */
typedef struct
{
  uint32_t TxStdId;   /*!< Specifies CAN transfer identifier */
  uint32_t RxStdId;   /*!< Specifies CAN transfer identifier */
  uint8_t FrameIndex;   /* index for motor transmit frame */
  CAN_TypeDef *Instance;   /*!< pointer to the CAN Register base address */
  uint8_t *TxFrameBuff;   /*!< pointer to the CAN frame transmit buff */
}Motor_CANFrameInfo_typedef;

/**
 * @brief typedef structure that contains the General information for the Motor Device.
 */
typedef struct 
{
  bool Initlized;   /*!< init flag */

  int16_t  current;   /*!< Motor electric current */
  int16_t  velocity;    /*!< Motor rotate velocity */
  int16_t  encoder;   /*!< Motor encoder angle */
  int16_t  last_encoder;   /*!< previous Motor encoder angle */
  float    angle;   /*!< Motor angle in degree */
  uint8_t  temperature;   /*!< Motor Temperature */
}Motor_GeneralInfo_Typedef;

/**
 * @brief typedef structure that contains the information for the DJI Motor Device.
 */
typedef struct
{
	DJI_Motor_Type_e Type;   /*!< Type of Motor */
  Motor_CANFrameInfo_typedef CANFrame;    /*!< information for the CAN Transfer */
	Motor_GeneralInfo_Typedef Data;   /*!< information for the Motor Device */
	Motor_ErrorrHandler_Typedef ERRORHandler;   /*!< information for the Motor Error */
	PID_Info_TypeDef *positionPID;   /*!< pointer to position for the Motor controller */
	PID_Info_TypeDef *speedPID;      /*!< pointer to speed for the Motor controller */
}DJI_Motor_Info_Typedef;

/**
 * @brief typedef structure that contains the information for the DJI Motor Device.
 */
typedef struct
{
	uint8_t order;   /*!< Motor feedback order */
	RMD_Motor_Type_e Type;   /*!< Type of Motor */
  Motor_CANFrameInfo_typedef CANFrame;    /*!< information for the CAN Transfer */
	Motor_GeneralInfo_Typedef Data;   /*!< information for the Motor Device */
	Motor_ErrorrHandler_Typedef ERRORHandler;   /*!< information for the Motor Error */
}RMD_L9025_Info_Typedef;

/* Exported functions prototypes ---------------------------------------------*/

#endif //DEVICE_MOTOR_H
