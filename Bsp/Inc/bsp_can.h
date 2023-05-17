/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_can.c
  * @brief          : bsp can functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : Pay attention to enable the can filter
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BSP_CAN_H
#define BSP_CAN_H


/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stm32f4xx.h"

/* Exported types ------------------------------------------------------------*/
/**
 * @brief typedef structure that contains the Information of CAN transmit.
 */
typedef struct {
		CAN_HandleTypeDef *hcan;
    CAN_RxHeaderTypeDef header;
    uint8_t 			data[8];
} CAN_RxFrameTypeDef;
/**
 * @brief typedef structure that contains the Information of CAN Receive.
 */
typedef struct {
		CAN_HandleTypeDef *hcan;
    CAN_TxHeaderTypeDef header;
    uint8_t				data[8];
}CAN_TxFrameTypeDef;

/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief  Configures the CAN Filter.
  */
extern void BSP_CAN_Init(void);
/**
  * @brief  USER function to transmit the Specifies message.
  */
extern void USER_CAN_TxMessage(CAN_TxFrameTypeDef *TxMessage);


#endif //BSP_CAN_H

