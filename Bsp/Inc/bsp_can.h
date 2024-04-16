#ifndef __BSP_CAN_H
#define __BSP_CAN_H
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_can.h
  * @brief          : Prototypes of can communication.
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


/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief  USER function to transmit message.
  * @param  data: point to the transmit message
  * @retval None
  */
extern void USER_CAN1_TxMessage(uint32_t StdId,uint8_t *data,uint8_t length);
//------------------------------------------------------------------------------

/**
  * @brief  USER function to transmit message.
  * @param  data: point to the transmit message
  * @retval None
  */
extern void USER_CAN2_TxMessage(uint32_t StdId,uint8_t *data,uint8_t length);
//------------------------------------------------------------------------------

#endif