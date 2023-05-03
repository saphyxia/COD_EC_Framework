/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : ramp.c
  * @brief          : ramp functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : To be perfected
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef RAMP_H
#define RAMP_H


/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief update the floating-point ramp filter.
  */
extern float f_Ramp_Update(float input,float target,float ramp);

#endif //RAMP_H

