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
  * @brief Calculate the floating-point ramp filter.
  */
extern float f_Ramp_Calc(float input,float target,float ramp);
/**
  * @brief Calculate the floating-point logistic curves.
  */
extern float f_LogisticCurves_Calc(float x , float k ,float x0);

#endif //RAMP_H

