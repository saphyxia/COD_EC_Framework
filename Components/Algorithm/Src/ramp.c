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

/* Includes ------------------------------------------------------------------*/
#include "ramp.h"


/**
  * @brief ramp calculate
  * @retval ramp value
  */
float f_ramp_calc(float input,float target,float ramp)
{
    float buffer = target - input;
    float output = input;

	if (buffer > 0){
        if (buffer > ramp){output += ramp;}   
        else{output += buffer;}
    }else{
        if (buffer < -ramp){output += -ramp;}
        else{output += buffer;}
    }

    return output;
}
