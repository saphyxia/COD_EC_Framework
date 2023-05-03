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
  * @brief Update the floating-point ramp filter.
  * @param input: the filter input variables
  * @param target: the input variables target value
  * @param ramp: the filter ramp
  * @retval the filter output
  */
float f_Ramp_Update(float input,float target,float ramp)
{
    float error = target - input;
    float output = input;

	if (error > 0){
        if (error > ramp){output += ramp;}   
        else{output += error;}
    }else{
        if (error < -ramp){output += -ramp;}
        else{output += error;}
    }

    return output;
}
