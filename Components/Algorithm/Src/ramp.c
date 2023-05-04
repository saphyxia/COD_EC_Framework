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

#include "math.h"

/* Private define ------------------------------------------------------------*/
/**
  * @brief Euler's Number
  */
#define Euler_Number 2.718281828459045f

/**
  * @brief Calculate the floating-point ramp filter.
  * @param input: the filter input variables
  * @param target: the input variables target value
  * @param ramp: the ramp slope
  * @retval the filter output
  */
float f_Ramp_Calc(float input,float target,float ramp)
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

/**
  * @brief Calculate the floating-point logistic curves.
  * @param x: the curves input variables
  * @param k: the curves slope
  * @param x0: the curves phase
  * @note y = 1/(1+e^(-k*(x-x0)))
  *       k > 0: 1->0
  *       k < 0: 0->1
  * @retval the curves output
  */
float f_LogisticCurves_Calc(float x , float k ,float x0)
{
	float y = 0.f;
	
	if(k == 0.f)return 1.f;
	
	y = 1/(1+pow(Euler_Number,(k*(x-x0))));
	
	return y;
}


