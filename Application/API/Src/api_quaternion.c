/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : api_quaternion.c
  * @brief          : quaternion fusion api
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  * @note           : see .\Docs\Quaternion.pdf
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "api_quaternion.h"
#include "math.h"
#include "pid.h"

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  fast calculate the inverse square root
  */
static float Fast_InverseSqrt(float x);




 
/**
  * @brief  fast calculate the inverse square root
  * @param  x: the input variable
  * @note   see http://en.wikipedia.org/wiki/Fast_inverse_square_root
  * @retval the inverse square root of input
  */
static float Fast_InverseSqrt(float x)
{

    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;

    i = 0x5f375a86 - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}
//-------------------------------------------------------------------------------------------------------


