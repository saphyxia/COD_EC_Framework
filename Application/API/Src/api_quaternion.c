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
  * @note 
  * 
  * Calculate direction of gravity indicated by algorithm:    
  * case CoordinateSystem_NWU:
  * case CoordinateSystem_ENU: 
  * third column of transposed rotation matrix scaled by 0.5
  * halfvx = q1 * q3 - q0 * q2
  * halfvy = q0 * q1 + q2 * q3
  * halfvz = q0 * q0 + q3 * q3 - 0.5f
  * case CoordinateSystem_NED: 
  * third column of transposed rotation matrix scaled by -0.5
  * halfvx =  q0 * q2 - q1 * q3
  * halfvy = -q0 * q1 - q2 * q3
  * halfvz =  0.5f - q0 * q0 + q3 * q3
  * 
  * Calculate accelerometer feedback scaled by 0.5:
  * ax = ax / InverseSqrt(ax*ax + ay*ay + az*az)
  * ay = ay / InverseSqrt(ax*ax + ay*ay + az*az)
  * az = az / InverseSqrt(ax*ax + ay*ay + az*az)
  * halfax = ay * halfvx - az * halfvx
  * halfay = az * halfvy - ax * halfvy
  * halfaz = ax * halfvz - ay * halfvz
  * 
  * gx -= 1.f * halfax
  * gy -= 1.f * halfay
  * gz -= 1.f * halfaz
  * 
  * Convert gyroscope to radians per second scaled by 0.5
  * halfgxdt = 0.5 * gx * dt
  * halfgydt = 0.5 * gy * dt
  * halfgzdt = 0.5 * gz * dt
  * 
  * q0 += -q1 * halfgxdt - q2 * halfgydt - q3 * halfgzdt
  * q1 +=  q0 * halfgxdt + q2 * halfgzdt - q3 * halfgydt
  * q1 +=  q0 * halfgydt - q1 * halfgzdt + q3 * halfgxdt
  * q1 +=  q0 * halfgzdt + q1 * halfgydt - q2 * halfgxdt
  * 
  * Normalise quaternion:
  * q0 = q0 * InverseSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
  * q1 = q1 * InverseSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
  * q2 = q2 * InverseSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
  * q3 = q3 * InverseSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "api_quaternion.h"
#include "math.h"

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  Evil floating point bit level hacking.
  */
static float f_InvSqrt(float number);





/**
  * @brief  Evil floating point bit level hacking.
  * @param  number: input variable
  * @retval y = 1 / sqrt(number)
  */
static float f_InvSqrt(float number)
{
	float halfnumber = 0.5f * number;
	float y = number;

	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfnumber * y * y));

	return y;
}
