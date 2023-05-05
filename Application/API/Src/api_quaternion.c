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




static void HalfGravity_Update(Quaternion_Info_Typedef *quat)
{
  switch (quat->settings.CoordinateSystem)
  {
    case CoordinateSystem_NWU:
    case CoordinateSystem_ENU: 
    /* third column of transposed rotation matrix scaled by 0.5 */
    /*!<  q1 * q3 - q0 * q2 */
    /*!<  q0 * q1 + q2 * q3 */
    /*!<  q0 * q0 + q3 * q3 - 0.5f */
    {
    }
    break;
    case CoordinateSystem_NED: 
    /* third column of transposed rotation matrix scaled by -0.5 */
    /*!<  q0 * q2 - q1 * q3 */
    /*!< -q0 * q1 - q2 * q3 */
    /*!<  0.5f - q0 * q0 + q3 * q3 */
    {
    }
    break;
    default:break;
  }
}

static void HalfAccelerometer_Update(Quaternion_Info_Typedef *quat)
{
  /* Normalise accelerometer measurement */

  /* Error is sum of cross product between estimated and measured direction of gravity */
  /*!< Halfax = ay * Halfgz - az * Halfgy */
  /*!< Halfay = az * Halfgx - ax * Halfgz */
  /*!< Halfaz = ax * Halfgy - ay * Halfgx */
}

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
