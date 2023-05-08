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
  * ----------------------- Quaternion -----------------------
  * Initializes the Quaternion:
  * q0 = 1, q1 = 0, q2 = 0, q3 = 0
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
  * axNorm = ax / InverseSqrt(ax*ax + ay*ay + az*az)
  * ayNorm = ay / InverseSqrt(ax*ax + ay*ay + az*az)
  * azNorm = az / InverseSqrt(ax*ax + ay*ay + az*az)   
  * 
  * deviategx = 1.f * ( ayNorm * halfvx - azNorm * halfvx )
  * deviategy = 1.f * ( azNorm * halfvy - axNorm * halfvy )
  * deviategz = 1.f * ( axNorm * halfvz - ayNorm * halfvz )
  * 
  * Convert gyroscope to radians per second scaled by 0.5
  * halfgxdt = 0.5 * gx * dt
  * halfgydt = 0.5 * gy * dt
  * halfgzdt = 0.5 * gz * dt
  * 
  * q0 += -halfgxdt * q1 - halfgydt * q2 - halfgzdt * q3 + 0.5*q1*dt * deviategx + 0.5*q2*dt * deviategy + 0.5*q3*dt * deviategz
  * q1 +=  halfgxdt * q0 + halfgzdt * q2 - halfgydt * q3 - 0.5*q0*dt * deviategx + 0.5*q3*dt * deviategy - 0.5*q2*dt * deviategz
  * q2 +=  halfgydt * q0 - halfgzdt * q1 + halfgxdt * q3 - 0.5*q3*dt * deviategx - 0.5*q0*dt * deviategy + 0.5*q1*dt * deviategz
  * q3 +=  halfgzdt * q0 + halfgydt * q1 - halfgxdt * q2 + 0.5*q2*dt * deviategx - 0.5*q1*dt * deviategy - 0.5*q0*dt * deviategz
  * 
  * Normalise quaternion:
  * q0 *= InverseSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
  * q1 *= InverseSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
  * q2 *= InverseSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
  * q3 *= InverseSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
  * 
  * ----------------------- Extended Kalman Filter -----------------------
  * x(k) = f(x(k-1),u(k-1),w(k-1))
  * x = [q0,q1,q2,q3,deviategx,deviategy]T
  * z(k) = h(x(k),v(k))
  * z = [accelxNorm,accelyNorm,accelzNorm]T
  * 
  * A = \frac{\partial f}{\partial x} 
  * A[36]=[1, -halfgxdt, -halfgydt, -halfgzdt, +0.5*q1*dt, +0.5*q2*dt,
  *        +halfgxdt, 1, +halfgzdt, -halfgydt, -0.5*q0*dt, +0.5*q3*dt,
  *        +halfgydt, -halfgzdt, 1, +halfgxdt, -0.5*q3*dt, -0.5*q0*dt,
  *        +halfgzdt, +halfgydt, -halfgxdt, 1, +0.5*q2*dt, -0.5*q1*dt,
  *        0,          0,          0,         0,         1,         0,
  *        0,          0,          0,         0,         0,         1,]
  * 
  * H = \frac{\partial h}{\partial x}
  * H[18]=[]
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
