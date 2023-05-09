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
  * third column of transposed rotation matrix scaled by 0.5:
  * halfvx = q1 * q3 - q0 * q2
  * halfvy = q0 * q1 + q2 * q3
  * halfvz = q0 * q0 + q3 * q3 - 0.5f
  * 
  * Calculate accelerometer feedback scaled by 0.5:
  * axNorm = ax / InverseSqrt(ax*ax + ay*ay + az*az)
  * ayNorm = ay / InverseSqrt(ax*ax + ay*ay + az*az)
  * azNorm = az / InverseSqrt(ax*ax + ay*ay + az*az)  
  * 
  * deviategx = 1.f * ( ayNorm * halfvz - azNorm * halfvy )
  * deviategy = 1.f * ( azNorm * halfvx - axNorm * halfvz )
  * deviategz = 1.f * ( axNorm * halfvy - ayNorm * halfvx )
  * 
  * Convert gyroscope to radians per second scaled by 0.5
  * halfgxdt = 0.5 * gx * dt
  * halfgydt = 0.5 * gy * dt
  * halfgzdt = 0.5 * gz * dt
  * 
  * q0 += -halfgxdt * q1 - halfgydt * q2 - halfgzdt * q3 - 0.5*q1*dt * deviategx - 0.5*q2*dt * deviategy - 0.5*q3*dt * deviategz
  * q1 +=  halfgxdt * q0 + halfgzdt * q2 - halfgydt * q3 + 0.5*q0*dt * deviategx - 0.5*q3*dt * deviategy + 0.5*q2*dt * deviategz
  * q2 +=  halfgydt * q0 - halfgzdt * q1 + halfgxdt * q3 + 0.5*q3*dt * deviategx + 0.5*q0*dt * deviategy - 0.5*q1*dt * deviategz
  * q3 +=  halfgzdt * q0 + halfgydt * q1 - halfgxdt * q2 - 0.5*q2*dt * deviategx + 0.5*q1*dt * deviategy + 0.5*q0*dt * deviategz
  * 
  * Normalise quaternion:
  * q0 *= InverseSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
  * q1 *= InverseSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
  * q2 *= InverseSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
  * q3 *= InverseSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
  * 
  * ----------------------- Extended Kalman Filter -----------------------
  * x(k) = f(x(k-1),u(k-1),w(k-1))
  * z(k) = h(x(k),v(k))
  * 
  * x = [q0,q1,q2,q3,deviategx,deviategy]T
  * z = [accelxNorm,accelyNorm,accelzNorm]T
  * 
  * A = \frac{\partial f}{\partial x}:
  * A[36]=[1, -halfgxdt, -halfgydt, -halfgzdt, -0.5*q1*dt, -0.5*q2*dt,
  *        +halfgxdt, 1, +halfgzdt, -halfgydt, +0.5*q0*dt, -0.5*q3*dt,
  *        +halfgydt, -halfgzdt, 1, +halfgxdt, +0.5*q3*dt, +0.5*q0*dt,
  *        +halfgzdt, +halfgydt, -halfgxdt, 1, -0.5*q2*dt, +0.5*q1*dt,
  *        0,          0,          0,         0,         1,         0,
  *        0,          0,          0,         0,         0,         1,]
  * 
  * H = \frac{\partial h}{\partial x}:
  * H[18]=[0.5f∗q2∗dt/(halfvz),0.5f∗q3∗dt/(halfvz),-0.5f∗q0∗dt/(halfvz),-0.5f∗q1∗dt/(halfvz),0,-1/(halfvz),
  *        0.5f∗q1∗dt/(halfvz),-0.5f∗q0∗dt/(halfvz),-0.5f∗q3∗dt/(halfvz),0.5f∗q2∗dt/(halfvz),-1/(halfvz),0,
  *       0.5*q1*dt/halfvy - 0.5*q2*dt/halfvx, -0.5*q0*dt/halfvy - 0.5*q3*dt/halfvx, -0.5*q3*dt/halfvy + 0.5*q0*dt/halfvx,
  *       0.5*q2*dt/halfvy + 0.5*q1*dt/halfvx, -1/halfvy, 1/halfvx,]
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
  * @brief Initializes the Quaternion EKF according to the specified parameters in the
  *         Quaternion_Info_Typedef.
  * @param quaternion: pointer to a Quaternion_Info_Typedef structure that
  *         contains the information  for the Quaternion.
  * @param Settings: pointer to a Quaternion_Settings_Typedef structure that
  *         contains the settings  for the Quaternion.
  * @retval none
  */
static void QuaternionEKF_Init(Quaternion_Info_Typedef *quaternion,Quaternion_Settings_Typedef *Settings)
{
  /* Initializes the accel LPF2p */
  memcpy(quaternion->settings.alpha , Settings->alpha ,sizeof(quaternion->settings.alpha));

  /* Initializes the CoordinateSystem */
  quaternion->settings.CoordinateSystem = Settings->CoordinateSystem;

  /* Initializes the Kalman filter */
  quaternion->settings.xhatSize = Settings->xhatSize;
  quaternion->settings.uSize = Settings->uSize;
  quaternion->settings.zSize = Settings->zSize;
  quaternion->settings.ProcessNoise = Settings->ProcessNoise;
  quaternion->settings.MeasureNoise = Settings->MeasureNoise;
  Kalman_Filter_Init(&quaternion->QuaternionEKF,quaternion->settings.xhatSize,quaternion->settings.uSize,quaternion->settings.zSize);

  /* Initializes the ChiSquare matrix */
  memset(quaternion->ChiSquare_Data,0,sizeof(quaternion->ChiSquare_Data));
  Matrix_Init(&quaternion->ChiSquare, 1, 1, (float *)quaternion->ChiSquare_Data);

  /* Initializes the Quaternion */
  quaternion->q[0] = 1.f;
  quaternion->q[1] = 0.f;
  quaternion->q[2] = 0.f;
  quaternion->q[3] = 0.f;

  /* Set the IInitialized flag */
  quaternion->init = true;
}

static void QuaternionEKF_Update(Quaternion_Info_Typedef *quaternion)
{


}

/**
  * @brief Update the state transition matrix
  * @param quaternion: pointer to a Quaternion_Info_Typedef structure that
  *         contains the information  for the Quaternion.
  * @note  A = \frac{\partial f}{\partial x}
  * @retval none
  */
static void QuaternionEKF_A_Update(Quaternion_Info_Typedef *quaternion)
{

  /* Calculate the inverse square root of the quaternion norm */
  quaternion->qInverseSqrt = quaternion->QuaternionEKF.Memory.xhatminus[0] * quaternion->QuaternionEKF.Memory.xhatminus[0] \
                           + quaternion->QuaternionEKF.Memory.xhatminus[1] * quaternion->QuaternionEKF.Memory.xhatminus[1] \
                           + quaternion->QuaternionEKF.Memory.xhatminus[2] * quaternion->QuaternionEKF.Memory.xhatminus[2] \
                           + quaternion->QuaternionEKF.Memory.xhatminus[3] * quaternion->QuaternionEKF.Memory.xhatminus[3] ;
  
  /* Normalized quaternion */
  for(uint8_t i = 0; i < 4; i++)
  {
    *(quaternion->QuaternionEKF.Memory.xhatminus++) *= quaternion->qInverseSqrt; 
  }

  /**
   * @brief  Linearize at the operating point
   *  A[36]=[1, -halfgxdt, -halfgydt, -halfgzdt, (-0.5*q1*dt), (-0.5*q2*dt),
   *         +halfgxdt, 1, +halfgzdt, -halfgydt, (+0.5*q0*dt), (-0.5*q3*dt),
   *         +halfgydt, -halfgzdt, 1, +halfgxdt, (+0.5*q3*dt), (+0.5*q0*dt),
   *         +halfgzdt, +halfgydt, -halfgxdt, 1, (-0.5*q2*dt), (+0.5*q1*dt),
   *         0,          0,          0,         0,         1,         0,
   *         0,          0,          0,         0,         0,         1,]
   * */
  quaternion->QuaternionEKF.Memory.A[4]  = -0.5*quaternion->dt*quaternion->QuaternionEKF.Memory.xhatminus[1];
  quaternion->QuaternionEKF.Memory.A[5]  = -0.5*quaternion->dt*quaternion->QuaternionEKF.Memory.xhatminus[2];

  quaternion->QuaternionEKF.Memory.A[10] =  0.5*quaternion->dt*quaternion->QuaternionEKF.Memory.xhatminus[0];
  quaternion->QuaternionEKF.Memory.A[11] = -0.5*quaternion->dt*quaternion->QuaternionEKF.Memory.xhatminus[3];

  quaternion->QuaternionEKF.Memory.A[16] =  0.5*quaternion->dt*quaternion->QuaternionEKF.Memory.xhatminus[3];
  quaternion->QuaternionEKF.Memory.A[17] =  0.5*quaternion->dt*quaternion->QuaternionEKF.Memory.xhatminus[0];

  quaternion->QuaternionEKF.Memory.A[22] = -0.5*quaternion->dt*quaternion->QuaternionEKF.Memory.xhatminus[2];
  quaternion->QuaternionEKF.Memory.A[23] =  0.5*quaternion->dt*quaternion->QuaternionEKF.Memory.xhatminus[1];

}

/**
  * @brief Update the measurement transition matrix
  * @param quaternion: pointer to a Quaternion_Info_Typedef structure that
  *         contains the information  for the Quaternion.
  * @note  H = \frac{\partial h}{\partial x}
  * @retval none
  */
static void QuaternionEKF_H_Update(Quaternion_Info_Typedef *quaternion)
{
  /* Clear the matrix H */
  memset(quaternion->QuaternionEKF.Memory.H, 0, sizeof(float)*quaternion->settings.zSize*quaternion->settings.xhatSize);

  /**
   * @brief  Linearize at the operating point
   * H[18]=[0.5f∗q2∗dt/halfvz                ,  0.5f∗q3∗dt/halfvz                , -0.5f∗q0∗dt/halfvz                ,-0.5f∗q1∗dt/halfvz                ,0         ,-1/halfvz,
   *        0.5f∗q1∗dt/halfvz                , -0.5f∗q0∗dt/halfvz                , -0.5f∗q3∗dt/halfvz                , 0.5f∗q2∗dt/halfvz                ,-1/halfvz ,0        ,
   *        0.5*q1*dt/halfvy-0.5*q2*dt/halfvx, -0.5*q0*dt/halfvy-0.5*q3*dt/halfvx, -0.5*q3*dt/halfvy+0.5*q0*dt/halfvx,0.5*q2*dt/halfvy+0.5*q1*dt/halfvx, -1/halfvy, 1/halfvx,]
   * */
  quaternion->QuaternionEKF.Memory.H[0]  = 0.5f*quaternion->QuaternionEKF.Memory.xhatminus[2]*quaternion->dt / quaternion->halfv[2];
  quaternion->QuaternionEKF.Memory.H[1]  = 0.5f*quaternion->QuaternionEKF.Memory.xhatminus[3]*quaternion->dt / quaternion->halfv[2];
  quaternion->QuaternionEKF.Memory.H[2]  =-0.5f*quaternion->QuaternionEKF.Memory.xhatminus[0]*quaternion->dt / quaternion->halfv[2];
  quaternion->QuaternionEKF.Memory.H[3]  =-0.5f*quaternion->QuaternionEKF.Memory.xhatminus[1]*quaternion->dt / quaternion->halfv[2];
  quaternion->QuaternionEKF.Memory.H[4]  = 0.f;
  quaternion->QuaternionEKF.Memory.H[5]  = -1.f / quaternion->halfv[3];

  quaternion->QuaternionEKF.Memory.H[6]  = 0.5f*quaternion->QuaternionEKF.Memory.xhatminus[1]*quaternion->dt / quaternion->halfv[2];
  quaternion->QuaternionEKF.Memory.H[7]  =-0.5f*quaternion->QuaternionEKF.Memory.xhatminus[0]*quaternion->dt / quaternion->halfv[2];
  quaternion->QuaternionEKF.Memory.H[8]  =-0.5f*quaternion->QuaternionEKF.Memory.xhatminus[3]*quaternion->dt / quaternion->halfv[2];
  quaternion->QuaternionEKF.Memory.H[9]  = 0.5f*quaternion->QuaternionEKF.Memory.xhatminus[2]*quaternion->dt / quaternion->halfv[2];
  quaternion->QuaternionEKF.Memory.H[10] =-1.f / quaternion->halfv[3];
  quaternion->QuaternionEKF.Memory.H[11] = 0.f;

  quaternion->QuaternionEKF.Memory.H[12] = 0.5f*quaternion->QuaternionEKF.Memory.xhatminus[1]*quaternion->dt / quaternion->halfv[1] \
                                         - 0.5f*quaternion->QuaternionEKF.Memory.xhatminus[2]*quaternion->dt / quaternion->halfv[0];
  quaternion->QuaternionEKF.Memory.H[13] =-0.5f*quaternion->QuaternionEKF.Memory.xhatminus[0]*quaternion->dt / quaternion->halfv[1] \
                                         - 0.5f*quaternion->QuaternionEKF.Memory.xhatminus[3]*quaternion->dt / quaternion->halfv[0];
  quaternion->QuaternionEKF.Memory.H[14] =-0.5f*quaternion->QuaternionEKF.Memory.xhatminus[3]*quaternion->dt / quaternion->halfv[1] \
                                         + 0.5f*quaternion->QuaternionEKF.Memory.xhatminus[0]*quaternion->dt / quaternion->halfv[0];
  quaternion->QuaternionEKF.Memory.H[15] = 0.5f*quaternion->QuaternionEKF.Memory.xhatminus[2]*quaternion->dt / quaternion->halfv[1] \
                                         + 0.5f*quaternion->QuaternionEKF.Memory.xhatminus[1]*quaternion->dt / quaternion->halfv[0];
  quaternion->QuaternionEKF.Memory.H[16] =-1.f / quaternion->halfv[1];
  quaternion->QuaternionEKF.Memory.H[17] = 1.f / quaternion->halfv[0];
}

/**
  * @brief Update the Posteriori EstiMate
  * @param quaternion: pointer to a Quaternion_Info_Typedef structure that
  *         contains the information  for the Quaternion.
  * @note  xhat = xhatminus + K(k)·(z(k) - h(xhatminus))
  * @retval none
  */
static void QuaternionEKF_xhat_Update(Quaternion_Info_Typedef *quaternion)
{
  /* HT */
  quaternion->QuaternionEKF.MatStatus = Matrix_Transpose(&quaternion->QuaternionEKF.Mat.H, &quaternion->QuaternionEKF.Mat.HT);

  /* cache_matrix[0] = H·Pminus */
  quaternion->QuaternionEKF.Mat.cache_matrix[0].numRows = quaternion->QuaternionEKF.Mat.H.numRows;
  quaternion->QuaternionEKF.Mat.cache_matrix[0].numCols = quaternion->QuaternionEKF.Mat.Pminus.numCols;
  quaternion->QuaternionEKF.MatStatus = Matrix_Multiply(&quaternion->QuaternionEKF.Mat.H, &quaternion->QuaternionEKF.Mat.Pminus, &quaternion->QuaternionEKF.Mat.cache_matrix[0]);


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
