/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : quaternion.c
  * Description        : Code for attitude algorithm
  ******************************************************************************
  * @author         : YuanBin Yan
  * @date           : 2024/02/23
  * @version        : 1.2.2
  * @attention      : none
  * 
  * rotation matrix
  * 1−2q2^2−2q3^2 2q1q2−2q0q3 2q1q3+2q0q2 
  * 2q1q2+2q0q3 1−2q1^2−2q3^2 2q2q3−2q0q1 
  * 2q1q3−2q0q2 2q2q3+2q0q1 1−2q1^2−2q2^2
  * 
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "quaternion.h"
#include "math.h"
#include "pid.h"

/* Private function ----------------------------------------------------------*/
/**
  * @brief  fast calculate the inverse square root
  * @note   see http://en.wikipedia.org/wiki/Fast_inverse_square_root
  * @retval the inverse square root of x
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
//------------------------------------------------------------------------------

/**
  * @brief Update the state transition
  * @param kf: point to a Kalman_Info_TypeDef structure that
  *         contains the informations of kalman filter.
  * @retval none
  */
static void QuatEKF_A_Update(Kalman_Info_TypeDef *kf)
{
  /* normalise quaternion */
  /* calc_vector[0][0] = 1.f / sqrt(q0^2.f + q1^2.f + q2^2.f + q3^2.f) */
  memset(kf->pdata.calc_vector[0],0,kf->sizeof_float * kf->xhatSize);
  kf->pdata.calc_vector[0][0] = Fast_InverseSqrt(kf->pdata.xhatminus[0]*kf->pdata.xhatminus[0] \
                                                +kf->pdata.xhatminus[1]*kf->pdata.xhatminus[1] \
                                                +kf->pdata.xhatminus[2]*kf->pdata.xhatminus[2] \
                                                +kf->pdata.xhatminus[3]*kf->pdata.xhatminus[3]);

  /* q0 = q0 / sqrt(q0^2.f + q1^2.f + q2^2.f + q3^2.f) */
  /* q1 = q1 / sqrt(q0^2.f + q1^2.f + q2^2.f + q3^2.f) */
  /* q2 = q2 / sqrt(q0^2.f + q1^2.f + q2^2.f + q3^2.f) */
  /* q3 = q3 / sqrt(q0^2.f + q1^2.f + q2^2.f + q3^2.f) */
  kf->pdata.xhatminus[0] *= kf->pdata.calc_vector[0][0];
  kf->pdata.xhatminus[1] *= kf->pdata.calc_vector[0][0];
  kf->pdata.xhatminus[2] *= kf->pdata.calc_vector[0][0];
  kf->pdata.xhatminus[3] *= kf->pdata.calc_vector[0][0];

  /**
   * @brief A = \frac{\partial f}{\partial x}
   *        1,        -halfgxdt,  -halfgydt,  -halfgzdt, (  0.5f*q1*dt,  0.5f*q2*dt )
   *        halfgxdt,  1,          halfgzdt,  -halfgydt, ( -0.5f*q0*dt,  0.5f*q3*dt )
   *        halfgydt, -halfgzdt,   1,          halfgxdt, ( -0.5f*q3*dt, -0.5f*q0*dt )
   *        halfgzdt,  halfgydt,  -halfgxdt,   1,        (  0.5f*q2*dt, -0.5f*q1*dt )
   *        0,         0,          0,          0,         1,            0 
   *        0,         0,          0,          0,         0,            1
   */
  kf->pdata.A[4]  =  0.5f*kf->pdata.xhatminus[1]*kf->dt;
  kf->pdata.A[5]  =  0.5f*kf->pdata.xhatminus[2]*kf->dt;

  kf->pdata.A[10] = -0.5f*kf->pdata.xhatminus[0]*kf->dt;
  kf->pdata.A[11] =  0.5f*kf->pdata.xhatminus[3]*kf->dt;

  kf->pdata.A[16] = -0.5f*kf->pdata.xhatminus[3]*kf->dt;
  kf->pdata.A[17] = -0.5f*kf->pdata.xhatminus[0]*kf->dt;

  kf->pdata.A[22] =  0.5f*kf->pdata.xhatminus[2]*kf->dt;
  kf->pdata.A[23] = -0.5f*kf->pdata.xhatminus[1]*kf->dt;

  /* Limit the P data */
  VAL_LIMIT(kf->pdata.P[28],-10000,10000);
  VAL_LIMIT(kf->pdata.P[35],-10000,10000);
}
//------------------------------------------------------------------------------

/**
  * @brief Update the state-measurement matrix
  * @param kf: point to a Kalman_Info_TypeDef structure that
  *         contains the informations of kalman filter.
  * @retval none
  */
static void QuatEKF_H_Update(Kalman_Info_TypeDef *kf)
{
  /**
   * @brief H = \frac{\partial h}{\partial x}
   *  -2.f*q2,  2.f*q3, -2.f*q0, 2.f*q1, 0, 0
   *   2.f*q1,  2.f*q0,  2.f*q3, 2.f*q2, 0, 0
   *   2.f*q0, -2.f*q1, -2.f*q2, 2.f*q3, 0, 0
   */
  memset(kf->pdata.H,0,kf->sizeof_float * kf->zSize * kf->xhatSize);

  kf->pdata.H[0]  = -2.f*kf->pdata.xhatminus[2];
  kf->pdata.H[1]  =  2.f*kf->pdata.xhatminus[3];
  kf->pdata.H[2]  = -2.f*kf->pdata.xhatminus[0];
  kf->pdata.H[3]  =  2.f*kf->pdata.xhatminus[1];

  kf->pdata.H[6]  =  2.f*kf->pdata.xhatminus[1];
  kf->pdata.H[7]  =  2.f*kf->pdata.xhatminus[0];
  kf->pdata.H[8]  =  2.f*kf->pdata.xhatminus[3];
  kf->pdata.H[9]  =  2.f*kf->pdata.xhatminus[2];

  kf->pdata.H[12] =  2.f*kf->pdata.xhatminus[0];
  kf->pdata.H[13] = -2.f*kf->pdata.xhatminus[1];
  kf->pdata.H[14] = -2.f*kf->pdata.xhatminus[2];
  kf->pdata.H[15] =  2.f*kf->pdata.xhatminus[3];
}
//------------------------------------------------------------------------------

/**
  * @brief Chi Square root Test
  * @param kf: point to a Kalman_Info_TypeDef structure that
  *         contains the informations of kalman filter.
  * @retval none
  */
static bool QuatEKF_ChiSqrtTest(Kalman_Info_TypeDef *kf)
{
  /* calc_matrix[0] = inverse(H·Pminus(k)·HT + R)·(z(k) - h(xhatminus)) */
  kf->mat.calc_matrix[0].numRows = kf->mat.calc_matrix[1].numRows;
  kf->mat.calc_matrix[0].numCols = 1;
  kf->ErrorStatus = Matrix_Multiply(&kf->mat.calc_matrix[1], &kf->mat.calc_vector[1], &kf->mat.calc_matrix[0]);

  /* calc_vector[0] = (z(k) - h(xhatminus)' */
  kf->mat.calc_vector[0].numRows = 1;
  kf->mat.calc_vector[0].numCols = kf->mat.calc_matrix[1].numRows;
  kf->ErrorStatus = Matrix_Transpose(&kf->mat.calc_matrix[1], &kf->mat.calc_vector[0]);

  /* ChiSquare_Matrix = (z(k) - h(xhatminus)'·inverse(H·Pminus·HT + R)·(z(k) - h(xhatminus)) */
  kf->ErrorStatus = Matrix_Multiply(&kf->mat.calc_vector[0], &kf->mat.calc_matrix[0], &kf->ChiSquareTest.ChiSquare_Matrix);

  /* rk is smaller,filter converg */ 
  if (kf->ChiSquareTest.ChiSquare_Data[0] < 0.5f * kf->ChiSquareTest.ChiSquareTestThresholds)
  {
    kf->ChiSquareTest.result = true;
  }
  /* rk is bigger */ 
  if (kf->ChiSquareTest.ChiSquare_Data[0] > kf->ChiSquareTest.ChiSquareTestThresholds && kf->ChiSquareTest.result)
  {
    if (kf->ChiSquareTest.TestFlag)
    {
      kf->ChiSquareTest.ChiSquareCnt++;
    }
    else
    {
      kf->ChiSquareTest.ChiSquareCnt = 0;
    }

    if (kf->ChiSquareTest.ChiSquareCnt > 50)
    {
      kf->ChiSquareTest.result = 0;
      kf->SkipStep5 = false;
    }
    else
    {
      /* xhat(k) = xhat'(k) */
      /* P(k) = P'(k) */
      memcpy(kf->pdata.xhat, kf->pdata.xhatminus, kf->sizeof_float * kf->xhatSize);
      memcpy(kf->pdata.P, kf->pdata.Pminus, kf->sizeof_float * kf->xhatSize * kf->xhatSize);

      /* skip the P update */
      kf->SkipStep5 = true;
      return true;
    }
  }
  else
  {
    if(kf->ChiSquareTest.ChiSquare_Data[0] > 0.1f * kf->ChiSquareTest.ChiSquareTestThresholds && kf->ChiSquareTest.result)
    {
      kf->pdata.calc_vector[0][0] = (kf->ChiSquareTest.ChiSquareTestThresholds - kf->ChiSquareTest.ChiSquare_Data[0]) / (0.9f * kf->ChiSquareTest.ChiSquareTestThresholds);
    }
    else
    {
      kf->pdata.calc_vector[0][0] = 1.f;
    }
    
    kf->ChiSquareTest.ChiSquareCnt = 0;
    kf->SkipStep5 = false;
  }
  return false;
}
//------------------------------------------------------------------------------
/**
  * @brief  Update the posteriori state estimate
  * @param  kf: point to a Kalman_Info_TypeDef structure that
  *         contains the informations of kalman filter.
  * @retval none
  */
static void QuatEKF_xhat_Update(Kalman_Info_TypeDef *kf)
{
  /* HT */
  kf->ErrorStatus = Matrix_Transpose(&kf->mat.H,&kf->mat.HT);

  /* calc_matrix[0] = H·Pminus(k) */
  kf->mat.calc_matrix[0].numRows = kf->mat.H.numRows;
  kf->mat.calc_matrix[0].numCols = kf->mat.Pminus.numCols;
  kf->ErrorStatus = Matrix_Multiply(&kf->mat.H, &kf->mat.Pminus, &kf->mat.calc_matrix[0]);

  /* calc_matrix[1] = H·Pminus(k)·HT */
  kf->mat.calc_matrix[1].numRows = kf->mat.calc_matrix[0].numRows;
  kf->mat.calc_matrix[1].numCols = kf->mat.HT.numCols;
  kf->ErrorStatus = Matrix_Multiply(&kf->mat.calc_matrix[0], &kf->mat.HT, &kf->mat.calc_matrix[1]); 
  
  /* K_d = H·Pminus(k)·HT + R */
  kf->mat.S.numRows = kf->mat.R.numRows;
  kf->mat.S.numCols = kf->mat.R.numCols;
  kf->ErrorStatus = Matrix_Add(&kf->mat.calc_matrix[1], &kf->mat.R, &kf->mat.S);

  /* calc_matrix[1] = inverse(H·Pminus(k)·HT + R) */
  kf->ErrorStatus = Matrix_Inverse(&kf->mat.S, &kf->mat.calc_matrix[1]);

  /* direction of gravity indicated by algorithm */
  kf->mat.calc_vector[0].numRows = kf->mat.H.numRows;
  kf->mat.calc_vector[0].numCols = 1;
  /* calc_vector[0][0] = 2.f*(q1*q3 - q0*q2) */
  /* calc_vector[0][1] = 2.f*(q0*q1 + q2*q3) */
  /* calc_vector[0][2] = q0^2.f - q1^2.f - q2^2.f + q3^2.f */
  kf->pdata.calc_vector[0][0] = 2.f * (kf->pdata.xhatminus[1] * kf->pdata.xhatminus[3] - kf->pdata.xhatminus[0] * kf->pdata.xhatminus[2]);
  kf->pdata.calc_vector[0][1] = 2.f * (kf->pdata.xhatminus[0] * kf->pdata.xhatminus[1] + kf->pdata.xhatminus[2] * kf->pdata.xhatminus[3]);
  kf->pdata.calc_vector[0][2] = kf->pdata.xhatminus[0] * kf->pdata.xhatminus[0] \
                              - kf->pdata.xhatminus[1] * kf->pdata.xhatminus[1] \
                              - kf->pdata.xhatminus[2] * kf->pdata.xhatminus[2] \
                              + kf->pdata.xhatminus[3] * kf->pdata.xhatminus[3];

  /* the cosine of three axis orientation */
	float OrientationCosine[3];
	for (uint8_t i = 0; i < 3; i++)
	{
		OrientationCosine[i] = acosf(fabsf(kf->pdata.calc_vector[0][i]));
	}
	
  /* calc_vector[1] = z(k) - h(xhat'(k)) */
  kf->mat.calc_vector[1].numRows = kf->mat.z.numRows;
  kf->mat.calc_vector[1].numCols = 1;
  kf->ErrorStatus = Matrix_Subtract(&kf->mat.z, &kf->mat.calc_vector[0], &kf->mat.calc_vector[1]);

  /* Chi Square root Test */
  if(QuatEKF_ChiSqrtTest(kf)==true)
  {
    return;
  }

  /* calc_matrix[0] = Pminus(k)·HT */
  kf->mat.calc_matrix[0].numRows = kf->mat.Pminus.numRows;
  kf->mat.calc_matrix[0].numCols = kf->mat.HT.numCols;
  kf->ErrorStatus = Matrix_Multiply(&kf->mat.Pminus, &kf->mat.HT, &kf->mat.calc_matrix[0]);

  /* k = Pminus·HT·inverse(H·Pminus·HT + R) */
  kf->ErrorStatus = Matrix_Multiply(&kf->mat.calc_matrix[0], &kf->mat.calc_matrix[1], &kf->mat.K);
	
	for(uint8_t i = 0; i < kf->mat.K.numCols*kf->mat.K.numRows; i++)
	{
		kf->pdata.K[i] *= kf->pdata.calc_vector[0][0];
	}

  /**
   * @brief K = \frac {P·minus·HT}{H·Pminus·HT + V·R·VT}
   *          = [  0,  1,  2,
   *               3,  4,  5,
   *               6,  7,  8,
   *               9, 10, 11, 
   *             (12, 13, 14,)
   *             (15, 16, 17,)]
   * @note  K[12..17] *=  cos(axis)/(PI/2.f)
   */
  for (uint8_t i = 4; i < 6; i++)
  {
    for (uint8_t j = 0; j < 3; j++)
    {
        kf->pdata.K[i * 3 + j] *= OrientationCosine[i - 4] / 1.5707963f; // 1 rad
    }
  }

  /* calc_vector[0] = K(k)·(z(k) - H·xhat'(k)) */
  kf->mat.calc_vector[0].numRows = kf->mat.K.numRows;
  kf->mat.calc_vector[0].numCols = 1;
  kf->ErrorStatus = Matrix_Multiply(&kf->mat.K, &kf->mat.calc_vector[1], &kf->mat.calc_vector[0]);

  if(kf->ChiSquareTest.result)
  {
    VAL_LIMIT(kf->pdata.calc_vector[0][4],-1e-2f*kf->dt,1e-2f*kf->dt);
    VAL_LIMIT(kf->pdata.calc_vector[0][5],-1e-2f*kf->dt,1e-2f*kf->dt);
  }
  kf->pdata.calc_vector[0][3] = 0;

  kf->ErrorStatus = Matrix_Add(&kf->mat.xhatminus, &kf->mat.calc_vector[0], &kf->mat.xhat);
}
//------------------------------------------------------------------------------

/**
  * @brief Initializes the Quaternion EKF.
  * @param quat: point to a Quat_Info_Typedef structure that
  *         contains the informations of Quaternion EK
  * @param Q1/Q2: process noise
  * @param R: measurement noise
  * @param pdata_A: point to the data of state transition
  * @param pdata_P: point to the data of posteriori covariance
  */
void QuatEKF_Init(Quat_Info_Typedef *quat,float Q1,float Q2,float R,float *pdata_A,float *pdata_P)
{
  /* store the data of process and measurement noise */
  quat->Q1 = Q1;
  quat->Q2 = Q2;
  quat->R  = R;

  /* store the data of state transition and posteriori covariance */
  quat->pdata_A = pdata_A;
  quat->pdata_P = pdata_P;

  /* Initialize the Extended kalman filter */
  Kalman_Filter_Init(&quat->QuatEKF,6,0,3);

  /* Initializes the relation matrix */
  quat->relation.numRows = 3;
  quat->relation.numCols = 3;
  quat->relation.pData = (float *)user_malloc(quat->QuatEKF.sizeof_float * quat->relation.numRows * quat->relation.numCols);
  memset(quat->relation.pData, 0, quat->QuatEKF.sizeof_float * quat->relation.numRows * quat->relation.numCols);

  /* Initializes the chi square test */
  quat->QuatEKF.ChiSquareTest.TestFlag = false;
  quat->QuatEKF.ChiSquareTest.result = false;
  quat->QuatEKF.ChiSquareTest.ChiSquareTestThresholds = 1e-8f;
  quat->QuatEKF.ChiSquareTest.ChiSquareCnt = 0;

  /* Initializes the position */
  quat->QuatEKF.pdata.xhat[0] = 1.f;
  quat->QuatEKF.pdata.xhat[1] = 0.f;
  quat->QuatEKF.pdata.xhat[2] = 0.f;
  quat->QuatEKF.pdata.xhat[3] = 0.f;

  quat->QuatEKF.User_Function1 = QuatEKF_A_Update;
  quat->QuatEKF.User_Function2 = QuatEKF_H_Update;
  quat->QuatEKF.User_Function3 = QuatEKF_xhat_Update;

  quat->QuatEKF.SkipStep3 = true;
  quat->QuatEKF.SkipStep4 = true;

  memcpy(quat->QuatEKF.pdata.A,quat->pdata_A,quat->QuatEKF.sizeof_float * quat->QuatEKF.xhatSize * quat->QuatEKF.xhatSize);
  memcpy(quat->QuatEKF.pdata.P,quat->pdata_P,quat->QuatEKF.sizeof_float * quat->QuatEKF.xhatSize * quat->QuatEKF.xhatSize);
}
//------------------------------------------------------------------------------

/**
  * @brief  Update the Extended Kalman Filter
  * @param quat: point to a Quat_Info_Typedef structure that
  *         contains the informations of Quaternion EKF
  * @param gyro: point to the accel measurement
  * @param accel: point to the gyro measurement
  * @param dt: system latency
  */
void QuatEKF_Update(Quat_Info_Typedef *quat,float gyro[3],float accel[3],float dt)
{
  /* store the system latency */
  quat->QuatEKF.dt = dt;

  /* offsets to gyro */
  quat->gyro[0] = gyro[0] - quat->offsets[0];
  quat->gyro[1] = gyro[1] - quat->offsets[1];
  quat->gyro[2] = gyro[2] - quat->offsets[2];

  /* gyroInvNorm = 1.f/(gyro[0]^2.f + gyro[1]^2.f + gyro[2]^2.f) */
  quat->gyroInvNorm = Fast_InverseSqrt(quat->gyro[0]*quat->gyro[0]+quat->gyro[1]*quat->gyro[1]+quat->gyro[2]*quat->gyro[2]);

  /* convert gyros to radians per second scaled by 0.5 */
  quat->halfgyrodt[0] = 0.5f * quat->gyro[0] * quat->QuatEKF.dt;
  quat->halfgyrodt[1] = 0.5f * quat->gyro[1] * quat->QuatEKF.dt;
  quat->halfgyrodt[2] = 0.5f * quat->gyro[2] * quat->QuatEKF.dt;

  /**
   * @brief A = \frac{\partial f}{\partial x}
   *        (1,        -halfgxdt,  -halfgydt,  -halfgzdt,)   0.5f*q1*dt,  0.5f*q2*dt
   *        (halfgxdt,  1,          halfgzdt,  -halfgydt,)  -0.5f*q0*dt,  0.5f*q3*dt
   *        (halfgydt, -halfgzdt,   1,          halfgxdt,)  -0.5f*q3*dt, -0.5f*q0*dt
   *        (halfgzdt,  halfgydt,  -halfgxdt,   1,       )   0.5f*q2*dt, -0.5f*q1*dt
   *        (0,         0,          0,          0,         1,            0 )
   *        (0,         0,          0,          0,         0,            1 )
   */
  memcpy(quat->QuatEKF.pdata.A,quat->pdata_A,quat->QuatEKF.sizeof_float * quat->QuatEKF.xhatSize * quat->QuatEKF.xhatSize);

  quat->QuatEKF.pdata.A[1]  = -quat->halfgyrodt[0];
  quat->QuatEKF.pdata.A[2]  = -quat->halfgyrodt[1];
  quat->QuatEKF.pdata.A[3]  = -quat->halfgyrodt[2];

  quat->QuatEKF.pdata.A[6]  =  quat->halfgyrodt[0];
  quat->QuatEKF.pdata.A[8]  =  quat->halfgyrodt[2];
  quat->QuatEKF.pdata.A[9]  = -quat->halfgyrodt[1];

  quat->QuatEKF.pdata.A[12] =  quat->halfgyrodt[1];
  quat->QuatEKF.pdata.A[13] = -quat->halfgyrodt[2];
  quat->QuatEKF.pdata.A[15] =  quat->halfgyrodt[0];

  quat->QuatEKF.pdata.A[18] =  quat->halfgyrodt[2];
  quat->QuatEKF.pdata.A[19] =  quat->halfgyrodt[1];
  quat->QuatEKF.pdata.A[20] = -quat->halfgyrodt[0];
	
	memcpy(quat->accel,accel,sizeof(quat->accel));

  /* accelInvNorm = 1.f/(accel[0]^2.f + accel[1]^2.f + accel[3]^2.f) */
  quat->accelInvNorm = Fast_InverseSqrt(quat->accel[0]*quat->accel[0]+quat->accel[1]*quat->accel[1]+quat->accel[2]*quat->accel[2]);
  quat->QuatEKF.MeasureInput[0] = quat->accel[0] * quat->accelInvNorm;
  quat->QuatEKF.MeasureInput[1] = quat->accel[1] * quat->accelInvNorm;
  quat->QuatEKF.MeasureInput[2] = quat->accel[2] * quat->accelInvNorm;
	 
  /* chi square test */
  if(1.f/quat->gyroInvNorm < 0.3f && 1.f/quat->accelInvNorm  > (GravityAccel-0.5f) && 1.f/quat->accelInvNorm < (GravityAccel+0.5f))
	{
		quat->QuatEKF.ChiSquareTest.TestFlag = true;
  }
  else
  {
    quat->QuatEKF.ChiSquareTest.TestFlag = false;
  }

  /* update the process/measurement noise covariance */
  quat->QuatEKF.pdata.Q[0]  = quat->Q1 * quat->QuatEKF.dt;
  quat->QuatEKF.pdata.Q[7]  = quat->Q1 * quat->QuatEKF.dt;
  quat->QuatEKF.pdata.Q[14] = quat->Q1 * quat->QuatEKF.dt;

  quat->QuatEKF.pdata.Q[21] = quat->Q1 * quat->QuatEKF.dt;
  quat->QuatEKF.pdata.Q[28] = quat->Q2 * quat->QuatEKF.dt;
  quat->QuatEKF.pdata.Q[35] = quat->Q2 * quat->QuatEKF.dt;

  quat->QuatEKF.pdata.R[0]  = quat->R;
  quat->QuatEKF.pdata.R[4]  = quat->R;
  quat->QuatEKF.pdata.R[8]  = quat->R;

  /* update the kalman filter */
  Kalman_Filter_Update(&quat->QuatEKF);

  /* Update the quaternion */
  quat->quat[0]    = quat->QuatEKF.Output[0];
  quat->quat[1]    = quat->QuatEKF.Output[1];
  quat->quat[2]    = quat->QuatEKF.Output[2];
  quat->quat[3]    = quat->QuatEKF.Output[3];
  quat->offsets[0] = quat->QuatEKF.Output[4];
  quat->offsets[1] = quat->QuatEKF.Output[5];
  quat->offsets[2] = 0.f;

  /* Update the relation matrix */
  quat->relation.pData[0] = 1 - 2.f*quat->quat[2]*quat->quat[2] - 2.f*quat->quat[3]*quat->quat[3];
  quat->relation.pData[1] = 2.f*quat->quat[1]*quat->quat[2] - 2.f*quat->quat[0]*quat->quat[3];
  quat->relation.pData[2] = 2.f*quat->quat[1]*quat->quat[3] + 2.f*quat->quat[0]*quat->quat[2];

  quat->relation.pData[3] = 2.f*quat->quat[1]*quat->quat[2] + 2.f*quat->quat[0]*quat->quat[3];
  quat->relation.pData[4] = 1 - 2.f*quat->quat[1]*quat->quat[1] - 2.f*quat->quat[3]*quat->quat[3];
  quat->relation.pData[5] = 2.f*quat->quat[2]*quat->quat[3] - 2.f*quat->quat[0]*quat->quat[1];

  quat->relation.pData[6] = 2.f*quat->quat[1]*quat->quat[3] - 2.f*quat->quat[0]*quat->quat[2];
  quat->relation.pData[7] = 2.f*quat->quat[2]*quat->quat[3] + 2.f*quat->quat[0]*quat->quat[1];
  quat->relation.pData[8] = 1 - 2.f*quat->quat[1]*quat->quat[1] + 2.f*quat->quat[2]*quat->quat[2];
  
	/* get angle in radians */
  quat->angle[0] = atan2f(2.f*(quat->quat[0]*quat->quat[3] + quat->quat[1]*quat->quat[2]), 2.f*(quat->quat[0]*quat->quat[0] + quat->quat[1]*quat->quat[1])-1.f);
  quat->angle[1] = asinf(-2.f*(quat->quat[1]*quat->quat[3] - quat->quat[0]*quat->quat[2]));
  quat->angle[2] = atan2f(2.f*(quat->quat[0]*quat->quat[1] + quat->quat[2]*quat->quat[3]), 2.f*(quat->quat[0]*quat->quat[0] + quat->quat[3]*quat->quat[3])-1.f);
}
//------------------------------------------------------------------------------
