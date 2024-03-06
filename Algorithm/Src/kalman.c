/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : kalman.c
  * Description        : Code for kalman filter
  ******************************************************************************
  * @author         : YuanBin Yan
  * @date           : 2024/02/23
  * @version        : 1.2.2
  * @attention      : 1. fix comment of kalman formula
  * @note Adaptive kalman filter:
  *       1.xhatminus(k) = A·xhat(k-1) + B·u(k)
  *       2.Pminus(k) = A·P(k-1)·AT + Q
  *         R(k) = alpha·R(k-1)+(1-alpha)·(e(k)·e(k)T + H·Pminus(k)·HT)
  *       3.K(k) = Pminus(k)·HT/(H·Pminus(k)·HT + R)
  *       4.xhat(k) = xhatminus(k) + K(k)·(z(k) - H·xhatminus(k))
  *       5.P(k) = (I - K(k)·H)·Pminus(k)
  *         Q(k) = alpha·Q(k-1)+(1-alpha)·(K(k)·r(k)·r(k)T·K(k)T)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "kalman.h"

/**
  * @brief Initialize the kalman filter.
  * @param kf: point to  Kalman_Info_TypeDef structure that
  *         contains the informatrixions of kalman filter.
  * @param xhatSize: size of state vector
  * @param uSize: size of control vector
  * @param zSize: size of measurement vector
  * @retval none
  */
void Kalman_Filter_Init(Kalman_Info_TypeDef *kf,uint8_t xhatSize,uint8_t uSize,uint8_t zSize)
{
  /* store the size of float/double */
  kf->sizeof_float = sizeof(float);
  kf->sizeof_double = sizeof(double);
  
  /* check the size of state and measurement vector */
  if(xhatSize == 0 || zSize == 0)
  {
      kf->ErrorStatus = ARM_MATH_LENGTH_ERROR;  
  }
  
  /* store the size of state vector */
  kf->xhatSize = xhatSize;
  
  /* store the size of control vector */
  kf->uSize = uSize;
  
  /* store the size of measurement vector */      
  kf->zSize = zSize;
  
  /* Initialize the ChiSquare matrix */
  memset(kf->ChiSquareTest.ChiSquare_Data,0,sizeof(kf->ChiSquareTest.ChiSquare_Data));
  Matrix_Init(&kf->ChiSquareTest.ChiSquare_Matrix, 1, 1, (float *)kf->ChiSquareTest.ChiSquare_Data);
  
  if(true == kf->Adaptive_NoiseCov.Adaptive_Enable)
  {
    kf->Adaptive_NoiseCov.r.numRows = kf->zSize;
    kf->Adaptive_NoiseCov.r.numCols = 1;
    kf->Adaptive_NoiseCov.r.pData = (float *)user_malloc(kf->sizeof_float * zSize);
    memset(kf->Adaptive_NoiseCov.r.pData, 0, kf->sizeof_float * zSize);

    kf->Adaptive_NoiseCov.e.numRows = kf->zSize;
    kf->Adaptive_NoiseCov.e.numCols = 1;
    kf->Adaptive_NoiseCov.e.pData = (float *)user_malloc(kf->sizeof_float * zSize);
    memset(kf->Adaptive_NoiseCov.e.pData, 0, kf->sizeof_float * zSize);

    kf->Adaptive_NoiseCov.temp_vector.numRows = 1;
    kf->Adaptive_NoiseCov.temp_vector.numCols = kf->zSize;
    kf->Adaptive_NoiseCov.temp_vector.pData = (float *)user_malloc(kf->sizeof_float * zSize);
    memset(kf->Adaptive_NoiseCov.temp_vector.pData, 0, kf->sizeof_float * zSize);

    kf->Adaptive_NoiseCov.temp_matrix[0].numRows = kf->xhatSize;
    kf->Adaptive_NoiseCov.temp_matrix[0].numCols = kf->xhatSize;
    kf->Adaptive_NoiseCov.temp_matrix[0].pData = (float *)user_malloc(kf->sizeof_float * kf->xhatSize * kf->xhatSize);
    memset(kf->Adaptive_NoiseCov.temp_matrix[0].pData, 0, kf->sizeof_float * kf->xhatSize * kf->xhatSize);

    kf->Adaptive_NoiseCov.temp_matrix[1].numRows = kf->xhatSize;
    kf->Adaptive_NoiseCov.temp_matrix[1].numCols = kf->xhatSize;
    kf->Adaptive_NoiseCov.temp_matrix[1].pData = (float *)user_malloc(kf->sizeof_float * kf->xhatSize * kf->xhatSize);
    memset(kf->Adaptive_NoiseCov.temp_matrix[1].pData, 0, kf->sizeof_float * kf->xhatSize * kf->xhatSize);
  }

  /* Initialize the measurement Input */
  kf->MeasureInput = (float *)user_malloc(kf->sizeof_float * zSize);
  memset(kf->MeasureInput, 0, kf->sizeof_float * zSize);
  
  /* Initialize the posteriori state estimate */
  kf->pdata.xhat = (float *)user_malloc(kf->sizeof_float * xhatSize);
  memset(kf->pdata.xhat, 0, kf->sizeof_float * xhatSize);
  Matrix_Init(&kf->mat.xhat, kf->xhatSize, 1, (float *)kf->pdata.xhat);
  
  /* Initialize the priori state estimate */
  kf->pdata.xhatminus = (float *)user_malloc(kf->sizeof_float * xhatSize);
  memset(kf->pdata.xhatminus, 0, kf->sizeof_float * xhatSize);
  Matrix_Init(&kf->mat.xhatminus, kf->xhatSize, 1, (float *)kf->pdata.xhatminus);
  
  /* Initialize the measurement */
  kf->pdata.z = (float *)user_malloc(kf->sizeof_float * zSize);
  memset(kf->pdata.z, 0, kf->sizeof_float * zSize);
  Matrix_Init(&kf->mat.z, kf->zSize, 1, (float *)kf->pdata.z);
  
  if (kf->uSize != 0)
  {
    /* Initialize the control input */
    kf->ControlInput = (float *)user_malloc(kf->sizeof_float * uSize);
    memset(kf->ControlInput, 0, kf->sizeof_float * uSize);

    /* Initialize the control-input */ 
    kf->pdata.u = (float *)user_malloc(kf->sizeof_float * uSize);
    memset(kf->pdata.u, 0, kf->sizeof_float * uSize);
    Matrix_Init(&kf->mat.u, kf->uSize, 1, (float *)kf->pdata.u);

    /* Initialize the input-state matrix */  
    kf->pdata.B = (float *)user_malloc(kf->sizeof_float * xhatSize * uSize);
    memset(kf->pdata.B, 0, kf->sizeof_float * xhatSize * uSize);
    Matrix_Init(&kf->mat.B, kf->xhatSize, kf->uSize, (float *)kf->pdata.B);
  }
  
  /* Initialize the state transition matrix */ 
  kf->pdata.A = (float *)user_malloc(kf->sizeof_float * xhatSize * xhatSize);
  memset(kf->pdata.A, 0, kf->sizeof_float * xhatSize * xhatSize);
  Matrix_Init(&kf->mat.A, kf->xhatSize, kf->xhatSize, (float *)kf->pdata.A);
  
  kf->pdata.AT = (float *)user_malloc(kf->sizeof_float * xhatSize * xhatSize);
  memset(kf->pdata.AT, 0, kf->sizeof_float * xhatSize * xhatSize);
  Matrix_Init(&kf->mat.AT, kf->xhatSize, kf->xhatSize, (float *)kf->pdata.AT);
  
  /* Initialize the state-measurement matrix */ 
  kf->pdata.H = (float *)user_malloc(kf->sizeof_float * zSize * xhatSize);
  memset(kf->pdata.H, 0, kf->sizeof_float * zSize * xhatSize);
  Matrix_Init(&kf->mat.H, kf->zSize, kf->xhatSize, (float *)kf->pdata.H);
  
  kf->pdata.HT = (float *)user_malloc(kf->sizeof_float * xhatSize * zSize);
  memset(kf->pdata.HT, 0, kf->sizeof_float * xhatSize * zSize);
  Matrix_Init(&kf->mat.HT, kf->xhatSize, kf->zSize, (float *)kf->pdata.HT);
  
  /* Initialize the posteriori covariance matrix */
  kf->pdata.P = (float *)user_malloc(kf->sizeof_float * xhatSize * xhatSize);
  memset(kf->pdata.P, 0, kf->sizeof_float * xhatSize * xhatSize);
  Matrix_Init(&kf->mat.P, kf->xhatSize, kf->xhatSize, (float *)kf->pdata.P);
  
  /* Initialize the priori covariance matrix */
  kf->pdata.Pminus = (float *)user_malloc(kf->sizeof_float * xhatSize * xhatSize);
  memset(kf->pdata.Pminus, 0, kf->sizeof_float * xhatSize * xhatSize);
  Matrix_Init(&kf->mat.Pminus, kf->xhatSize, kf->xhatSize, (float *)kf->pdata.Pminus);
  
  /* Initialize the process noise covariance matrix */  
  kf->pdata.Q = (float *)user_malloc(kf->sizeof_float * xhatSize * xhatSize);
  memset(kf->pdata.Q, 0, kf->sizeof_float * xhatSize * xhatSize);
  Matrix_Init(&kf->mat.Q, kf->xhatSize, kf->xhatSize, (float *)kf->pdata.Q);
  
  /* Initialize the measurement noise covariance matrix */
  kf->pdata.R = (float *)user_malloc(kf->sizeof_float * zSize * zSize);
  memset(kf->pdata.R, 0, kf->sizeof_float * zSize * zSize);
  Matrix_Init(&kf->mat.R, kf->zSize, kf->zSize, (float *)kf->pdata.R);
  
  /* Initialize the kalman gain matrix */  
  kf->pdata.K = (float *)user_malloc(kf->sizeof_float * xhatSize * zSize);
  memset(kf->pdata.K, 0, kf->sizeof_float * xhatSize * zSize);
  Matrix_Init(&kf->mat.K, kf->xhatSize, kf->zSize, (float *)kf->pdata.K);
  
  /* Initialize the S matrix (S = H Pminus HT + R) */  
  kf->pdata.S = (float *)user_malloc(kf->sizeof_float * kf->xhatSize * kf->xhatSize);
  memset(kf->pdata.S, 0, kf->sizeof_float * kf->xhatSize * kf->xhatSize);
  Matrix_Init(&kf->mat.S, kf->xhatSize, kf->xhatSize, (float *)kf->pdata.S);
  
  /* Initialize the calculate process matrix */
  kf->pdata.calc_matrix[0] = (float *)user_malloc(kf->sizeof_float * kf->xhatSize * kf->xhatSize);
  memset(kf->pdata.calc_matrix[0],0,kf->sizeof_float * kf->xhatSize * kf->xhatSize);
  Matrix_Init(&kf->mat.calc_matrix[0], kf->xhatSize, kf->xhatSize, (float *)kf->pdata.calc_matrix[0]);
  
  kf->pdata.calc_matrix[1] = (float *)user_malloc(kf->sizeof_float * kf->xhatSize * kf->xhatSize);
  memset(kf->pdata.calc_matrix[1],0,kf->sizeof_float * kf->xhatSize * kf->xhatSize);
  Matrix_Init(&kf->mat.calc_matrix[1], kf->xhatSize, kf->xhatSize, (float *)kf->pdata.calc_matrix[1]);
  
  /* Initialize the calculate process vector */
  kf->pdata.calc_vector[0] = (float *)user_malloc(kf->sizeof_float * kf->xhatSize);
  memset(kf->pdata.calc_vector[0],0,kf->sizeof_float * kf->xhatSize);
  Matrix_Init(&kf->mat.calc_vector[0], kf->xhatSize, 1, (float *)kf->pdata.calc_vector[0]);
  
  kf->pdata.calc_vector[1] = (float *)user_malloc(kf->sizeof_float * kf->xhatSize);
  memset(kf->pdata.calc_vector[1],0,kf->sizeof_float * kf->xhatSize);
  Matrix_Init(&kf->mat.calc_vector[1], kf->xhatSize, 1, (float *)kf->pdata.calc_vector[1]);
  
  /* Initialize the filter output */
  kf->Output = (float *)user_malloc(kf->sizeof_float * xhatSize);
  memset(kf->Output, 0, kf->sizeof_float * xhatSize);
}
//------------------------------------------------------------------------------

/**
  * @brief Update the input of kalman
  * @param kf: point to a Kalman_Info_TypeDef structure that
  *         contains the informations of kalman filter.
  * @retval none
  */
static void Kalman_Input_Update(Kalman_Info_TypeDef *kf)
{
  /* store the measuerment vector */
  memcpy(kf->pdata.z, kf->MeasureInput, kf->sizeof_float * kf->zSize);
  
  /* clear the measuerment vector */
  memset(kf->MeasureInput, 0, kf->sizeof_float * kf->zSize);
  
  if(kf->uSize > 0)
  {
    /* store the control-input vector */
    memcpy(kf->pdata.u, kf->ControlInput, kf->sizeof_float * kf->uSize);
  }
}
//------------------------------------------------------------------------------


/**
  * @brief Update the priori state estimate
  * @param kf: point to a Kalman_Info_TypeDef structure that
  *         contains the informations of kalman filter.
  * @note xhatminus(k) = A·xhat(k-1) + B·u(k)
  * @retval none
  */
static void Kalman_xhatminus_Update(Kalman_Info_TypeDef *kf)
{
  /* skip the step */
  if(kf->SkipStep1 == 1)
  {
    return;
  }

  if(kf->uSize > 0)
  {
    /* calc_vector[0] = A·xhat(k-1) */ 
    kf->mat.calc_vector[0].numRows = kf->xhatSize;
    kf->mat.calc_vector[0].numCols = 1;
    kf->ErrorStatus = Matrix_Multiply(&kf->mat.A, &kf->mat.xhat, &kf->mat.calc_vector[0]);   

    /* calc_vector[1] = B·u(k) */ 
    kf->mat.calc_vector[0].numRows = kf->xhatSize;
    kf->mat.calc_vector[0].numCols = 1;
    kf->ErrorStatus = Matrix_Multiply(&kf->mat.B, &kf->mat.u, &kf->mat.calc_vector[1]);    

    /* xhatminus(k) = A·xhat(k-1) + B·u(k) */
    kf->ErrorStatus = Matrix_Add(&kf->mat.calc_vector[0], &kf->mat.calc_vector[1], &kf->mat.xhatminus);   
  }
  else
  {
    /* xhatminus(k) = A·xhat(k-1) */
    kf->ErrorStatus = Matrix_Multiply(&kf->mat.A, &kf->mat.xhat, &kf->mat.xhatminus);   
  }
}
//------------------------------------------------------------------------------

/**
  * @brief Update the priori covariance
  * @param kf: point to a Kalman_Info_TypeDef structure that
  *         contains the informations of kalman filter.
  * @note Pminus(k) = A·P(k-1)·AT + Q
  * @retval none
  */
static void Kalman_Pminus_Update(Kalman_Info_TypeDef *kf)
{
  /* skip this step */
  if(kf->SkipStep2 == 1)
  {
    return;
  }

  /* AT */
  kf->ErrorStatus = Matrix_Transpose(&kf->mat.A, &kf->mat.AT); 

  /* Pminus = A·P(k-1) */ 
  kf->ErrorStatus = Matrix_Multiply(&kf->mat.A, &kf->mat.P, &kf->mat.Pminus); 

  /* calc_matrix[0] = A·P(k-1)·AT */ 
  kf->mat.calc_matrix[0].numRows = kf->mat.Pminus.numRows;
  kf->mat.calc_matrix[0].numCols = kf->mat.AT.numCols;
  kf->ErrorStatus = Matrix_Multiply(&kf->mat.Pminus, &kf->mat.AT, &kf->mat.calc_matrix[0]); 

  /* Pminus(k) = A·P(k-1)·AT + Q */
  kf->ErrorStatus = Matrix_Add(&kf->mat.calc_matrix[0], &kf->mat.Q, &kf->mat.Pminus); 
}
//------------------------------------------------------------------------------

/**
  * @brief Update the kalman gain
  * @param kf: point to a Kalman_Info_TypeDef structure that
  *         contains the informations of kalman filter.
  * @note  K(k) = Pminus(k)·HT/(H·Pminus(k)·HT + R)
  * @retval none
  */
static void Kalman_K_Update(Kalman_Info_TypeDef *kf)
{
  /* skip this step */
  if(kf->SkipStep3 == 1)
  {
    return;
  }

  /* HT */
  kf->ErrorStatus = Matrix_Transpose(&kf->mat.H, &kf->mat.HT); 

  /* calc_matrix[0] = H·Pminus(k) */
  kf->mat.calc_matrix[0].numRows = kf->mat.H.numRows;
  kf->mat.calc_matrix[0].numCols = kf->mat.Pminus.numCols;
  kf->ErrorStatus = Matrix_Multiply(&kf->mat.H, &kf->mat.Pminus, &kf->mat.calc_matrix[0]); 

  /* calc_matrix[1] = H·Pminus(k)·HT */
  kf->mat.calc_matrix[1].numRows = kf->mat.calc_matrix[0].numRows;
  kf->mat.calc_matrix[1].numCols = kf->mat.HT.numCols;
  kf->ErrorStatus = Matrix_Multiply(&kf->mat.calc_matrix[0], &kf->mat.HT, &kf->mat.calc_matrix[1]);  

  if(true == kf->Adaptive_NoiseCov.Adaptive_Enable)
  {
    /* e(k)=z(k)-xhatminus(k) */
    kf->ErrorStatus = Matrix_Subtract(&kf->mat.z,&kf->mat.xhatminus,&kf->Adaptive_NoiseCov.e);

    /* e(k)T */
    kf->ErrorStatus = Matrix_Transpose(&kf->Adaptive_NoiseCov.e,&kf->Adaptive_NoiseCov.temp_vector);

    /* temp_matrix[1] = e(k)·e(k)T */
    kf->Adaptive_NoiseCov.temp_matrix[1].numRows = kf->Adaptive_NoiseCov.e.numRows;
    kf->Adaptive_NoiseCov.temp_matrix[1].numCols = kf->Adaptive_NoiseCov.temp_vector.numCols;
    kf->ErrorStatus = Matrix_Multiply(&kf->Adaptive_NoiseCov.e,&kf->Adaptive_NoiseCov.temp_vector,&kf->Adaptive_NoiseCov.temp_matrix[1]);

    /* temp_matrix[0] = e(k)·e(k)T + H·Pminus(k)·HT */
    kf->ErrorStatus = Matrix_Add(&kf->Adaptive_NoiseCov.temp_matrix[1],&kf->mat.calc_matrix[1],&kf->Adaptive_NoiseCov.temp_matrix[0]);

    kf->mat.calc_matrix[0].numRows = kf->mat.R.numRows;
    kf->mat.calc_matrix[0].numCols = kf->mat.R.numCols; 

    /* alpha·R(k-1), (1-alpha)·(e(k)·e(k)T + H·Pminus(k)·HT) */
    for(uint8_t i=0;i < (kf->zSize*kf->zSize);i++)
    {
      kf->pdata.calc_matrix[0][i] = kf->Adaptive_NoiseCov.alpha*kf->pdata.R[i];
      kf->Adaptive_NoiseCov.temp_matrix[0].pData[i] *= (1.f-kf->Adaptive_NoiseCov.alpha);
    }
    
    /* R(k) = alpha·R(k-1)+(1-alpha)·(e(k)·e(k)T + H·Pminus(k)·HT) */
    kf->ErrorStatus = Matrix_Add(&kf->mat.calc_matrix[0],&kf->Adaptive_NoiseCov.temp_matrix[0],&kf->mat.R);
  }

  /* S = H·Pminus(k)·HT + R */
  kf->mat.S.numRows = kf->mat.R.numRows;
  kf->mat.S.numCols = kf->mat.R.numCols;
  kf->ErrorStatus = Matrix_Add(&kf->mat.calc_matrix[1], &kf->mat.R, &kf->mat.S); 

  /* calc_matrix[1] = inverse(H·Pminus(k)·HT + R) */
  kf->ErrorStatus = Matrix_Inverse(&kf->mat.S, &kf->mat.calc_matrix[1]);

  /* calc_matrix[0] = Pminus(k)·HT */
  kf->mat.calc_matrix[0].numRows = kf->mat.Pminus.numRows;
  kf->mat.calc_matrix[0].numCols = kf->mat.HT.numCols;
  kf->ErrorStatus = Matrix_Multiply(&kf->mat.Pminus, &kf->mat.HT, &kf->mat.calc_matrix[0]);

  /* K(k) = Pminus(k)·HT / (H·Pminus(k)·HT + R) */
  kf->ErrorStatus = Matrix_Multiply(&kf->mat.calc_matrix[0], &kf->mat.calc_matrix[1], &kf->mat.K);
}
//------------------------------------------------------------------------------

/**
  * @brief Update the posteriori state estimate
  * @param kf: point to a Kalman_Info_TypeDef structure that
  *         contains the informations of kalman filter.
  * @note  xhat(k) = xhatminus(k) + K(k)·(z(k) - H·xhatminus(k))
  * @retval none
  */
static void Kalman_xhat_Update(Kalman_Info_TypeDef *kf)
{
  /* skip this step */
  if(kf->SkipStep4 == 1)
  {
    return;
  }

  /* calc_vector[0] = H xhatminus(k) */
  kf->mat.calc_vector[0].numRows = kf->mat.H.numRows;
  kf->mat.calc_vector[0].numCols = 1;
  kf->ErrorStatus = Matrix_Multiply(&kf->mat.H, &kf->mat.xhatminus, &kf->mat.calc_vector[0]);

  /* calc_vector[1] = z(k) - H·xhatminus(k) */
  kf->mat.calc_vector[1].numRows = kf->mat.z.numRows;
  kf->mat.calc_vector[1].numCols = 1;
  kf->ErrorStatus = Matrix_Subtract(&kf->mat.z, &kf->mat.calc_vector[0], &kf->mat.calc_vector[1]); 

  /* calc_vector[0] = K(k)·(z(k) - H·xhatminus(k)) */
  kf->mat.calc_vector[0].numRows = kf->mat.K.numRows;
  kf->mat.calc_vector[0].numCols = 1;
  kf->ErrorStatus = Matrix_Multiply(&kf->mat.K, &kf->mat.calc_vector[1], &kf->mat.calc_vector[0]);

  /* xhat(k) = xhatminus(k) + K(k)·(z(k) - H·xhatminus(k)) */
  kf->ErrorStatus = Matrix_Add(&kf->mat.xhatminus, &kf->mat.calc_vector[0], &kf->mat.xhat); 

  if(true == kf->Adaptive_NoiseCov.Adaptive_Enable)
  {
    /* r(k)=z(k)-xhat(k) */
    kf->ErrorStatus = Matrix_Subtract(&kf->mat.z,&kf->mat.xhat,&kf->Adaptive_NoiseCov.r);

    /* r(k)T */
    kf->ErrorStatus = Matrix_Transpose(&kf->Adaptive_NoiseCov.r,&kf->Adaptive_NoiseCov.temp_vector);

    /* temp_matrix[1] = r(k)·r(k)T */
    kf->Adaptive_NoiseCov.temp_matrix[1].numRows = kf->Adaptive_NoiseCov.r.numRows;
    kf->Adaptive_NoiseCov.temp_matrix[1].numCols = kf->Adaptive_NoiseCov.temp_vector.numCols;
    kf->ErrorStatus = Matrix_Multiply(&kf->Adaptive_NoiseCov.r,&kf->Adaptive_NoiseCov.temp_vector,&kf->Adaptive_NoiseCov.temp_matrix[1]);

    /* calc_matrix[1] = K(k)·r(k)·r(k)T */
    kf->mat.calc_matrix[1].numRows = kf->mat.K.numCols;
    kf->mat.calc_matrix[1].numCols = kf->Adaptive_NoiseCov.temp_matrix[1].numRows;
    kf->ErrorStatus = Matrix_Multiply(&kf->mat.K,&kf->Adaptive_NoiseCov.temp_matrix[1],&kf->mat.calc_matrix[1]);

    /* temp_matrix[1] = K(k)T */
    kf->Adaptive_NoiseCov.temp_matrix[1].numRows = kf->mat.K.numCols;
    kf->Adaptive_NoiseCov.temp_matrix[1].numCols = kf->mat.K.numRows;
    kf->ErrorStatus = Matrix_Transpose(&kf->mat.K,&kf->Adaptive_NoiseCov.temp_matrix[1]);
  
    /* calc_matrix[0] = K(k)·r(k)·r(k)T·K(k)T */
    kf->mat.calc_matrix[0].numRows = kf->mat.Q.numRows;
    kf->mat.calc_matrix[0].numCols = kf->mat.Q.numCols; 
    kf->ErrorStatus = Matrix_Multiply(&kf->mat.calc_matrix[1],&kf->Adaptive_NoiseCov.temp_matrix[1],&kf->mat.calc_matrix[0]);

    /* alpha·Q(k-1), (1-alpha)·(K(k)·r(k)·r(k)T·K(k)T) */
    for(uint8_t i=0;i < (kf->zSize*kf->zSize);i++)
    {
      kf->pdata.calc_matrix[1][i] = kf->Adaptive_NoiseCov.alpha*kf->pdata.Q[i];
      kf->pdata.calc_matrix[0][i] *= (1.f-kf->Adaptive_NoiseCov.alpha);
    }

    /* Q(k) = alpha·Q(k-1)+(1-alpha)·(K(k)·r(k)·r(k)T·K(k)T) */
    kf->ErrorStatus = Matrix_Add(&kf->mat.calc_matrix[1],&kf->mat.calc_matrix[0],&kf->mat.Q);

    if(fabsf(kf->Adaptive_NoiseCov.b-kf->Adaptive_NoiseCov.alpha) > 0.001f)
    {
      kf->Adaptive_NoiseCov.alpha = 1.f-(1-kf->Adaptive_NoiseCov.b)/(1-pow(kf->Adaptive_NoiseCov.b,(kf->Adaptive_NoiseCov.t+1)));
      kf->Adaptive_NoiseCov.t += kf->Adaptive_NoiseCov.dt;
    }
  }
}
//------------------------------------------------------------------------------
/**
  * @brief  Update the posteriori covariance
  * @param  kf: point to a Kalman_Info_TypeDef structure that
  *         contains the informations of kalman filter.
  * @note   P(k) = (I - K(k)·H)·Pminus(k)
  * @retval none
  */
static void Kalman_P_Update(Kalman_Info_TypeDef *kf)
{
  /* skip this step */
  if(kf->SkipStep5 == 1)
  {
    return;
  }

  /* calc_vector[0] = K(k)·H */
  kf->mat.calc_matrix[0].numRows = kf->mat.K.numRows;
  kf->mat.calc_matrix[0].numCols = kf->mat.H.numCols;
  kf->ErrorStatus = Matrix_Multiply(&kf->mat.K, &kf->mat.H, &kf->mat.calc_matrix[0]);

  /* calc_vector[1] = K(k)·H·Pminus(k) */
  kf->mat.calc_matrix[1].numRows = kf->mat.calc_matrix[0].numRows;
  kf->mat.calc_matrix[1].numCols = kf->mat.Pminus.numCols;
  kf->ErrorStatus = Matrix_Multiply(&kf->mat.calc_matrix[0], &kf->mat.Pminus, &kf->mat.calc_matrix[1]);
  
  /* P(k) = (I - K(k)·H)·Pminus(k) */
  kf->ErrorStatus = Matrix_Subtract(&kf->mat.Pminus, &kf->mat.calc_matrix[1], &kf->mat.P); 
}
//------------------------------------------------------------------------------

/**
  * @brief Update the Kalman Filter.
  * @param kf: point to a Kalman_Info_TypeDef structure that
  *         contains the informations of kalman filter.
  * @retval point of kalman filter output
  * @note kalman filter:
  *       1.xhatminus(k) = A·xhat(k-1) + B·u(k)
  *       2.Pminus(k) = A·P(k-1)·AT + Q
  *       3.K(k) = Pminus(k)·HT/(H·Pminus(k)·HT + R)
  *       4.xhat(k) = xhatminus(k) + K(k)·(z(k) - H·xhatminus(k))
  *       5.P(k) = (I - K(k)·H)·Pminus(k)
  */
float *Kalman_Filter_Update(Kalman_Info_TypeDef *kf)
{
  /* Update the input */
  Kalman_Input_Update(kf);

  /* User Function 0 */
  if(kf->User_Function0 != NULL)
  {
    kf->User_Function0(kf);
  }

  /* Update the priori state estimate */
  Kalman_xhatminus_Update(kf);
  /* User Function 1 */
  if(kf->User_Function1 != NULL)
  {
    kf->User_Function1(kf);
  }

  /* Update the priori covariance */
  Kalman_Pminus_Update(kf);
  /* User Function 2 */
  if(kf->User_Function2 != NULL)
  {
    kf->User_Function2(kf);
  }

  /* Update the kalman gain */
  Kalman_K_Update(kf);
  /* User Function 3 */
  if(kf->User_Function3 != NULL)
  {
    kf->User_Function3(kf);
  }

  /* Update the posteriori state estimate */
  Kalman_xhat_Update(kf);
  /* User Function 4 */
  if(kf->User_Function4 != NULL)
  {
    kf->User_Function4(kf);
  }

  /* Update the posteriori covariance */
  Kalman_P_Update(kf);
  /* User Function 5 */
  if(kf->User_Function5 != NULL)
  {
    kf->User_Function5(kf);
  }

  /* User Function 6 */
  if(kf->User_Function6 != NULL)
  {
    kf->User_Function6(kf);
  }

  /* store the output */
  memcpy(kf->Output, kf->pdata.xhat, kf->sizeof_float * kf->xhatSize);

  return kf->Output;
}
//------------------------------------------------------------------------------

