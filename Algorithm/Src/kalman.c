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
  * @note kalman filter:
  *       1.xhatminus(k) = A·xhat(k-1) + B·u(k)
  *       2.Pminus(k) = A·P(k-1)·AT + Q
  *       3.K(k) = Pminus(k)·HT/(H·Pminus(k)·HT + R)
  *       4.xhat(k) = xhatminus(k) + K(k)·(z(k) - H·xhatminus(k))
  *       5.P(k) = (I - K(k)·H)·Pminus(k)
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
  
  /* Initialize the measurement Input */
  kf->MeasureInput = (float *)user_malloc(kf->sizeof_float * zSize);
  memset(kf->MeasureInput, 0, kf->sizeof_float * zSize);
  
  /* Initialize the posteriori state estimate */
  kf->pdata.xhat = (float *)user_malloc(kf->sizeof_float * xhatSize);
  memset(kf->pdata.xhat, 0, kf->sizeof_float * xhatSize);
  Matrix_Init(&kf->matrix.xhat, kf->xhatSize, 1, (float *)kf->pdata.xhat);
  
  /* Initialize the priori state estimate */
  kf->pdata.xhatminus = (float *)user_malloc(kf->sizeof_float * xhatSize);
  memset(kf->pdata.xhatminus, 0, kf->sizeof_float * xhatSize);
  Matrix_Init(&kf->matrix.xhatminus, kf->xhatSize, 1, (float *)kf->pdata.xhatminus);
  
  /* Initialize the measurement */
  kf->pdata.z = (float *)user_malloc(kf->sizeof_float * zSize);
  memset(kf->pdata.z, 0, kf->sizeof_float * zSize);
  Matrix_Init(&kf->matrix.z, kf->zSize, 1, (float *)kf->pdata.z);
  
  if (kf->uSize != 0)
  {
    /* Initialize the control input */
    kf->ControlInput = (float *)user_malloc(kf->sizeof_float * uSize);
    memset(kf->ControlInput, 0, kf->sizeof_float * uSize);

    /* Initialize the control-input */ 
    kf->pdata.u = (float *)user_malloc(kf->sizeof_float * uSize);
    memset(kf->pdata.u, 0, kf->sizeof_float * uSize);
    Matrix_Init(&kf->matrix.u, kf->uSize, 1, (float *)kf->pdata.u);

    /* Initialize the input-state matrix */  
    kf->pdata.B = (float *)user_malloc(kf->sizeof_float * xhatSize * uSize);
    memset(kf->pdata.B, 0, kf->sizeof_float * xhatSize * uSize);
    Matrix_Init(&kf->matrix.B, kf->xhatSize, kf->uSize, (float *)kf->pdata.B);
  }
  
  /* Initialize the state transition matrix */ 
  kf->pdata.A = (float *)user_malloc(kf->sizeof_float * xhatSize * xhatSize);
  memset(kf->pdata.A, 0, kf->sizeof_float * xhatSize * xhatSize);
  Matrix_Init(&kf->matrix.A, kf->xhatSize, kf->xhatSize, (float *)kf->pdata.A);
  
  kf->pdata.AT = (float *)user_malloc(kf->sizeof_float * xhatSize * xhatSize);
  memset(kf->pdata.AT, 0, kf->sizeof_float * xhatSize * xhatSize);
  Matrix_Init(&kf->matrix.AT, kf->xhatSize, kf->xhatSize, (float *)kf->pdata.AT);
  
  /* Initialize the state-measurement matrix */ 
  kf->pdata.H = (float *)user_malloc(kf->sizeof_float * zSize * xhatSize);
  memset(kf->pdata.H, 0, kf->sizeof_float * zSize * xhatSize);
  Matrix_Init(&kf->matrix.H, kf->zSize, kf->xhatSize, (float *)kf->pdata.H);
  
  kf->pdata.HT = (float *)user_malloc(kf->sizeof_float * xhatSize * zSize);
  memset(kf->pdata.HT, 0, kf->sizeof_float * xhatSize * zSize);
  Matrix_Init(&kf->matrix.HT, kf->xhatSize, kf->zSize, (float *)kf->pdata.HT);
  
  /* Initialize the posteriori covariance matrix */
  kf->pdata.P = (float *)user_malloc(kf->sizeof_float * xhatSize * xhatSize);
  memset(kf->pdata.P, 0, kf->sizeof_float * xhatSize * xhatSize);
  Matrix_Init(&kf->matrix.P, kf->xhatSize, kf->xhatSize, (float *)kf->pdata.P);
  
  /* Initialize the priori covariance matrix */
  kf->pdata.Pminus = (float *)user_malloc(kf->sizeof_float * xhatSize * xhatSize);
  memset(kf->pdata.Pminus, 0, kf->sizeof_float * xhatSize * xhatSize);
  Matrix_Init(&kf->matrix.Pminus, kf->xhatSize, kf->xhatSize, (float *)kf->pdata.Pminus);
  
  /* Initialize the process noise covariance matrix */  
  kf->pdata.Q = (float *)user_malloc(kf->sizeof_float * xhatSize * xhatSize);
  memset(kf->pdata.Q, 0, kf->sizeof_float * xhatSize * xhatSize);
  Matrix_Init(&kf->matrix.Q, kf->xhatSize, kf->xhatSize, (float *)kf->pdata.Q);
  
  /* Initialize the measurement noise covariance matrix */
  kf->pdata.R = (float *)user_malloc(kf->sizeof_float * zSize * zSize);
  memset(kf->pdata.R, 0, kf->sizeof_float * zSize * zSize);
  Matrix_Init(&kf->matrix.R, kf->zSize, kf->zSize, (float *)kf->pdata.R);
  
  /* Initialize the kalman gain matrix */  
  kf->pdata.K = (float *)user_malloc(kf->sizeof_float * xhatSize * zSize);
  memset(kf->pdata.K, 0, kf->sizeof_float * xhatSize * zSize);
  Matrix_Init(&kf->matrix.K, kf->xhatSize, kf->zSize, (float *)kf->pdata.K);
  
  /* Initialize the K_d matrix (K_d = H Pminus HT + R) */  
  kf->pdata.K_d = (float *)user_malloc(kf->sizeof_float * kf->xhatSize * kf->xhatSize);
  memset(kf->pdata.K_d, 0, kf->sizeof_float * kf->xhatSize * kf->xhatSize);
  Matrix_Init(&kf->matrix.K_d, kf->xhatSize, kf->xhatSize, (float *)kf->pdata.K_d);
  
  /* Initialize the calculate process matrix */
  kf->pdata.calc_matrix[0] = (float *)user_malloc(kf->sizeof_float * kf->xhatSize * kf->xhatSize);
  memset(kf->pdata.calc_matrix[0],0,kf->sizeof_float * kf->xhatSize * kf->xhatSize);
  Matrix_Init(&kf->matrix.calc_matrix[0], kf->xhatSize, kf->xhatSize, (float *)kf->pdata.calc_matrix[0]);
  
  kf->pdata.calc_matrix[1] = (float *)user_malloc(kf->sizeof_float * kf->xhatSize * kf->xhatSize);
  memset(kf->pdata.calc_matrix[1],0,kf->sizeof_float * kf->xhatSize * kf->xhatSize);
  Matrix_Init(&kf->matrix.calc_matrix[1], kf->xhatSize, kf->xhatSize, (float *)kf->pdata.calc_matrix[1]);
  
  /* Initialize the calculate process vector */
  kf->pdata.calc_vector[0] = (float *)user_malloc(kf->sizeof_float * kf->xhatSize);
  memset(kf->pdata.calc_vector[0],0,kf->sizeof_float * kf->xhatSize);
  Matrix_Init(&kf->matrix.calc_vector[0], kf->xhatSize, 1, (float *)kf->pdata.calc_vector[0]);
  
  kf->pdata.calc_vector[1] = (float *)user_malloc(kf->sizeof_float * kf->xhatSize);
  memset(kf->pdata.calc_vector[1],0,kf->sizeof_float * kf->xhatSize);
  Matrix_Init(&kf->matrix.calc_vector[1], kf->xhatSize, 1, (float *)kf->pdata.calc_vector[1]);
  
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
        kf->matrix.calc_vector[0].numRows = kf->xhatSize;
        kf->matrix.calc_vector[0].numCols = 1;
        kf->ErrorStatus = Matrix_Multiply(&kf->matrix.A, &kf->matrix.xhat, &kf->matrix.calc_vector[0]);   

        /* calc_vector[1] = B·u(k) */ 
        kf->matrix.calc_vector[0].numRows = kf->xhatSize;
        kf->matrix.calc_vector[0].numCols = 1;
        kf->ErrorStatus = Matrix_Multiply(&kf->matrix.B, &kf->matrix.u, &kf->matrix.calc_vector[1]);    

        /* xhatminus(k) = A·xhat(k-1) + B·u(k) */
        kf->ErrorStatus = Matrix_Add(&kf->matrix.calc_vector[0], &kf->matrix.calc_vector[1], &kf->matrix.xhatminus);   
    }
    else
    {
        /* xhatminus(k) = A·xhat(k-1) */
        kf->ErrorStatus = Matrix_Multiply(&kf->matrix.A, &kf->matrix.xhat, &kf->matrix.xhatminus);   
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
    kf->ErrorStatus = Matrix_Transpose(&kf->matrix.A, &kf->matrix.AT); 

    /* Pminus = A·P(k-1) */ 
    kf->ErrorStatus = Matrix_Multiply(&kf->matrix.A, &kf->matrix.P, &kf->matrix.Pminus); 

    /* calc_matrix[0] = A·P(k-1)·AT */ 
    kf->matrix.calc_matrix[0].numRows = kf->matrix.Pminus.numRows;
    kf->matrix.calc_matrix[0].numCols = kf->matrix.AT.numCols;
    kf->ErrorStatus = Matrix_Multiply(&kf->matrix.Pminus, &kf->matrix.AT, &kf->matrix.calc_matrix[0]); 

    /* Pminus(k) = A·P(k-1)·AT + Q */
    kf->ErrorStatus = Matrix_Add(&kf->matrix.calc_matrix[0], &kf->matrix.Q, &kf->matrix.Pminus);  
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
    kf->ErrorStatus = Matrix_Transpose(&kf->matrix.H, &kf->matrix.HT); 

    /* calc_matrix[0] = H·Pminus(k) */
    kf->matrix.calc_matrix[0].numRows = kf->matrix.H.numRows;
    kf->matrix.calc_matrix[0].numCols = kf->matrix.Pminus.numCols;
    kf->ErrorStatus = Matrix_Multiply(&kf->matrix.H, &kf->matrix.Pminus, &kf->matrix.calc_matrix[0]); 

    /* calc_matrix[1] = H·Pminus(k)·HT */
    kf->matrix.calc_matrix[1].numRows = kf->matrix.calc_matrix[0].numRows;
    kf->matrix.calc_matrix[1].numCols = kf->matrix.HT.numCols;
    kf->ErrorStatus = Matrix_Multiply(&kf->matrix.calc_matrix[0], &kf->matrix.HT, &kf->matrix.calc_matrix[1]);  

    /* K_d = H·Pminus(k)·HT + R */
    kf->matrix.K_d.numRows = kf->matrix.R.numRows;
    kf->matrix.K_d.numCols = kf->matrix.R.numCols;
    kf->ErrorStatus = Matrix_Add(&kf->matrix.calc_matrix[1], &kf->matrix.R, &kf->matrix.K_d); 

    /* calc_matrix[1] = inverse(H·Pminus(k)·HT + R) */
    kf->ErrorStatus = Matrix_Inverse(&kf->matrix.K_d, &kf->matrix.calc_matrix[1]);

    /* calc_matrix[0] = Pminus(k)·HT */
    kf->matrix.calc_matrix[0].numRows = kf->matrix.Pminus.numRows;
    kf->matrix.calc_matrix[0].numCols = kf->matrix.HT.numCols;
    kf->ErrorStatus = Matrix_Multiply(&kf->matrix.Pminus, &kf->matrix.HT, &kf->matrix.calc_matrix[0]);

    /* K(k) = Pminus(k)·HT / (H·Pminus(k)·HT + R) */
    kf->ErrorStatus = Matrix_Multiply(&kf->matrix.calc_matrix[0], &kf->matrix.calc_matrix[1], &kf->matrix.K);
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
    kf->matrix.calc_vector[0].numRows = kf->matrix.H.numRows;
    kf->matrix.calc_vector[0].numCols = 1;
    kf->ErrorStatus = Matrix_Multiply(&kf->matrix.H, &kf->matrix.xhatminus, &kf->matrix.calc_vector[0]);

    /* calc_vector[1] = z(k) - H·xhatminus(k) */
    kf->matrix.calc_vector[1].numRows = kf->matrix.z.numRows;
    kf->matrix.calc_vector[1].numCols = 1;
    kf->ErrorStatus = Matrix_Subtract(&kf->matrix.z, &kf->matrix.calc_vector[0], &kf->matrix.calc_vector[1]); 

    /* calc_vector[0] = K(k)·(z(k) - H·xhatminus(k)) */
    kf->matrix.calc_vector[0].numRows = kf->matrix.K.numRows;
    kf->matrix.calc_vector[0].numCols = 1;
    kf->ErrorStatus = Matrix_Multiply(&kf->matrix.K, &kf->matrix.calc_vector[1], &kf->matrix.calc_vector[0]);

    /* xhat(k) = xhatminus(k) + K(k)·(z(k) - H·xhatminus(k)) */
    kf->ErrorStatus = Matrix_Add(&kf->matrix.xhatminus, &kf->matrix.calc_vector[0], &kf->matrix.xhat); 
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
    kf->matrix.calc_matrix[0].numRows = kf->matrix.K.numRows;
    kf->matrix.calc_matrix[0].numCols = kf->matrix.H.numCols;
    kf->ErrorStatus = Matrix_Multiply(&kf->matrix.K, &kf->matrix.H, &kf->matrix.calc_matrix[0]);

    /* calc_vector[1] = K(k)·H·Pminus(k) */
    kf->matrix.calc_matrix[1].numRows = kf->matrix.calc_matrix[0].numRows;
    kf->matrix.calc_matrix[1].numCols = kf->matrix.Pminus.numCols;
    kf->ErrorStatus = Matrix_Multiply(&kf->matrix.calc_matrix[0], &kf->matrix.Pminus, &kf->matrix.calc_matrix[1]);
		
    /* P(k) = (I - K(k)·H)·Pminus(k) */
    kf->ErrorStatus = Matrix_Subtract(&kf->matrix.Pminus, &kf->matrix.calc_matrix[1], &kf->matrix.P); 
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

