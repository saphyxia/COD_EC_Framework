/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : kalman.c
  * @brief          : Kalman filter 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : To be perfected
  * @note kalman filter formula:
  *       1.xhatminus = A xhat(k-1) + B u(k-1)
  *       2.Pminus = A P(k-1) AT + Q
  *       3.K = H·Pminus / (H·Pminus·HT + R)
  *       4.xhat = xhatminus + K(k)·(z(k) - H·xhatminus)
  *       5.P = (I - K(k)·H)·Pminus
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "kalman.h"
#include "math.h"
#include "string.h"

uint16_t sizeof_float, sizeof_double;

/**
  * @brief Initializes the kalman filter according to the specified parameters in the
  *         Kalman_Filter_Info_TypeDef.
  * @param kf: pointer to a Kalman_Filter_Info_TypeDef structure that
  *         contains the information  for the kalman filter.
  * @param xhatSize: state vector dimension
  * @param uSize: control vector dimension
  * @param zSize: measurement vector dimension
  * @retval none
  */
arm_status Kalman_Filter_Init(Kalman_Filter_Info_TypeDef *kf,uint8_t xhatSize,uint8_t uSize,uint8_t zSize)
{
    /* Update the size of float/double */
    sizeof_float = sizeof(float);
    sizeof_double = sizeof(double);
    
    /* Initializes the state vector dimension */
    kf->xhatSize = xhatSize;

     /* Initializes the control vector dimension */
    kf->uSize = uSize;

    /**< Initializes the measurement vector dimension */      
    kf->zSize = zSize;          

    /* judge the length of vector */
    if(kf->xhatSize == 0 || kf->zSize == 0)
    {
        return ARM_MATH_LENGTH_ERROR;  
    }

    /*Initializes the matrix of kalman filter ----------------------------------*/

    /* Initializes the external measure vector */
    kf->MeasuredVector = (float *)user_malloc(sizeof_float * zSize);
    memset(kf->MeasuredVector, 0, sizeof_float * zSize);

    /* Initializes the external control vector */
    kf->ControlVector = (float *)user_malloc(sizeof_float * uSize);
    memset(kf->ControlVector, 0, sizeof_float * uSize);

    /* Initializes the xhat */
    kf->Memory_t.xhat = (float *)user_malloc(sizeof_float * xhatSize);
    memset(kf->Memory_t.xhat, 0, sizeof_float * xhatSize);
    Matrix_Init(&kf->Mat_t.xhat, kf->xhatSize, 1, (float *)kf->Memory_t.xhat);

    /* Initializes the xhatminus */
    kf->Memory_t.xhatminus = (float *)user_malloc(sizeof_float * xhatSize);
    memset(kf->Memory_t.xhatminus, 0, sizeof_float * xhatSize);
    Matrix_Init(&kf->Mat_t.xhatminus, kf->xhatSize, 1, (float *)kf->Memory_t.xhatminus);

    /* Initializes the measurement vector */
    kf->Memory_t.z = (float *)user_malloc(sizeof_float * zSize);
    memset(kf->Memory_t.z, 0, sizeof_float * zSize);
    Matrix_Init(&kf->Mat_t.z, kf->zSize, 1, (float *)kf->Memory_t.z);

    if (uSize != 0)
    {
        /* Initializes the control vector */ 
        kf->Memory_t.u = (float *)user_malloc(sizeof_float * uSize);
        memset(kf->Memory_t.u, 0, sizeof_float * uSize);
        Matrix_Init(&kf->Mat_t.u, kf->uSize, 1, (float *)kf->Memory_t.u);

        /* Initializes the control Mat_trix */  
        kf->Memory_t.B = (float *)user_malloc(sizeof_float * xhatSize * uSize);
        memset(kf->Memory_t.B, 0, sizeof_float * xhatSize * uSize);
        Matrix_Init(&kf->Mat_t.B, kf->xhatSize, kf->uSize, (float *)kf->Memory_t.B);
    }

    /* Initializes the state transition Mat_trix */ 
    kf->Memory_t.A = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
    memset(kf->Memory_t.A, 0, sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->Mat_t.A, kf->xhatSize, kf->xhatSize, (float *)kf->Memory_t.A);

    kf->Memory_t.AT = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
    memset(kf->Memory_t.AT, 0, sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->Mat_t.AT, kf->xhatSize, kf->xhatSize, (float *)kf->Memory_t.AT);

    /* Initializes the measurement Mat_trix */ 
    kf->Memory_t.H = (float *)user_malloc(sizeof_float * zSize * xhatSize);
    memset(kf->Memory_t.H, 0, sizeof_float * zSize * xhatSize);
    Matrix_Init(&kf->Mat_t.H, kf->zSize, kf->xhatSize, (float *)kf->Memory_t.H);

    kf->Memory_t.HT = (float *)user_malloc(sizeof_float * xhatSize * zSize);
    memset(kf->Memory_t.HT, 0, sizeof_float * xhatSize * zSize);
    Matrix_Init(&kf->Mat_t.HT, kf->xhatSize, kf->zSize, (float *)kf->Memory_t.HT);

    /* Initializes the posteriori error covariance Mat_trix */
    kf->Memory_t.P = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
    memset(kf->Memory_t.P, 0, sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->Mat_t.P, kf->xhatSize, kf->xhatSize, (float *)kf->Memory_t.P);

    /* Initializes the priori error covariance Mat_trix */
    kf->Memory_t.Pminus = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
    memset(kf->Memory_t.Pminus, 0, sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->Mat_t.Pminus, kf->xhatSize, kf->xhatSize, (float *)kf->Memory_t.Pminus);

    /* Initializes the process noise covariance Mat_trix */  
    kf->Memory_t.Q = (float *)user_malloc(sizeof_float * xhatSize * xhatSize);
    memset(kf->Memory_t.Q, 0, sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->Mat_t.Q, kf->xhatSize, kf->xhatSize, (float *)kf->Memory_t.Q);

    /* Initializes the measurement noise covariance Mat_trix */
    kf->Memory_t.R = (float *)user_malloc(sizeof_float * zSize * zSize);
    memset(kf->Memory_t.R, 0, sizeof_float * zSize * zSize);
    Matrix_Init(&kf->Mat_t.R, kf->zSize, kf->zSize, (float *)kf->Memory_t.R);

    /* Initializes the kalman gain */  
    kf->Memory_t.K = (float *)user_malloc(sizeof_float * xhatSize * zSize);
    memset(kf->Memory_t.K, 0, sizeof_float * xhatSize * zSize);
    Matrix_Init(&kf->Mat_t.K, kf->xhatSize, kf->zSize, (float *)kf->Memory_t.K);

    /* Initializes the K_denominator (K_denominator = H Pminus HT + R) */  
    kf->Memory_t.K_denominator = (float *)user_malloc(sizeof_float * kf->xhatSize * kf->xhatSize);
    memset(kf->Memory_t.K_denominator, 0, sizeof_float * kf->xhatSize * kf->xhatSize);
    Matrix_Init(&kf->Mat_t.K_denominator, kf->xhatSize, kf->xhatSize, (float *)kf->Memory_t.K_denominator);

    /* Initializes the calculate cache Mat_trix */
    kf->Memory_t.cache_matrix[0] = (float *)user_malloc(sizeof_float * kf->xhatSize * kf->xhatSize);
    memset(kf->Memory_t.cache_matrix[0],0,sizeof_float * kf->xhatSize * kf->xhatSize);
    Matrix_Init(&kf->Mat_t.cache_matrix[0], kf->xhatSize, kf->xhatSize, (float *)kf->Memory_t.cache_matrix[0]);

    kf->Memory_t.cache_matrix[1] = (float *)user_malloc(sizeof_float * kf->xhatSize * kf->xhatSize);
    memset(kf->Memory_t.cache_matrix[1],0,sizeof_float * kf->xhatSize * kf->xhatSize);
    Matrix_Init(&kf->Mat_t.cache_matrix[1], kf->xhatSize, kf->xhatSize, (float *)kf->Memory_t.cache_matrix[1]);

    /* Initializes the calculate cache vector */
    kf->Memory_t.cache_vector[0] = (float *)user_malloc(sizeof_float * kf->xhatSize);
    memset(kf->Memory_t.cache_vector[0],0,sizeof_float * kf->xhatSize);
    Matrix_Init(&kf->Mat_t.cache_vector[0], kf->xhatSize, 1, (float *)kf->Memory_t.cache_vector[0]);

    kf->Memory_t.cache_vector[1] = (float *)user_malloc(sizeof_float * kf->xhatSize);
    memset(kf->Memory_t.cache_vector[1],0,sizeof_float * kf->xhatSize);
    Matrix_Init(&kf->Mat_t.cache_vector[1], kf->xhatSize, 1, (float *)kf->Memory_t.cache_vector[1]);

    /* Initializes the filter output */
    kf->Output = (float *)user_malloc(sizeof_float * xhatSize);
    memset(kf->Output, 0, sizeof_float * xhatSize);

    return ARM_MATH_SUCCESS;
}

/**
  * @brief Update the Measuerment Information
  * @param kf: pointer to a Kalman_Filter_Info_TypeDef structure that
  *         contains the information  for the kalman filter.
  * @retval none
  */
static void Kalman_Filter_Measurement_Update(Kalman_Filter_Info_TypeDef *kf)
{
    /* update the measuerment vector from the external measuerment vector */
    memcpy(kf->Memory_t.z, kf->MeasuredVector, sizeof_float * kf->zSize);

    /* clear the external measuerment vector */
    memset(kf->MeasuredVector, 0, sizeof_float * kf->zSize);

    /* update the control vector from the external control vector */
    memcpy(kf->Memory_t.u, kf->ControlVector, sizeof_float * kf->uSize);
}

/**
  * @brief Update the Priori EstiMate
  * @param kf: pointer to a Kalman_Filter_Info_TypeDef structure that
  *         contains the information  for the kalman filter.
  * @note xhatminus = A xhat(k-1) + B u(k-1)
  * @retval none
  */
static void Kalman_Filter_xhatminus_Update(Kalman_Filter_Info_TypeDef *kf)
{
    if (kf->uSize > 0)
    {
        /* cache_vector[0] = A xhat(k-1) */ 
        kf->Mat_t.cache_vector[0].numRows = kf->xhatSize;
        kf->Mat_t.cache_vector[0].numCols = 1;
        kf->MatStatus = Matrix_Multiply(&kf->Mat_t.A, &kf->Mat_t.xhat, &kf->Mat_t.cache_vector[0]);   

        /* cache_vector[1] = B u(k-1) */ 
        kf->Mat_t.cache_vector[0].numRows = kf->xhatSize;
        kf->Mat_t.cache_vector[0].numCols = 1;
        kf->MatStatus = Matrix_Multiply(&kf->Mat_t.B, &kf->Mat_t.u, &kf->Mat_t.cache_vector[1]);    

        /* xhatminus = A xhat(k-1) + B u(k-1) */
        kf->MatStatus = Matrix_Add(&kf->Mat_t.cache_vector[0], &kf->Mat_t.cache_vector[1], &kf->Mat_t.xhatminus);   
    }
    /* lack of control vector */
    else
    {
        /* xhatminus = A xhat(k-1) */
        kf->MatStatus = Matrix_Multiply(&kf->Mat_t.A, &kf->Mat_t.xhat, &kf->Mat_t.xhatminus);   
    }
}

/**
  * @brief Update the Priori Error Covariance Matrix
  * @param kf: pointer to a Kalman_Filter_Info_TypeDef structure that
  *         contains the information  for the kalman filter.
  * @note Pminus = A P(k-1) AT + Q
  * @retval none
  */
static void Kalman_Filter_Pminus_Update(Kalman_Filter_Info_TypeDef *kf)
{
    /* AT */
    kf->MatStatus = Matrix_Transpose(&kf->Mat_t.A, &kf->Mat_t.AT); 

    /* Pminus = A P(k-1) */ 
    kf->MatStatus = Matrix_Multiply(&kf->Mat_t.A, &kf->Mat_t.P, &kf->Mat_t.Pminus); 

    /* cache_matrix[0] = A P(k-1) AT */ 
    kf->Mat_t.cache_matrix[0].numRows = kf->Mat_t.Pminus.numRows;
    kf->Mat_t.cache_matrix[0].numCols = kf->Mat_t.AT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->Mat_t.Pminus, &kf->Mat_t.AT, &kf->Mat_t.cache_matrix[0]); 

    /* Pminus = A P(k-1) AT + Q */
    kf->MatStatus = Matrix_Add(&kf->Mat_t.cache_matrix[0], &kf->Mat_t.Q, &kf->Mat_t.Pminus);  
}

/**
  * @brief Update the Kalman Gain
  * @param kf: pointer to a Kalman_Filter_Info_TypeDef structure that
  *         contains the information  for the kalman filter.
  * @note K = H·Pminus / (H·Pminus·HT + R)
  * @retval none
  */
static void Kalman_Filter_K_Update(Kalman_Filter_Info_TypeDef *kf)
{
    /* HT */
    kf->MatStatus = Matrix_Transpose(&kf->Mat_t.H, &kf->Mat_t.HT); 

    /* cache_matrix[0] = H·Pminus */
    kf->Mat_t.cache_matrix[0].numRows = kf->Mat_t.H.numRows;
    kf->Mat_t.cache_matrix[0].numCols = kf->Mat_t.Pminus.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->Mat_t.H, &kf->Mat_t.Pminus, &kf->Mat_t.cache_matrix[0]); 

    /* cache_matrix[1] = H·Pminus·HT */
    kf->Mat_t.cache_matrix[1].numRows = kf->Mat_t.cache_matrix[0].numRows;
    kf->Mat_t.cache_matrix[1].numCols = kf->Mat_t.HT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->Mat_t.cache_matrix[0], &kf->Mat_t.HT, &kf->Mat_t.cache_matrix[1]);  

    /* K_denominator = H·Pminus·HT + R */
    kf->Mat_t.K_denominator.numRows = kf->Mat_t.R.numRows;
    kf->Mat_t.K_denominator.numCols = kf->Mat_t.R.numCols;
    kf->MatStatus = Matrix_Add(&kf->Mat_t.cache_matrix[1], &kf->Mat_t.R, &kf->Mat_t.K_denominator); 

    /* cache_matrix[1] = inverse(H·Pminus·HT + R) */
    kf->MatStatus = Matrix_Inverse(&kf->Mat_t.K_denominator, &kf->Mat_t.cache_matrix[1]);

    /* cache_matrix[0] = Pminus·HT */
    kf->Mat_t.cache_matrix[0].numRows = kf->Mat_t.Pminus.numRows;
    kf->Mat_t.cache_matrix[0].numCols = kf->Mat_t.HT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->Mat_t.Pminus, &kf->Mat_t.HT, &kf->Mat_t.cache_matrix[0]);

    /* K = H·Pminus / (H·Pminus·HT + R) */
    kf->MatStatus = Matrix_Multiply(&kf->Mat_t.cache_matrix[0], &kf->Mat_t.cache_matrix[1], &kf->Mat_t.K);
}

/**
  * @brief Update the Posteriori EstiMate
  * @param kf: pointer to a Kalman_Filter_Info_TypeDef structure that
  *         contains the information  for the kalman filter.
  * @note xhat = xhatminus + K(k)·(z(k) - H·xhatminus)
  * @retval none
  */
static void Kalman_Filter_xhat_Update(Kalman_Filter_Info_TypeDef *kf)
{
    /* cache_vector[0] = H xhatminus */
    kf->Mat_t.cache_vector[0].numRows = kf->Mat_t.H.numRows;
    kf->Mat_t.cache_vector[0].numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->Mat_t.H, &kf->Mat_t.xhatminus, &kf->Mat_t.cache_vector[0]);

    /* cache_vector[1] = z(k) - H·xhatminus */
    kf->Mat_t.cache_vector[1].numRows = kf->Mat_t.z.numRows;
    kf->Mat_t.cache_vector[1].numCols = 1;
    kf->MatStatus = Matrix_Subtract(&kf->Mat_t.z, &kf->Mat_t.cache_vector[0], &kf->Mat_t.cache_vector[1]); 

    /* cache_vector[0] = K(k)·(z(k) - H·xhatminus) */
    kf->Mat_t.cache_vector[0].numRows = kf->Mat_t.K.numRows;
    kf->Mat_t.cache_vector[0].numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->Mat_t.K, &kf->Mat_t.cache_vector[1], &kf->Mat_t.cache_vector[0]);

    /* xhat = xhatminus + K(k)·(z(k) - H·xhatminus) */
    kf->MatStatus = Matrix_Add(&kf->Mat_t.xhatminus, &kf->Mat_t.cache_vector[0], &kf->Mat_t.xhat); 
}
/**
  * @brief Update the Posteriori Error Covariance Matrix
  * @param kf: pointer to a Kalman_Filter_Info_TypeDef structure that
  *         contains the information  for the kalman filter.
  * @note P = (I - K(k)·H)·Pminus
  * @retval none
  */
static void Kalman_Filter_P_Update(Kalman_Filter_Info_TypeDef *kf)
{
    /* cache_vector[0] = K(k)·H */
    kf->Mat_t.cache_vector[0].numRows = kf->Mat_t.K.numRows;
    kf->Mat_t.cache_vector[0].numCols = kf->Mat_t.H.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->Mat_t.K, &kf->Mat_t.H, &kf->Mat_t.cache_vector[0]);

    /* cache_vector[1] = K(k)·H·Pminus */
    kf->Mat_t.cache_vector[1].numRows = kf->Mat_t.cache_vector[0].numRows;
    kf->Mat_t.cache_vector[1].numCols = kf->Mat_t.Pminus.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->Mat_t.cache_vector[0], &kf->Mat_t.Pminus, &kf->Mat_t.cache_vector[1]);

    /* P = (I - K(k)·H)·Pminus */
    kf->MatStatus = Matrix_Subtract(&kf->Mat_t.Pminus, &kf->Mat_t.cache_vector[1], &kf->Mat_t.P); 
}

/**
  * @brief Update the Kalman Filter according to the specified parameters in the
  *         Kalman_Filter_Info_TypeDef.
  * @param kf: pointer to a Kalman_Filter_Info_TypeDef structure that
  *         contains the information  for the kalman filter.
  * @retval pointer of kalman filter output
  * @note kalman filter formula:
  *       1.xhatminus = A xhat(k-1) + B u(k-1)
  *       2.Pminus = A P(k-1) AT + Q
  *       3.K = H·Pminus / (H·Pminus·HT + R)
  *       4.xhat = xhatminus + K(k)·(z(k) - H·xhatminus)
  *       5.P = (I - K(k)·H)·Pminus
  */
float *Kalman_Filter_Update(Kalman_Filter_Info_TypeDef *kf)
{
    /* Update the Measuerment Information */
    Kalman_Filter_Measurement_Update(kf);

    /* Update the Priori EstiMate */
    Kalman_Filter_xhatminus_Update(kf);

    /* Update the Priori Error Covariance Matrix */
    Kalman_Filter_Pminus_Update(kf);

    /* Update the kalman filter */
    Kalman_Filter_K_Update(kf);

    /* Update the Posteriori EstiMate */
    Kalman_Filter_xhat_Update(kf);

    /* Update the Posteriori Error Covariance Matrix */
    Kalman_Filter_P_Update(kf);

    /* Update the kalman filter output */
    memcpy(kf->Output, kf->Memory_t.xhat, sizeof_float * kf->xhatSize);

    return kf->Output;
}


