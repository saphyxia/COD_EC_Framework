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
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "arm_math.h"

/* Exported defines -----------------------------------------------------------*/
/**
 * @brief macro definition of the user_malloc that returns a pointer to the memory of the allocated size.
 */
#ifndef user_malloc
#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif
#endif

/**
 * @brief macro definition of the matrix calculation.
 */
#define matrix             arm_matrix_instance_f32
#define matrix_64          arm_matrix_instance_f64
#define Matrix_Init        arm_mat_init_f32
#define Matrix_Add         arm_mat_add_f32
#define Matrix_Subtract    arm_mat_sub_f32
#define Matrix_Multiply    arm_mat_mult_f32
#define Matrix_Transpose   arm_mat_trans_f32
#define Matrix_Inverse     arm_mat_inverse_f32
#define Matrix_Inverse_64  arm_mat_inverse_f64

/* Exported types ------------------------------------------------------------*/
/**
 * @brief typedef structure that contains the information  for the kalman filter.
 */
typedef struct 
{
  uint8_t xhatSize;   /*!< state vector dimension */
  uint8_t uSize;      /*!< control vector dimension */
  uint8_t zSize;      /*!< measurement vector dimension */

  float *MeasuredVector; /*!< external measure vector pointer */
  float *ControlVector;  /*!< external control vector pointer */

  /**
   * @brief Instance structure for the floating-point matrix structure.
   */
  struct 
  {
    matrix xhat;              /*!< posteriori estimate matrix */
    matrix xhatminus;         /*!< priori estimate matrix */
    matrix u;                 /*!< control vector */
    matrix z;                 /*!< measurement vector */
    matrix B;                 /*!< control matrix */ 
    matrix A,AT;              /*!< state transition matrix */
    matrix H,HT;              /*!< measurement matrix */
    matrix P;                 /*!< posteriori error covariance matrix */
    matrix Pminus;            /*!< priori error covariance matrix */
    matrix Q;                 /*!< process noise covariance matrix */ 
    matrix R;                 /*!< measurement noise covariance matrix */ 
    matrix K;                 /*!< kalman gain matrix */
    matrix K_denominator;     /*!< K_denominator matrix (K_denominator = H Pminus HT + R) */
    matrix cache_matrix[2];   /*!< calculate cache matrix */
    matrix cache_vector[2];   /*!< calculate cache vector */
  }Mat_t;

  arm_status MatStatus;   /*!< Error status. */

  /**
   * @brief Instance structure for the floating-point matrix memory pointer.
   */
  struct 
  {
    float *xhat,*xhatminus;   /*!< posteriori/priori estimate matrix memory pointer */
    float *u;                 /*!< control vector memory pointer */
    float *z;                 /*!< measurement vector memory pointer */
    float *B;                 /*!< control matrix memory pointer */ 
    float *A,*AT;             /*!< state transition matrix memory pointer */
    float *H,*HT;             /*!< measurement matrix memory pointer */
    float *P;                 /*!< posteriori error covariance matrix memory pointer */
    float *Pminus;            /*!< priori error covariance matrix memory pointer */
    float *Q;                 /*!< process noise covariance matrix memory pointer */ 
    float *R;                 /*!< measurement noise covariance matrix memory pointer */ 
    float *K;                 /*!< kalman gain matrix memory pointer */
    float *K_denominator;     /*!< K_denominator matrix memory pointer */
    float *cache_matrix[2];   /*!< calculate cache matrix memory pointer */
    float *cache_vector[2];   /*!< calculate cache vector memory pointer */
  }Memory_t;

  float *Output;  /*!< kalman filter output */

}Kalman_Filter_Info_TypeDef;

/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief Initializes the kalman filter according to the specified parameters in the Kalman_Filter_Info_TypeDef.
  */
extern arm_status Kalman_Filter_Init(Kalman_Filter_Info_TypeDef *kf,uint8_t xhatSize,uint8_t uSize,uint8_t zSize);
/**
  * @brief Update the Kalman Filter according to the specified parameters in the Kalman_Filter_Info_TypeDef.
  */
extern float *Kalman_Filter_Update(Kalman_Filter_Info_TypeDef *kf);

#endif //KALMAN_FILTER_H




