#ifndef __KALMAN_H
#define __KALMAN_H
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : kalman.h
  * @brief          : Header for kalman.c file.
  * 
  ******************************************************************************
  * @attention      : 1. fix user_malloc, allocate space in freertos heap(configTOTAL_HEAP_SIZE) instead of SRAM heap
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"
#include "stdlib.h"
#include "arm_math.h"
#include "cmsis_os.h"

/* Exported defines -----------------------------------------------------------*/
/**
 * @brief allocate space for an object
 */
#ifndef user_malloc
  #ifdef _CMSIS_OS_H
      #define user_malloc pvPortMalloc
  #else
      #define user_malloc malloc
  #endif
#endif

/**
 * @brief  matrix calculation.
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
 * @brief informations of the Chi Square Test.
 */
typedef struct
{
  bool TestFlag;    /*!< start Flag */
  matrix ChiSquare_Matrix;   /*!< test matrix */
  float ChiSquare_Data[1];    /*!< test value */
  float ChiSquareTestThresholds;    /*!< test Thresholds */
  uint8_t ChiSquareCnt;   /*!< test count */
  bool result;   /*!< test result */
}ChiSquareTest_Typedef;

/**
 * @brief informations of forgetting factor based adaptive noise covariance.
 */
typedef struct
{
  bool Adaptive_Enable;
  matrix r;       /*!< r(k) = z(k)-xhatminus(k) */
  matrix e;       /*!< e(k) = z(k)-xhat(k) */
  matrix temp_vector;
  matrix temp_matrix[2];   
  float b,dt,t;
  float alpha;   /*!< forgetting factor, alpha=1-(1-b)/(1-b^(t+1)), 0<b<1 */
}Adaptive_NoiseCov_Typdef;

/**
 * @brief informations of the kalman filter.
 */
typedef struct KF_Info_TypeDef
{
  uint16_t sizeof_float, sizeof_double; /*!< size of float/double */

  uint8_t xhatSize;   /*!< size of state vector */
  uint8_t uSize;      /*!< size of control vector */
  uint8_t zSize;      /*!< size of measurement vector */

  float dt;   /*!< system latency */
  float *MeasureInput; /*!< pointer to measure input  */
  float *ControlInput;  /*!< pointer to control input  */

  ChiSquareTest_Typedef ChiSquareTest;  /*!< Chi Square Test */

  Adaptive_NoiseCov_Typdef Adaptive_NoiseCov; /*!< adaptive noise covariance */

  /**
   * @brief Instance structure for the floating-point matrix structure.
   */
  struct 
  {
    matrix xhat;              /*!< posteriori state estimate */
    matrix xhatminus;         /*!< priori state estimate */
    matrix u;                 /*!< control-input  */
    matrix z;                 /*!< measurement  */
    matrix B;                 /*!< input-state  */ 
    matrix A,AT;              /*!< state transition  */
    matrix H,HT;              /*!< state-measurement  */
    matrix P;                 /*!< posteriori covariance  */
    matrix Pminus;            /*!< priori covariance  */
    matrix Q;                 /*!< process noise covariance  */ 
    matrix R;                 /*!< measurement noise covariance  */ 
    matrix K;                 /*!< kalman gain  */
    matrix S;                 /*!< S = H Pminus HT + R */
    matrix calc_matrix[2];    /*!< calculation process  */
    matrix calc_vector[2];    /*!< calculation process  */
  }mat;

  arm_status ErrorStatus;   /*!< Error status. */

  /**
   * @brief points to the data of the matrix.
   */
  struct 
  {
    float *xhat,*xhatminus;
    float *u;              
    float *z;              
    float *B;              
    float *A,*AT;          
    float *H,*HT;          
    float *P;              
    float *Pminus;         
    float *Q;              
    float *R;              
    float *K;              
    float *S;  
    float *calc_matrix[2];
    float *calc_vector[2];
  }pdata;

  /*!< flag to skip the specified step of kalman filter */
  uint8_t SkipStep1 : 1;
  uint8_t SkipStep2 : 1;
  uint8_t SkipStep3 : 1;
  uint8_t SkipStep4 : 1;
  uint8_t SkipStep5 : 1;
  uint8_t reserve   : 3;

  /**
   * @brief user functions that can replace steps of kalman filter.
   */
  void (*User_Function0)(struct KF_Info_TypeDef *kf);
  void (*User_Function1)(struct KF_Info_TypeDef *kf);
  void (*User_Function2)(struct KF_Info_TypeDef *kf);
  void (*User_Function3)(struct KF_Info_TypeDef *kf);
  void (*User_Function4)(struct KF_Info_TypeDef *kf);
  void (*User_Function5)(struct KF_Info_TypeDef *kf);
  void (*User_Function6)(struct KF_Info_TypeDef *kf);

  float *Output;  /*!< point to kalman filter output */

}Kalman_Info_TypeDef;

/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief Initializes the kalman filter.
  */
extern void Kalman_Filter_Init(Kalman_Info_TypeDef *kf,uint8_t xhatSize,uint8_t uSize,uint8_t zSize);
/**
  * @brief Update the Kalman Filter.
  */
extern float *Kalman_Filter_Update(Kalman_Info_TypeDef *kf);

#endif
