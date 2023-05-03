/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : lowpass_filter.c
  * @brief          : lowpass filter 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : To be perfected
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H


/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported types ------------------------------------------------------------*/
/**
 * @brief typedef structure that contains the information  for the first order lowpass filter.
 */
typedef struct
{
    float input;          /*!< input value */
    float output;         /*!< output value */
    float alpha;          /*!< filter coefficient */
    float frame_period;   /*!< frame perood */

}LPF_First_Order_TypeDef;

/**
 * @brief typedef structure that contains the information  for the second order lowpass filter.
 */
typedef struct 
{
    float input;       /*!< input value */
    float output[3];   /*!< output value */
    float alpha[3];    /*!< filter coefficient */

}LPF_Second_Order_TypeDef;

/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief Initializes the first order lowpass filter according to the specified parameters in the
  *         LPF_First_Order_TypeDef.
  */
extern void LPF_First_Order_Init(LPF_First_Order_TypeDef *lpf,float alpha,float frame_period);
/**
  * @brief Update the first order lowpass filter according to the specified parameters in the
  *         LPF_First_Order_TypeDef.
  */
extern float LPF_First_Order_Update(LPF_First_Order_TypeDef *lpf,float input);
/**
  * @brief Initializes the Second order lowpass filter according to the specified parameters in the
  *         LPF_Second_Order_TypeDef.
  */
extern void LPF_Second_Order_Init(LPF_Second_Order_TypeDef *lpf,float alpha[3]);
/**
  * @brief Update the Second order lowpass filter according to the specified parameters in the
  *         LPF_Second_Order_TypeDef.
  */
extern float LPF_Second_Order_Update(LPF_Second_Order_TypeDef *lpf,float input);

#endif //LOWPASS_FILTER_H
