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
typedef struct
{
    float input;        //input value
    float output;       //output value
    float alpha;        //filter coefficient
    float frame_period; //frame perood
}LPF_FIRST_ORDER_t;

typedef struct 
{
    float input;      //input value
    float output[3];  //output value
    float alpha[3];   //filter coefficient
}LPF_SECOND_ORDER_t;

/* Exported functions prototypes ---------------------------------------------*/
extern void LPF_First_Order_Init(LPF_FIRST_ORDER_t *lpf,float alpha,float frame_period);
extern float LPF_First_Order_Calc(LPF_FIRST_ORDER_t *lpf,float input);
extern void LPF_Second_Order_Init(LPF_SECOND_ORDER_t *lpf,float alpha[3]);
extern float LPF_Second_Order_Calc(LPF_SECOND_ORDER_t *lpf,float input);

#endif //LOWPASS_FILTER_H
