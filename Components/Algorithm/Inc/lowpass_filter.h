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
}lpf_first_order_t;

typedef struct 
{
    float input;      //input value
    float output[3];  //output value
    float alpha[3];   //filter coefficient
}lpf_second_order_t;

/* Exported functions prototypes ---------------------------------------------*/
extern void lpf_first_order_init(lpf_first_order_t *lpf,float alpha,float frame_period);
extern float lpf_first_order_calc(lpf_first_order_t *lpf,float input);
extern void lpf_second_order_init(lpf_second_order_t *lpf,float alpha[3]);
extern float lpf_second_order_calc(lpf_second_order_t *lpf,float input);

#endif //LOWPASS_FILTER_H
