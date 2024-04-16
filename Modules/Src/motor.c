/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : motor.c
  * Description        : Implementation of motor communication
  ******************************************************************************
  * @author         : YuanBin Yan
  * @date           : 2024/02/23
  * @version        : 1.2.2
  * @attention      : none
  *
  * Copyright 2024 COD USTL.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "motor.h"

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  transform the encoder(0-8192) to anglesum(3.4E38)
  */
static float encoder_to_anglesum(DJI_Motor_Info_Typedef *motor,float reduction_ratio,uint16_t MAXencoder);
/**
  * @brief  transform the encoder(0-8192) to angle(-180-180)
  */
static float encoder_to_angle(DJI_Motor_Info_Typedef *motor,float reduction_ratio,uint16_t MAXencoder);

/**
  * @brief  Update the DJI motor Information
  * @param  rxBuf  point to the can receive message
  * @param  DJI_Motor point to DJI_Motor_Info_t structure 
  *                   that contains the informations of DJI motor
  * @retval None
  */
void DJI_Motor_Info_Update(uint8_t *rxBuf,DJI_Motor_Info_Typedef *DJI_Motor)
{
  /* transform the general motor data */
  DJI_Motor->temperature = rxBuf[6];
  DJI_Motor->encoder  = ((int16_t)rxBuf[0] << 8 | (int16_t)rxBuf[1]);
  DJI_Motor->velocity = ((int16_t)rxBuf[2] << 8 | (int16_t)rxBuf[3]);
  DJI_Motor->current  = ((int16_t)rxBuf[4] << 8 | (int16_t)rxBuf[5]);
	
  /* transform the encoder to anglesum */
  switch(DJI_Motor->type)
  {
    case DJI_GM6020:
        DJI_Motor->angle = encoder_to_anglesum(DJI_Motor,1.f,8192);
    break;

    case DJI_M3508:
        DJI_Motor->angle = encoder_to_anglesum(DJI_Motor,3591.f/187.f,8192);
    break;
    
    case DJI_M2006:
        DJI_Motor->angle = encoder_to_anglesum(DJI_Motor,36.f,8192);
    break;
    
    default:break;
  }
}
//------------------------------------------------------------------------------

/**
  * @brief  transform the encoder(0-8192) to anglesum(3.4E38)
  * @param  *Info point to DJI_Motor_Info_Typedef structure that 
  *				  contains the infomations of motor
  * @param  reduction_ratio reduction ratio
  * @param  MAXencoder   max encoder angle
  * @retval anglesum
  */
static float encoder_to_anglesum(DJI_Motor_Info_Typedef *motor,float reduction_ratio,uint16_t MAXencoder)
{
  float res1 = 0,res2 =0;
  
  if(motor == NULL) return 0;
  
  /* check the motor Initlization */
  if(motor->Initlized != true)
  {
    /* update the last encoder */
    motor->encoder_prev = motor->encoder;

    /* reset the angle */
    motor->angle = 0;

    /* Set the init flag */
    motor->Initlized = true;
  }
  
  /* get the possiable minimal encoder err */
  if(motor->encoder < motor->encoder_prev)
  {
    res1 = motor->encoder - motor->encoder_prev + MAXencoder;
  }
  else if(motor->encoder > motor->encoder_prev)
  {
    res1 = motor->encoder - motor->encoder_prev - MAXencoder;
  }
  res2 = motor->encoder - motor->encoder_prev;
  
  /* update the last encoder */
  motor->encoder_prev = motor->encoder;
  
  /* transforms the encoder to tolangle */
  if(fabsf(res1) > fabsf(res2))
  {
  	motor->angle += (float)res2/(MAXencoder*reduction_ratio)*360.f;
  }
  else
  {
  	motor->angle += (float)res1/(MAXencoder*reduction_ratio)*360.f;
  }
  
  return motor->angle;
}
//------------------------------------------------------------------------------

/**
  * @brief  float loop constrain
  * @param  minValue minimum of the input
  * @param  maxValue maximum of the input
  * @retval 
  */
static float f_loop_constrain(float Input, float minValue, float maxValue)
{
  if (maxValue < minValue)
  {
    return Input;
  }
  
  float len = maxValue - minValue;    

  if (Input > maxValue)
  {
      do{
          Input -= len;
      }while (Input > maxValue);
  }
  else if (Input < minValue)
  {
      do{
          Input += len;
      }while (Input < minValue);
  }
  return Input;
}
//------------------------------------------------------------------------------

/**
  * @brief  transform the encoder(0-8192) to angle(-180-180)
  * @param  *Info point to DJI_Motor_Info_Typedef structure that 
  *				  contains the infomations of motor
  * @param  reduction_ratio reduction ratio
  * @param  MAXencoder   max encoder angle
  * @retval angle
  */
static float encoder_to_angle(DJI_Motor_Info_Typedef *motor,float reduction_ratio,uint16_t MAXencoder)
{	
  float encoder_diff = 0.f;
  
  /* check the motor Initlization */
  if(motor->Initlized != true)
  {
    /* update the last encoder */
    motor->encoder_prev = motor->encoder;

    /* reset the angle */
    motor->angle = 0;

    /* Set the init flag */
    motor->Initlized = true;
  }
  
  encoder_diff = motor->encoder - motor->encoder_prev;
  
  /* 0 -> MAXencoder */		
  if(encoder_diff > MAXencoder*0.5f)
  {
    motor->angle += (float)(encoder_diff - MAXencoder)/(MAXencoder*reduction_ratio)*360.f;
  }
  /* MAXencoder-> 0 */		
  else if(encoder_diff < -MAXencoder*0.5f)
  {
    motor->angle += (float)(encoder_diff + MAXencoder)/(MAXencoder*reduction_ratio)*360.f;
  }
  else
  {
    motor->angle += (float)(encoder_diff)/(MAXencoder*reduction_ratio)*360.f;
  }
  
  /* update the last encoder */
  motor->encoder_prev = motor->encoder;
  
  /* loop constrain */
  f_loop_constrain(motor->angle,-180.f,180.f);

  return motor->angle;
}
//------------------------------------------------------------------------------
