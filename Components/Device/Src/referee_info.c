/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : referee_info.c
  * @brief          : referee interfaces functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : to be tested
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "referee_info.h"
#include "crc.h"


/**
 * @brief  transform the bit8 to bit32
*/
float bit8TObit32(uint8_t change_info[4])
{
	union
	{
    float bit32;
		uint8_t  byte[4];
	}u32val;

  u32val.byte[0] = change_info[0];
  u32val.byte[1] = change_info[1];
  u32val.byte[2] = change_info[2];
  u32val.byte[3] = change_info[3];

	return u32val.bit32;
}

/**
 * @brief  transform the bit32 to bit8
*/
uint8_t bit32TObit8(uint8_t index_need,float bit32)
{
	union
	{
    float  bit32;
		uint8_t  byte[4];
	}u32val;

  u32val.bit32 = bit32;

	return u32val.byte[index_need];
}

/**
 * @brief  transform the bit8 to bit16
*/
int16_t bit8TObit16(uint8_t change_info[2])
{
	union
	{
    int16_t  bit16;
		uint8_t  byte[2];
	}u16val;

  u16val.byte[0] = change_info[0];
  u16val.byte[1] = change_info[1];

	return u16val.bit16;
}

/**
 * @brief  transform the bit16 to bit8
*/
uint8_t bit16TObit8(uint8_t index_need,int16_t bit16)
{
	union
	{
    int16_t  bit16;
		uint8_t  byte[2];
	}u16val;

  u16val.bit16 = bit16;
	return u16val.byte[index_need];
}

