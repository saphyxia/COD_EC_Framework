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

/* Exported variables ---------------------------------------------------------*/
/**
 * @brief Referee_RxDMA MultiBuffer
 */
uint8_t REFEREE_MultiRx_Buf[2][100];

/**
 * @brief Referee structure variable
 */
Referee_Info_TypeDef Referee_Info;

/* Private function prototypes -----------------------------------------------*/
uint32_t bit8TObit32(uint8_t change_info[4]);
uint8_t bit32TObit8(uint8_t index_need,uint32_t bit32);
int16_t bit8TObit16(uint8_t change_info[2]);
uint8_t bit16TObit8(uint8_t index_need,int16_t bit16);


void Referee_Info_Update(uint8_t *Buff,Referee_Info_TypeDef *referee)
{ 
  if(Buff[referee->index] == 0xA5 || verify_CRC8_check_sum(&Buff[referee->index],FrameHeader_Length) == true)
  {
    switch (bit8TObit16(&Buff[referee->index+FrameHeader_Length]))
    {
      case GAME_STATUS_ID:
        referee->game_status.game_type = Buff[referee->index+FrameHeader_Length+CMDID_Length] & 0xF0 >> 4;
        referee->game_status.game_progress = Buff[referee->index+FrameHeader_Length+CMDID_Length] & 0x0F;
        referee->game_status.stage_remain_time = bit8TObit16(&Buff[referee->index+FrameHeader_Length+CMDID_Length+1]);
      break;

      case EVENE_DATA_ID:
        referee->site_event.site.event_type = bit8TObit32(&Buff[referee->index+FrameHeader_Length+CMDID_Length]);
      break;

      case DART_REMAINING_TIME_ID:
        referee->dart_remaining.dart_remaining_time = Buff[referee->index+FrameHeader_Length+CMDID_Length];
      break;

      case DART_CLIENT_CMD_ID:
        referee->dart_client_cmd.dart_launch_opening_status = Buff[referee->index+FrameHeader_Length+CMDID_Length];
        referee->dart_client_cmd.dart_attack_target = Buff[referee->index+FrameHeader_Length+CMDID_Length+1];
        referee->dart_client_cmd.target_change_time = bit8TObit16(&Buff[referee->index+FrameHeader_Length+CMDID_Length+2]);
        referee->dart_client_cmd.operate_launch_cmd_time = bit8TObit16(&Buff[referee->index+FrameHeader_Length+CMDID_Length+4]);
      break;
      
      default:break;
    }
  }
}

/**
 * @brief  transform the bit8 to bit32
*/
uint32_t bit8TObit32(uint8_t change_info[4])
{
	union
	{
    uint32_t bit32;
		uint8_t  byte[4];
	}u32val;

  u32val.byte[0] = change_info[0];
  u32val.byte[1] = change_info[1];
  u32val.byte[2] = change_info[2];
  u32val.byte[3] = change_info[3];

	return u32val.bit32;
}
//------------------------------------------------------------------------------

/**
 * @brief  transform the bit32 to bit8
*/
uint8_t bit32TObit8(uint8_t index_need,uint32_t bit32)
{
	union
	{
    uint32_t  bit32;
		uint8_t  byte[4];
	}u32val;

  u32val.bit32 = bit32;

	return u32val.byte[index_need];
}
//------------------------------------------------------------------------------

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
//------------------------------------------------------------------------------

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
//------------------------------------------------------------------------------





