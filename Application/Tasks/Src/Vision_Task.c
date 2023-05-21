/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Vision_Task.c
  * @brief          : Vision task
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "Vision_Task.h"
#include "api_trajectory.h"
#include "INS_Task.h"

/* Private variables -----------------------------------------------------------*/
/**
 * @brief structure that contains the information for the solved trajectory.
 */
SolveTrajectory_Typedef SolveTrajectory;

/* USER CODE BEGIN Header_Vision_Task */
/**
* @brief Function implementing the StartVisionTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Vision_Task */
void Vision_Task(void const * argument)
{
  /* USER CODE BEGIN Vision_Task */
	
  /* Infinite loop */
  for(;;)
  {
    /* update the solve trajectory */
		SolveTrajectory_Update(&SolveTrajectory,INS_Info.angle[2],INS_Info.angle[0],MiniPC_ReceivePacket.yaw,MiniPC_ReceivePacket.v_yaw,MiniPC_ReceivePacket.r1,MiniPC_ReceivePacket.r2,MiniPC_ReceivePacket.dz,18.f,MiniPC_ReceivePacket.armors_num);

    /* update the transmit euler angle in radians */
    MiniPC_SendPacket.pitch = INS_Info.angle[2];
    MiniPC_SendPacket.yaw = INS_Info.angle[0];
    MiniPC_SendPacket.roll = INS_Info.angle[1];

    /* transform the solved trajetory */
    SolveTrajectory_Transform(&MiniPC_SendPacket,&MiniPC_ReceivePacket,&SolveTrajectory);

    /* transmit the minipc frame data */
    MiniPC_SendFrameInfo(&MiniPC_SendPacket);

    osDelay(2);
  }
  /* USER CODE END Vision_Task */
}
//------------------------------------------------------------------------------

