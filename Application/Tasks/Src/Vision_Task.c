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

/**
 * @brief structure that contains the information for the Vision.
 */
Vision_Info_Typedef Vision_Info = {
  .Fire_Yaw_Threshold = 0.2f,
};

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
    /* received minipc tracking , Enable the vision aiming */
    Vision_Info.IF_Aiming_Enable = (MiniPC_ReceivePacket.tracking == true);

    /* update the solve trajectory */
		SolveTrajectory_Update(&SolveTrajectory,INS_Info.angle[2],INS_Info.angle[0],MiniPC_ReceivePacket.yaw,MiniPC_ReceivePacket.v_yaw,MiniPC_ReceivePacket.r1,MiniPC_ReceivePacket.r2,MiniPC_ReceivePacket.dz,18.f,MiniPC_ReceivePacket.armors_num);

    /* update the transmit euler angle in radians */
    MiniPC_SendPacket.pitch = INS_Info.angle[2];
    MiniPC_SendPacket.yaw   = INS_Info.angle[0];
    MiniPC_SendPacket.roll  = INS_Info.angle[1];

    /* transform the solved trajetory */
    SolveTrajectory_Transform(&MiniPC_SendPacket,&MiniPC_ReceivePacket,&SolveTrajectory);

    /* Update the Gimbal target posture in degrees */
    Vision_Info.target_Pitch = SolveTrajectory.target_pitch * 57.295779513f;
    Vision_Info.target_Yaw = SolveTrajectory.target_yaw * 57.295779513f;
    Vision_Info.yawerror = fabs(SolveTrajectory.target_yaw - INS_Info.angle[0]) * 57.295779513f;

    /* Judge the fire acception */
    if(Vision_Info.yawerror < Vision_Info.Fire_Yaw_Threshold)
    {
      Vision_Info.IF_Fire_Accept = true;
    }
    else
    {
      Vision_Info.IF_Fire_Accept = false;
    }

    /* transmit the minipc frame data */
    MiniPC_SendFrameInfo(&MiniPC_SendPacket);

    osDelay(2);
  }
  /* USER CODE END Vision_Task */
}
//------------------------------------------------------------------------------

