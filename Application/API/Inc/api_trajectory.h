/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : api_trajectory.h
  * @brief          : solve trajectory
  * @author         : Yan Yuanbin
  * @date           : 2023/05/21
  * @version        : v1.0
  ******************************************************************************
  * @attention      : see https://github.com/chenjunnn/rm_auto_aim
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef API_TRAJECTORY_H
#define API_TRAJECTORY_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"
#include "minipc.h"

/* Exported defines -----------------------------------------------------------*/
/**
 * @brief the value of Gravity
 */
#define GRAVITY  9.79f

/* Exported types ------------------------------------------------------------*/
/**
 * @brief typedef structure that contains the information for the target armor posure.
 */
typedef struct 
{
    float x;       /*!< x in the ros coordinate system */
    float y;       /*!< y in the ros coordinate system */
    float z;       /*!< z in the ros coordinate system */
    float yaw;     /*!< target yaw angle */
}TargetArmor_Posure;

/**
 * @brief typedef structure that contains the information for the solved trajectory.
 */
typedef struct
{
    float bullet_speed;   /*!< referee bullet speed */
    float bullet_time;    /*!< ballistic time */
    float current_pitch;  /*!< current pitch angle */
    float current_yaw;    /*!< current yaw angle */

    float yaw_calc;       /*!< yaw angle in algorithm */
    float yawgyro_calc;   /*!< yaw gyro in algorithm */
    float r1;             /*!< Distance of target center to front and rear armor plates */
    float r2;             /*!< Distance of target center to armor plates in sides */
    float dz;             /*!< unknown */
    uint8_t armor_type;   /*!< the Type of armor */

    float target_yaw;      /*!< target yaw angle */
    float target_pitch;    /*!< target pitch angle  */
    
    TargetArmor_Posure target_posure[4];    /* target armor posure */
}SolveTrajectory_Typedef;

/* Exported functions prototypes ---------------------------------------------*/
/**
 * @brief  Update the solve trajectory 
 */
extern void SolveTrajectory_Update(SolveTrajectory_Typedef *SolveTrajectory,float picth,float yaw,float target_yaw,float v_yaw,float r1,float r2,float dz,float bullet_speed,float armor_type);
/**
 * @brief  Transform the solve trajectory 
 */
extern void SolveTrajectory_Transform(MiniPC_SendPacket_Typedef *MiniPCTxData,MiniPC_ReceivePacket_Typedef *MiniPCRxData,SolveTrajectory_Typedef *SolveTrajectory);


#endif




