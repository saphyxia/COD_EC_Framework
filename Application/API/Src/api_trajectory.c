/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : api_trajectory.c
  * @brief          : solve trajectory
  * @author         : Yan Yuanbin
  * @date           : 2023/05/21
  * @version        : v1.0
  ******************************************************************************
  * @attention      : see https://github.com/chenjunnn/rm_auto_aim
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "api_trajectory.h"
#include "math.h"
#include "arm_math.h"

/* Private defines -----------------------------------------------------------*/
/**
 * @brief  Decision Marking mode
 *         0: select the minimum yaw armor
 *         1: select the minimum distance armor
 */
#define Distance_Yaw_Decision  0

/**
 * @brief ballistic coefficient
 */
#define Bullet_Coefficient  0.076f

/**
 * @brief the vertical distance of yaw motor to the muzzle(m)
 */
#define Camera_Muzzle_Height  0.21265f
/**
 * @brief the distance of Muzzle push(m)
 */
#define Camera_Muzzle_Push    0.19133f

/**
 * @brief the bias time of system(s), contains the communication delay and trigger delay
 */
#define System_BiasTime  0.1f

/* Private function prototypes -----------------------------------------------*/
/**
 * @brief  Calculate Bullet Model offset
 */
static float Trajectory_BulletModel(float ,float ,float ,float *,float );
/**
  * @brief   Update the Trajectory pitch angle
  */
static float Trajectory_Picth_Update(float ,float ,SolveTrajectory_Typedef *);

/**
 * @brief  Update the solve trajectory 
 * @param  SolveTrajectory: pointer to a SolveTrajectory_Typedef structure that
 *          contains the information of solved trajectory.
 * @param  picth: current pitch angle
 * @param  yaw: current yaw amngle 
 * @param  target_yaw: minipc receive target yaw angle
 * @param  v_yaw: minipc receive yaw gyro 
 * @param  r1: minipc received Distance of target center to front and rear armor plates
 * @param  r2: minipc received Distance of target center to armor plates in sides
 * @param  dz: minipc receive unknown
 * @param  armors_num: the num of armor
 * @retval none
 */
void SolveTrajectory_Update(SolveTrajectory_Typedef *SolveTrajectory,float picth,float yaw,float target_yaw,float v_yaw,float r1,float r2,float dz,float bullet_speed,float armors_num)
{
    SolveTrajectory->current_pitch = picth;
    SolveTrajectory->current_yaw = yaw;
    
    SolveTrajectory->bullet_speed = bullet_speed;

    SolveTrajectory->yaw_calc = target_yaw;
    SolveTrajectory->yawgyro_calc = v_yaw;
    SolveTrajectory->r1 = r1;
    SolveTrajectory->r2 = r2;
    SolveTrajectory->dz = dz;

    SolveTrajectory->armors_num = armors_num;
}
//------------------------------------------------------------------------------

/**
 * @brief  Transform the solve trajectory 
 * @param  bias_time: system delay
 * @param  MiniPCTxData: pointer to a MiniPC_SendPacket_Typedef structure that
  *          contains the information of transmit Data.
 * @param  MiniPCRxData: pointer to a MiniPC_ReceivePacket_Typedef structure that
  *          contains the information of Received Data.
 * @param  SolveTrajectory: pointer to a SolveTrajectory_Typedef structure that
  *          contains the information of solved trajectory.
 * @retval none
 */
void SolveTrajectory_Transform(MiniPC_SendPacket_Typedef *MiniPCTxData,MiniPC_ReceivePacket_Typedef *MiniPCRxData,SolveTrajectory_Typedef *SolveTrajectory)
{
    /* calculate the timedelay */
    float timeDelay = System_BiasTime + SolveTrajectory->bullet_time;

    /* calculate the yaw linear prediction */
    SolveTrajectory->yaw_calc += SolveTrajectory->yawgyro_calc * timeDelay;

    /* calculate the for armor position */
	bool use_1 = 1;

    /* the index of target armor */
    uint8_t index = 0; 
    
    /* Judge the armor num,balance armor num is 2 */
    if (SolveTrajectory->armors_num == 2) 
    {
        for (uint8_t i = 0; i<2; i++) 
        {
            SolveTrajectory->target_posure[i].x = MiniPCRxData->x - SolveTrajectory->r1*cos(SolveTrajectory->yaw_calc + i * PI);
            SolveTrajectory->target_posure[i].y = MiniPCRxData->y - SolveTrajectory->r1*sin(SolveTrajectory->yaw_calc + i * PI);
            SolveTrajectory->target_posure[i].z = MiniPCRxData->z;
            SolveTrajectory->target_posure[i].yaw = SolveTrajectory->yaw_calc + i * PI;
        }

        /* Judge the minimum yaw armor */
        float yaw_diff_min = fabsf(SolveTrajectory->yaw_calc - SolveTrajectory->target_posure[0].yaw);
        float temp_yaw_diff = fabsf(SolveTrajectory->yaw_calc - SolveTrajectory->target_posure[0].yaw);

        if (temp_yaw_diff < yaw_diff_min)
        {
            yaw_diff_min = temp_yaw_diff;
            index = 1;
        }
    }
    /* outpost armor num is 3 */
    /* normal armor num is 4 */
    else 
    {
        /* store the armor posure */
        for (uint8_t i = 0; i<4; i++)
        {
            float r = use_1 ? SolveTrajectory->r1 : SolveTrajectory->r2;
            SolveTrajectory->target_posure[i].x = MiniPCRxData->x - r*cos(SolveTrajectory->yaw_calc + i * PI/2.f);
            SolveTrajectory->target_posure[i].y = MiniPCRxData->y - r*sin(SolveTrajectory->yaw_calc + i * PI/2.f);
            SolveTrajectory->target_posure[i].z = use_1 ? MiniPCRxData->z : SolveTrajectory->dz;
            SolveTrajectory->target_posure[i].yaw = SolveTrajectory->yaw_calc + i * PI/2.f;
            use_1 = !use_1;
        }

#if Distance_Yaw_Decision
        /* select the minimum distance armor */
        float dis_diff_min = arm_sqrt_f32(SolveTrajectory->target_posure[0].x * SolveTrajectory->target_posure[0].x + SolveTrajectory->target_posure[0].y * SolveTrajectory->target_posure[0].y);
        for (uint8_t i = 1; i<4; i++)
        {
            float temp_dis_diff = arm_sqrt_f32(SolveTrajectory->target_posure[i].x * SolveTrajectory->target_posure[i].x + SolveTrajectory->target_posure[i].y * SolveTrajectory->target_posure[i].y);
            if (temp_dis_diff < dis_diff_min)
            {
                dis_diff_min = temp_dis_diff;
                index = i;
            }
        }
#else
        /* select the minimum yaw armor */
        float yaw_diff_min = fabsf(SolveTrajectory->yaw_calc - SolveTrajectory->target_posure[0].yaw);
        for (uint8_t i = 1; i<4; i++) {
            float temp_yaw_diff = fabsf(SolveTrajectory->yaw_calc - SolveTrajectory->target_posure[i].yaw);
            if (temp_yaw_diff < yaw_diff_min)
            {
                yaw_diff_min = temp_yaw_diff;
                index = i;
            }
        }
#endif
    }

    MiniPCTxData->aim_x = SolveTrajectory->target_posure[index].z + MiniPCRxData->vx * timeDelay;
    MiniPCTxData->aim_y = SolveTrajectory->target_posure[index].x + MiniPCRxData->vy * timeDelay;
    MiniPCTxData->aim_z = SolveTrajectory->target_posure[index].y + MiniPCRxData->vz * timeDelay;

    /* distance in algorithm */
    float distance_calc = 0.f;
    
    arm_sqrt_f32((MiniPCTxData->aim_x) * (MiniPCTxData->aim_x) + (MiniPCTxData->aim_y) * (MiniPCTxData->aim_y),&distance_calc);

    SolveTrajectory->target_pitch = -Trajectory_Picth_Update( distance_calc + Camera_Muzzle_Push,
            SolveTrajectory->target_posure[index].z - Camera_Muzzle_Height, SolveTrajectory);
    SolveTrajectory->target_yaw = (float)(atan2(MiniPCTxData->aim_y, MiniPCTxData->aim_x));
}
//------------------------------------------------------------------------------

/**
 * @brief  Calculate Bullet Model offset
 * @param  distance: distance of target armor
 * @param  bullet_k: ballistic coefficient
 * @param  bullet_speed: bullet speeed from referee
 * @param  bullet_Time： pointer to the ballistic time
 * @param  current_pitch: current pitch angle
 * @retval Bullet Model offset in radians
 * @note   t = (e^(v*x)-1)/(k*v*cos(pitch))
 *         offset = v*sin(pitch)*t - 1/2*g*t^2
 */
static float Trajectory_BulletModel(float distance,float bullet_k,float bullet_speed,float *bullet_Time,float current_pitch)
{
    float offset = 0.f;

    *bullet_Time = (float)((exp(bullet_k * distance)- 1.f) / (bullet_k * bullet_speed * cos(current_pitch)));

    offset = (float)(bullet_speed * arm_sin_f32(current_pitch) * *bullet_Time - GRAVITY * *bullet_Time * *bullet_Time / 2.f);

    return offset;
}
//------------------------------------------------------------------------------

/**
  * @brief   Update the Trajectory pitch angle
  * @param   distance: distance of target center
  * @param   height: height of target armor
  * @param   solvetrajectory: pointer to a SolveTrajectory_Typedef structure that
  *          contains the information of solved trajectory.
  * @retval  solved Trajectory pitch angle in radians
  */
static float Trajectory_Picth_Update(float distance,float height,SolveTrajectory_Typedef *SolveTrajectory)
{
    float z_temp = 0.f,dz = 0.f;
    float pitchoffset = 0.f,pitchangle = 0.f;

    /* store the height of target armor */
    z_temp = height;

    for(uint8_t i=0; i<20; i++)
    {
        /* calculate the traget pitch angle */
        /* pitchangle = actan(z,x) */
        pitchangle = (float)atan2(z_temp,distance);

        /* Calculate the Bullet Model offset */
        pitchoffset = Trajectory_BulletModel(distance,Bullet_Coefficient,SolveTrajectory->bullet_speed,&SolveTrajectory->bullet_time,SolveTrajectory->current_pitch);

        /* calculate the difference of height */
        dz = 0.3f*(height - pitchoffset);
        
        /* height points compensation */
        z_temp += dz;

        /* judge the difference of height */
        if(fabs(dz) < 0.00001f)
        {
            break;
        }
    }
    return pitchangle;
}
//------------------------------------------------------------------------------
