/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Config.c
  * @brief          : Configuare the Robot Functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/05/21
  * @version        : v1.0
  ******************************************************************************
  * @attention      : To be perfected
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

/* General physics and mathematics constants ---------------------------------*/

/**
 * @brief the value of local gravity acceleration
 */
#define GravityAccel  9.8035f  

/**
 * @brief radian system rotation degrees system , 180.f/PI
 */
#define RadiansToDegrees 57.295779513f

/**
 * @brief degrees system rotation radian system , PI/180.f
 */
#define DegreesToRadians 0.01745329251f

/* Vision reslove constants -------------------------------------------------*/

/**
 * @brief  Decision Marking mode
 *         0: select the minimum yaw armor
 *         1: select the minimum distance armor
 */
#define Distance_Yaw_Decision  0

/**
 * @brief ballistic coefficient
 * @note  17mm: 0.038
 *        42mm: 0.019
 */
#define Bullet_Coefficient  0.038f

/**
 * @brief the vertical distance of yaw axis to the muzzle(m)
 */
#define Camera_Muzzle_vertical   0.21265f

/**
 * @brief the horizontal distance of yaw axis to the muzzle(m)
 */
#define Camera_Muzzle_horizontal 0.19133f

/**
 * @brief the bias time of system(s), contains the communication delay and trigger delay
 */
#define FireSystem_BiasTime  0.2f

/* IMU reslove constants ---------------------------------------------------*/
/**
 * @brief the flag of bmi088 Calibration
 *        0: DISABLE
 *        1: ENABLE
 */
#define IMU_Calibration_ENABLE  1U

/**
 * @brief the index of pitch angle update
 */
#define IMU_ANGLE_INDEX_PITCH  2U
/**
 * @brief the index of yaw angle update
 */
#define IMU_ANGLE_INDEX_YAW   0U
/**
 * @brief the index of roll angle update
 */
#define IMU_ANGLE_INDEX_ROLL   1U

/**
 * @brief the index of pitch gyro update
 */
#define IMU_GYRO_INDEX_PITCH  0U
/**
 * @brief the index of yaw gyro update
 */
#define IMU_GYRO_INDEX_YAW   2U
/**
 * @brief the index of roll gyro update
 */
#define IMU_GYRO_INDEX_ROLL   1U

/**
 * @brief the index of pitch accel update
 */
#define IMU_ACCEL_INDEX_PITCH  1U
/**
 * @brief the index of yaw accel update
 */
#define IMU_ACCEL_INDEX_YAW   2U
/**
 * @brief the index of roll accel update
 */
#define IMU_ACCEL_INDEX_ROLL   0U


#endif //ROBOT_CONFIG_H


