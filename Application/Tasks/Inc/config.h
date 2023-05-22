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

/* General physics and mathematics constant ----------------------------------*/

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

/* Vision reslove constant --------------------------------------------------*/

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


#endif //ROBOT_CONFIG_H


