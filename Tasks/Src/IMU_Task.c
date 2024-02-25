/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : IMU_Task.c
  * Description        : Code for IMU task
  *                      using Mahony algorithm and Extended Kalman Filter
  ******************************************************************************
  * @author         : YuanBin Yan
  * @date           : 2024/02/22
  * @version        : 1.2.2
  * @attention      : 1. fix the usage error of osDelayUntil
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "IMU_Task.h"
#include "bmi088.h"
#include "quaternion.h"
#include "lpf.h"
#include "pid.h"
#include "timer.h"

/**
  * @brief Instance structure of IMU.
  */
IMU_Info_Typedef IMU_Info;

/**
  * @brief Instance structure of BMI088.
  */
BMI088_Info_Typedef BMI088_Info;

/**
  * @brief parameters of accel second order low-pass filter.
  */
float Accel_Slpf_alpha[3] = {0.002329458745586203f, 1.929454039488895f, -0.93178349823448126f};

/**
  * @brief Instance structure of accel second order low-pass filter.
  */
SecondOrderLowpass_Typedef BMI088_Accel_Slpf[3];

/**
  * @brief data of state transition matrix.
  */
static float QuatEKF_Data_A[36]={1, 0, 0, 0, 0, 0,
                                0, 1, 0, 0, 0, 0,
                                0, 0, 1, 0, 0, 0,
                                0, 0, 0, 1, 0, 0,
                                0, 0, 0, 0, 1, 0,
                                0, 0, 0, 0, 0, 1};
/**
  * @brief data of posteriori covariance matrix.
  */
static float QuatEKF_Data_P[36]= {100000, 0.1, 0.1, 0.1, 0.1, 0.1,
                                 0.1, 100000, 0.1, 0.1, 0.1, 0.1,
                                 0.1, 0.1, 100000, 0.1, 0.1, 0.1,
                                 0.1, 0.1, 0.1, 100000, 0.1, 0.1,
                                 0.1, 0.1, 0.1, 0.1, 100, 0.1,
                                 0.1, 0.1, 0.1, 0.1, 0.1, 100};

/**
  * @brief parameters of Heat Power PID.
  */
static float HeatPower_PID_Param[PID_PARAMETER_NUM]={1600,20,0,0,0,10000};

/**
  * @brief Instance structure of Heat Power PID.
  */
PID_Info_TypeDef HeatPower_PID;

/**
  * @brief Instance structure of quaternion.
  */
Quat_Info_Typedef Quat_Info;

/**
  * @brief  Update BMI088 Heat Power PWM
  * @param  temp  measure temperature of the BMI088 
  * @retval none
  */
static void BMI088_HeatPower_Control(float temp)
{
	f_PID_Calculate(&HeatPower_PID,40.f,temp);
	
	VAL_LIMIT(HeatPower_PID.Output,0,10000);

	Heat_Power_Control((uint16_t)HeatPower_PID.Output);
}
//------------------------------------------------------------------------------

/**
 * @brief Initialize the IMU_Task.
 */
static void IMU_Task_Init(void)
{
	// update bmi088 informations
	BMI088_Info_Update(&BMI088_Info);
	
  /* Initializes the filter output */
  SecondOrderLowpass_Init(&BMI088_Accel_Slpf[0],Accel_Slpf_alpha,BMI088_Info.accel[IMU_ACCEL_GYRO_INDEX_PITCH]);
  SecondOrderLowpass_Init(&BMI088_Accel_Slpf[1],Accel_Slpf_alpha,BMI088_Info.accel[IMU_ACCEL_GYRO_INDEX_YAW])  ;
  SecondOrderLowpass_Init(&BMI088_Accel_Slpf[2],Accel_Slpf_alpha,BMI088_Info.accel[IMU_ACCEL_GYRO_INDEX_ROLL]) ;
	
  /* Initializes the Temperature Control PID  */
	PID_Init(&HeatPower_PID,PID_VELOCITY,HeatPower_PID_Param);
	
  /* Initializes the Quaternion EKF */
	QuatEKF_Init(&Quat_Info,10.f, 0.001f, 1000000.f,QuatEKF_Data_A,QuatEKF_Data_P);
}

/* USER CODE BEGIN Header_IMU_Task */
/**
* @brief Function implementing the IMUTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IMU_Task */
void IMU_Task(void const * argument)
{
  /* USER CODE BEGIN IMU_Task */

  // Initialize the IMU Task
  IMU_Task_Init();

  // Holds the time at which the task was last unblocked.
  TickType_t ticks = 0;

  // Initialize the time.
  // Will be update in function osDelayUntil.
  ticks = osKernelSysTick();

  /* Infinite loop */
  for(;;)
  {
		// update bmi088 informations
		BMI088_Info_Update(&BMI088_Info);

    // store the data of BMI088 accel and gyro
    IMU_Info.accel[IMU_ACCEL_GYRO_INDEX_PITCH] = SecondOrderLowpass_Update(&BMI088_Accel_Slpf[0],BMI088_Info.accel[IMU_ACCEL_GYRO_INDEX_PITCH]);
    IMU_Info.accel[IMU_ACCEL_GYRO_INDEX_YAW]   = SecondOrderLowpass_Update(&BMI088_Accel_Slpf[1],BMI088_Info.accel[IMU_ACCEL_GYRO_INDEX_YAW])  ;
    IMU_Info.accel[IMU_ACCEL_GYRO_INDEX_ROLL]  = SecondOrderLowpass_Update(&BMI088_Accel_Slpf[2],BMI088_Info.accel[IMU_ACCEL_GYRO_INDEX_ROLL]) ;

    IMU_Info.gyro[IMU_ACCEL_GYRO_INDEX_PITCH] = BMI088_Info.gyro[IMU_ACCEL_GYRO_INDEX_PITCH];
    IMU_Info.gyro[IMU_ACCEL_GYRO_INDEX_YAW]   = BMI088_Info.gyro[IMU_ACCEL_GYRO_INDEX_YAW]  ;
    IMU_Info.gyro[IMU_ACCEL_GYRO_INDEX_ROLL]  = BMI088_Info.gyro[IMU_ACCEL_GYRO_INDEX_ROLL] ;

		/* Update the Quaternion EKF */
    QuatEKF_Update(&Quat_Info,IMU_Info.gyro,IMU_Info.accel,0.001f);

    IMU_Info.angle[IMU_ANGLE_INDEX_YAW] = Quat_Info.angle[IMU_ANGLE_INDEX_YAW];
    IMU_Info.angle[IMU_ANGLE_INDEX_PITCH] = Quat_Info.angle[IMU_ANGLE_INDEX_PITCH];
    IMU_Info.angle[IMU_ANGLE_INDEX_ROLL] = Quat_Info.angle[IMU_ANGLE_INDEX_ROLL];  

		/* store the angle in degrees. */
    IMU_Info.pit_angle = Quat_Info.angle[IMU_ANGLE_INDEX_PITCH]*RadiansToDegrees;
    IMU_Info.yaw_angle = Quat_Info.angle[IMU_ANGLE_INDEX_YAW]*RadiansToDegrees;
    IMU_Info.rol_angle = Quat_Info.angle[IMU_ANGLE_INDEX_ROLL]*RadiansToDegrees;

    /* store the yaw total angle */
		if(IMU_Info.yaw_angle - IMU_Info.last_yawangle < -180.f)
		{
			IMU_Info.YawRoundCount++;
		}
		else if(IMU_Info.yaw_angle - IMU_Info.last_yawangle > 180.f)
		{
			IMU_Info.YawRoundCount--;
		}
		IMU_Info.last_yawangle = IMU_Info.yaw_angle;
		
		IMU_Info.yaw_tolangle = IMU_Info.yaw_angle + IMU_Info.YawRoundCount*360.f;

    /* Update the INS gyro in degrees */
    IMU_Info.pit_gyro = IMU_Info.gyro[IMU_ACCEL_GYRO_INDEX_PITCH]*RadiansToDegrees;
    IMU_Info.yaw_gyro = IMU_Info.gyro[IMU_ACCEL_GYRO_INDEX_YAW]*RadiansToDegrees;
    IMU_Info.rol_gyro = IMU_Info.gyro[IMU_ACCEL_GYRO_INDEX_ROLL]*RadiansToDegrees;

		if(ticks%2 == 0)
		{
			BMI088_HeatPower_Control(BMI088_Info.temperature);
		}

    // Delay the task until 1 ms
    osDelayUntil(&ticks,1);
  }
  /* USER CODE END IMU_Task */
}
//------------------------------------------------------------------------------

