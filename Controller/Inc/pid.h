#ifndef __PID_H
#define __PID_H
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : pid.h
  * @brief          : Header for pid.c file.
  * 
  ******************************************************************************
  * @attention      : none
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "string.h"
#include "stdlib.h"
#include "stdbool.h"
#include "math.h"

/* Exported define -----------------------------------------------------------*/
/**
 * @brief restricts x of the specified value.
 */
#define VAL_LIMIT(x,min,max)  do{ \
                                    if ((x) > (max)) {(x) = (max);} \
                                    else if ((x) < (min)) {(x) = (min);} \
                                    else {;} \
                                }while(0U)

/**
 * @brief number of pid parameters
*/
#ifndef PID_PARAMETER_NUM 
  #define PID_PARAMETER_NUM 6							
#endif

/* Exported types ------------------------------------------------------------*/

/**
 * @brief enum types of pid controller.
 */
typedef enum
{
	PID_Type_None = 0x00U,         /*!< No Type */
	PID_POSITION = 0x01U,          /*!< position pid */
	PID_VELOCITY = 0x02U,          /*!< velocity pid */
  PID_TYPE_NUM,
}PID_Type_e;

/**
 * @brief pid error handler.
 */
typedef struct
{
  uint8_t INIT_FAILED : 1;  /*!< Initialize failed */
  uint8_t RET_NAN_INF : 1;  /*!< Not a number or infinity */
  uint8_t reserve : 6;

  uint16_t ErrorCount;  /*!< Error count */
}PID_ErrorHandler_Typedef;

/**
 * @brief parameters of the pid controller.
 */
typedef struct
{
  float kp;             /*!< Proportional Gain */
  float ki;             /*!< Integral Gain */
  float kd;             /*!< Derivative Gain */

	float Deadband;     /*!< Deadband of error */
  float MaxIntegral;  /*!< Max Integral */
  float MaxOutput;    /*!< Max Output */
}PID_Parameter_Typedef;

/**
 * @brief structure of the pid controller.
 */
typedef struct _PID_TypeDef
{
	PID_Type_e type;    /*!< type of pid controller */

	float target;       /*!< target value */
	float measure;      /*!< measurement value */
  float err[3];       /*!< Error;previous Error;penultimate Error */
	float integral;     /*!< Integral */

	PID_Parameter_Typedef param;            /*!< parameters of pid */
  PID_ErrorHandler_Typedef ERRORHandler;  /*!< error handler. */

  float Pout;         /*!< Proportional Output */
  float Iout;         /*!< Integral Output */
  float Dout;         /*!< Derivative Output */
  float Output;       /*!< PID Output */

  /**
   * @brief Initialize PID Parameters.
   */
  uint8_t (*Param_Init)(struct _PID_TypeDef *pid,float para[PID_PARAMETER_NUM]);
  
  /**
   * @brief Clear Pid Calculation.
  */
  void (*Clear)(struct _PID_TypeDef *pid);
				
}PID_Info_TypeDef;

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief Initializes PID Controller.
 * @param pid: pointer to PID_Info_TypeDef structure that
 *         contains the information of PID controller.
 * @param type: type of pid controller
 * @param para: pointer to a floating-point array that
 *         contains the parameters for the PID controller.
 * @retval none
 */
extern void PID_Init(PID_Info_TypeDef *pid,PID_Type_e type,float para[PID_PARAMETER_NUM]);
//------------------------------------------------------------------------------

/**
  * @brief  Caculate the PID Controller
  * @param  *pid pointer to a PID_TypeDef_t structure that contains
  *              the configuration information for the specified PID. 
  * @param  Target  Target for the pid controller
  * @param  Measure Measure for the pid controller
  * @retval the Pid Output
  */
extern float f_PID_Calculate(PID_Info_TypeDef *pid, float target,float measure);
//------------------------------------------------------------------------------

#endif