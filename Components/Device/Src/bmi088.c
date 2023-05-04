/* USER CODE BEGIN Header */
/**
  *******************************************************************************
  * @file           : bmi088.c
  * @brief          : bmi088 interfaces functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : none
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "bmi088.h"
#include "stdbool.h"
#include "bsp_gpio.h"
#include "bsp_spi.h"
#include "bsp_tick.h"
#include "pid.h"

/* Private define ------------------------------------------------------------*/
#if defined(BMI088_USE_SPI)
/**
  * @brief Write the single register value to the sensor
  */
static void BMI088_Write_Single_Reg(uint8_t reg, uint8_t data);
/**
  * @brief Read the single register value to the sensor
  */
static void BMI088_Read_Single_Reg(uint8_t reg, uint8_t *return_data);
/**
  * @brief Read the multi register value to the sensor
  */
static void BMI088_Read_Multi_Reg(uint8_t reg, uint8_t *buf, uint8_t len);

/**
 * @brief macro definition of the BMI088_Accel_Write_Single_Reg that write a single data in the specified accelerator register
 * @param reg: the specified register address
 * @param data: the single data
 * @retval None
 */
#define BMI088_Accel_Write_Single_Reg(reg, data) \
    {                                            \
        BMI088_ACCEL_NS_L();                     \
        BMI088_Write_Single_Reg((reg), (data));  \
        BMI088_ACCEL_NS_H();                     \
    }
/**
 * @brief macro definition of the BMI088_Accel_Read_Single_Reg that read a single data in the specified accelerator register
 * @param reg: the specified register address
 * @param data: the single data
 * @retval None
 */
#define BMI088_Accel_Read_Single_Reg(reg, data) \
    {                                           \
        BMI088_ACCEL_NS_L();                    \
        BMI088_Read_Write_Byte((reg) | 0x80);   \
        BMI088_Read_Write_Byte(0x55);           \
        (data) = BMI088_Read_Write_Byte(0x55);  \
        BMI088_ACCEL_NS_H();                    \
    }
/**
 * @brief macro definition of the BMI088_Accel_Read_Multi_Reg that read multi datas in the specified accelerator register
 * @param reg: the specified register address
 * @param data: the multi datas
 * @param len: the length of data
 * @retval None
 */
#define BMI088_Accel_Read_Multi_Reg(reg, data, len) \
    {                                              \
        BMI088_ACCEL_NS_L();                       \
        BMI088_Read_Write_Byte((reg) | 0x80);      \
        BMI088_Read_Multi_Reg(reg, data, len);      \
        BMI088_ACCEL_NS_H();                       \
    }
/**
 * @brief macro definition of the BMI088_Gyro_Write_Single_Reg that write a single data in the specified gyro register
 * @param reg: the specified register address
 * @param data: the single data
 * @retval None
 */
#define BMI088_Gyro_Write_Single_Reg(reg, data) \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_Write_Single_Reg((reg), (data)); \
        BMI088_GYRO_NS_H();                     \
    }
/**
 * @brief macro definition of the BMI088_Gyro_Read_Single_Reg that read a single data in the specified gyro register
 * @param reg: the specified register address
 * @param data: the single data
 * @retval None
 */
#define BMI088_Gyro_Read_Single_Reg(reg, data)  \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_Read_Single_Reg((reg), &(data)); \
        BMI088_GYRO_NS_H();                     \
    }
/**
 * @brief macro definition of the BMI088_Gyro_Read_Multi_Reg that read multi datas in the specified gyro register
 * @param reg: the specified register address
 * @param data: the multi datas
 * @param len: the length of data
 * @retval None
 */
#define BMI088_Gyro_Read_Multi_Reg(reg, data, len)   \
    {                                               \
        BMI088_GYRO_NS_L();                         \
        BMI088_Read_Multi_Reg((reg), (data), (len)); \
        BMI088_GYRO_NS_H();                         \
    }

#endif

/* Private variables ---------------------------------------------------------*/

/**
  * @brief 3 times sampling frequency  Accelerator 
  */
static float BMI088_ACCEL_SEN = BMI088_ACCEL_3G_SEN;  

/**
  * @brief 2000 byte length gyro
  */
static float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;  

/**
  * @brief BMI088 Information structure
  */
BMI088_Info_Typedef BMI088_Info;

/**
  * @brief BMI088 Accelerator configuration data and Error Status
  */
static uint8_t Accel_Register_ConfigurationData_ErrorStatus[BMI088_WRITE_ACCEL_REG_NUM][3] =
{
    /* Turn on accelerometer */
    {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},   

    /* Pause mode */
    {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR}, 

    /* Acceleration Configuration */
    {BMI088_ACC_CONF,  BMI088_ACC_NORMAL| BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR}, 

    /* Accelerometer setting range */ 
    {BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G, BMI088_ACC_RANGE_ERROR},  

    /* INT1 Configuration input and output pin */ 
    {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR}, 

    /* interrupt map pin */
    {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}  
};

/**
  * @brief BMI088 Gyro configuration data and Error Status
  */
static uint8_t Gyro_Register_ConfigurationData_ErrorStatus[BMI088_WRITE_GYRO_REG_NUM][3] =
{
    /* Angular rate and resolution */
    {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR}, 

    /* Data Transfer Rate and Bandwidth Settings */
    {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR}, 

    /* Power Mode Selection Register */
    {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},   

    /* Data Interrupt Trigger Register */
    {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},   

    /* Interrupt Pin Trigger Register */
    {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},  

    /* interrupt map register */
    {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}   
};





#if defined(BMI088_USE_SPI)
/**
  * @brief Write the single register value to the sensor
  * @param reg: the specified register address
  * @param data: the specified register value
  * @retval none
  */
static void BMI088_Write_Single_Reg(uint8_t reg, uint8_t data)
{
    /* write the address to the sensor */
    BMI088_Read_Write_Byte(reg);  

    /* write the register value to the sensor */
    BMI088_Read_Write_Byte(data);
}
/**
  * @brief Read the single register value to the sensor
  * @param reg: the specified register address
  * @param return_data: pointer to the specified register value
  * @retval none
  */
static void BMI088_Read_Single_Reg(uint8_t reg, uint8_t *return_data)
{
    /**
     * @brief As mentioned in the manual, in the gyroscope mode, 
     *        write 0x80 to the register to trigger a new data interrupt, 
     *        and the other modes are the same. 
     */
    BMI088_Read_Write_Byte(reg | 0x80); 

    /* write/read the register value to the sensor */
    *return_data = BMI088_Read_Write_Byte(0x55);
}

/**
  * @brief Read the multi register value to the sensor
  * @param reg: the specified register address
  * @param buf: pointer to the specified register value
  * @param len: the length of specified register value
  * @retval none
  */
static void BMI088_Read_Multi_Reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    /* trigger a new data interrupt */
    BMI088_Read_Write_Byte(reg | 0x80);

    while (len != 0)
    {
        /* write/read the register value to the sensor */
        *buf = BMI088_Read_Write_Byte(0x55); 
        buf++;
        len--;
    }
}
#endif

