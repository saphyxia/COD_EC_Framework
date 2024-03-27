/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : BMI088.c
  * Description        : Implementation of communication with BMI088
  ******************************************************************************
  * @author         : YuanBin Yan
  * @date           : 2024/02/23
  * @version        : 1.2.2
  * @attention      : 1. fix bmi088 initialize status refresh error
  *
  * Copyright 2024 COD USTL.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "bmi088.h"
#include "bsp_timebase.h"
#include "bsp_tim.h"
#include "spi.h"
#include "main.h"

/* Private function ----------------------------------------------------------*/
/**
  * @brief Clear the BMI088_ACCEL_NS
  * @note CS1_ACCEL_GPIO_Port: GPIOA
  * @note CS1_ACCEL_Pin: GPIO_PIN_4
  */
static void BMI088_ACCEL_NS_L(void)
{
  HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port,CS1_ACCEL_Pin,GPIO_PIN_RESET);
}
//------------------------------------------------------------------------------

/**
  * @brief Set the BMI088_ACCEL_NS
  * @note CS1_ACCEL_GPIO_Port: GPIOA
  * @note CS1_ACCEL_Pin: GPIO_PIN_4
  */
static void BMI088_ACCEL_NS_H(void)
{
  HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port,CS1_ACCEL_Pin,GPIO_PIN_SET);
}
//------------------------------------------------------------------------------

/**
  * @brief Clear the BMI088_GYRO_NS
  * @note CS1_GYRO_GPIO_Port: GPIOB
  * @note CS1_GYRO_Pin: GPIO_PIN_0
  */
static void BMI088_GYRO_NS_L(void)
{
  HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port,CS1_GYRO_Pin,GPIO_PIN_RESET);
}
//------------------------------------------------------------------------------

/**
  * @brief Set the BMI088_GYRO_NS
  * @note CS1_GYRO_GPIO_Port: GPIOB
  * @note CS1_GYRO_Pin: GPIO_PIN_0
  */
static void BMI088_GYRO_NS_H(void)
{
  HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port,CS1_GYRO_Pin,GPIO_PIN_SET);
}
//------------------------------------------------------------------------------

/* Private define ------------------------------------------------------------*/
#if defined(BMI088_USE_SPI)

/**
  * @brief Transmit and Receive an amount of data in blocking mode. 
  * @param txdata: transmission data
  * @retval reception data
  */
static uint8_t BMI088_Read_Write_Byte(uint8_t txdata)
{
  uint8_t rxdata = 0;

  HAL_SPI_TransmitReceive(&hspi1,&txdata,&rxdata,1,1000);

  return rxdata;
}
//------------------------------------------------------------------------------

/**
  * @brief Transmit a data to specified address.
  * @param reg: specified register address
  * @param data: register value
  * @retval none
  */
static void BMI088_Write_Single_Reg(uint8_t reg, uint8_t data)
{
  /* Transmit the register address */
  BMI088_Read_Write_Byte(reg);  

  /* Transmit the register value */
  BMI088_Read_Write_Byte(data);
}
//------------------------------------------------------------------------------

/**
  * @brief Receive a register value from specified address
  * @param reg: specified register address
  * @param ret: pointer to reception values
  * @retval none
  */
static void BMI088_Read_Single_Reg(uint8_t reg, uint8_t *ret)
{
  /**
   * @brief In gyroscope mode, 
   *        write 0x80 to the register to trigger a new data interrupt, 
   *        and the other modes are the same. 
   */
  BMI088_Read_Write_Byte(reg | 0x80); 

  /* write/read the register value to the sensor */
  *ret = BMI088_Read_Write_Byte(0x55);
}
//------------------------------------------------------------------------------

/**
  * @brief Receive an amount of register value from specified address
  * @param reg: specified register address
  * @param buf: pointer to reception values
  * @param len: length of reception values
  * @retval none
  */
static void BMI088_Read_Multi_Reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
  /* trigger a new data interrupt */
  BMI088_Read_Write_Byte(reg | 0x80);

  while (len != 0)
  {
    /* receive the register value */
    *buf = BMI088_Read_Write_Byte(0x55); 
    buf++;
    len--;
  }
}
//------------------------------------------------------------------------------


/**
 * @brief Transmit a value to specified accelerator register address
 * @param reg: register address
 * @param data: register value
 */
#define BMI088_Accel_Write_Single_Reg(reg, data) \
    {                                            \
      BMI088_ACCEL_NS_L();                       \
      BMI088_Write_Single_Reg((reg), (data));    \
      BMI088_ACCEL_NS_H();                       \
    }
//------------------------------------------------------------------------------

/**
 * @brief Receive a value from a specified accelerator register address
 * @param reg: register address
 * @param data: received value
 */
#define BMI088_Accel_Read_Single_Reg(reg, data) \
    {                                           \
      BMI088_ACCEL_NS_L();                      \
      BMI088_Read_Write_Byte((reg) | 0x80);     \
      BMI088_Read_Write_Byte(0x55);             \
      (data) = BMI088_Read_Write_Byte(0x55);    \
      BMI088_ACCEL_NS_H();                      \
    }
//------------------------------------------------------------------------------

/**
 * @brief Receive values from a specified accelerator register address
 * @param reg: register address
 * @param data: pointer to received values buf
 * @param len: length of values
 */
#define BMI088_Accel_Read_Multi_Reg(reg, data, len) \
    {                                               \
      BMI088_ACCEL_NS_L();                          \
      BMI088_Read_Write_Byte((reg) | 0x80);         \
      BMI088_Read_Multi_Reg(reg, data, len);        \
      BMI088_ACCEL_NS_H();                          \
    }
//------------------------------------------------------------------------------


/**
 * @brief Transmit a value to specified gyro register address
 * @param reg: register address
 * @param data: register value
 */
#define BMI088_Gyro_Write_Single_Reg(reg, data) \
    {                                           \
      BMI088_GYRO_NS_L();                       \
      BMI088_Write_Single_Reg((reg), (data));   \
      BMI088_GYRO_NS_H();                       \
    }
//------------------------------------------------------------------------------


/**
 * @brief Receive a value from specified gyro register address
 * @param reg: register address
 * @param data: received value
 */
#define BMI088_Gyro_Read_Single_Reg(reg, data)  \
    {                                           \
      BMI088_GYRO_NS_L();                       \
      BMI088_Read_Single_Reg((reg), &(data));   \
      BMI088_GYRO_NS_H();                       \
    }
//------------------------------------------------------------------------------

/**
 * @brief Receive values from a specified gyro register address
 * @param reg: register address
 * @param data: pointer to received values buf
 * @param len: length of values
 */
#define BMI088_Gyro_Read_Multi_Reg(reg, data, len)   \
    {                                                \
      BMI088_GYRO_NS_L();                            \
      BMI088_Read_Multi_Reg((reg), (data), (len));   \
      BMI088_GYRO_NS_H();                            \
    }
//------------------------------------------------------------------------------

#endif

/**
  * @brief 6 times sampling frequency 
  */
static float BMI088_ACCEL_SEN = BMI088_ACCEL_6G_SEN;  

/**
  * @brief 2000 bytes length
  */
static float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;

/**
  * @brief Accelerator configuration infomations
  */
static uint8_t Accel_Register_ConfigInfo[BMI088_WRITE_ACCEL_REG_NUM][3] =
{
  /* Turn on accelerometer */
  {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},   

  /* Pause mode */
  {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR}, 

  /* Configuration value */
  {BMI088_ACC_CONF,  (BMI088_ACC_NORMAL| BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set), BMI088_ACC_CONF_ERROR}, 

  /* Accelerometer setting range */ 
  {BMI088_ACC_RANGE, BMI088_ACC_RANGE_6G, BMI088_ACC_RANGE_ERROR},  

  /* INT1 Configuration input and output pin */ 
  {BMI088_INT1_IO_CTRL, (BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW), BMI088_INT1_IO_CTRL_ERROR}, 

  /* interrupt map pin */
  {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}  
};

/**
  * @brief Gyro configuration infomations
  */
static uint8_t Gyro_Register_ConfigInfo[BMI088_WRITE_GYRO_REG_NUM][3] =
{
  /* Angular rate and resolution */
  {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR}, 

  /* Data Transfer Rate and Bandwidth Settings */
  {BMI088_GYRO_BANDWIDTH, (BMI088_GYRO_2000_230_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set), BMI088_GYRO_BANDWIDTH_ERROR}, 

  /* Power Mode */
  {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},   

  /* Data Interrupt Trigger */
  {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},   

  /* Interrupt Pin Trigger */
  {BMI088_GYRO_INT3_INT4_IO_CONF, (BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW), BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},  

  /* interrupt map */
  {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}   
};

/**
  * @brief Initializes the accelerator of bmi088.
  * @param None
  * @retval None
  */

static BMI088_Status_e BMI088_Accel_Init(void)
{
  uint8_t res = 0;

  /* read the accelerator ID */
  BMI088_Accel_Read_Single_Reg(BMI088_ACC_CHIP_ID, res);
  /* delay 150us */
  Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
  /* read again */
  BMI088_Accel_Read_Single_Reg(BMI088_ACC_CHIP_ID, res);
  /* delay 150us */
  Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

  /* software reset */
  BMI088_Accel_Write_Single_Reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE); 
  /* delay 80ms */
  Delay_ms(BMI088_LONG_DELAY_TIME);

  /* read the accelerator ID again ------------------------------------------------*/
  BMI088_Accel_Read_Single_Reg(BMI088_ACC_CHIP_ID, res);
  Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
  BMI088_Accel_Read_Single_Reg(BMI088_ACC_CHIP_ID, res);
  Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

  /* check software reset */
  if (res != BMI088_ACC_CHIP_ID_VALUE)
  {
      return BMI088_NO_SENSOR;
  }

  /* config the accelerator */
  for (uint8_t write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++)
  {
    /* Write the configuration values in the internal configuration register: */
    /*!< [0][0]  BMI088_ACC_PWR_CTRL 0x7D                accelerator address */
    /*!< [0][1]  BMI088_ACC_ENABLE_ACC_ON 0x04           Turn on the accelerator */
    /*!< [1][0]  BMI088_ACC_PWR_CONF 0x7C                accelerator mode address */
    /*!< [1][1]  BMI088_ACC_PWR_ACTIVE_MODE 0x00         power start  */
    /*!< [2][0]  BMI088_ACC_CONF 0x40                    config address */
    /*!< [2][1]  BMI088_ACC_CONF_DATA 0xAB               BMI088_ACC_NORMAL (0x2 << BMI088_ACC_BWP_SHFITS): normal sampling frequency  */
    /*!<                                                 | BMI088_ACC_800_HZ (0xB << BMI088_ACC_ODR_SHFITS): 800hz output frequency */  
    /*!<                                                 | BMI088_ACC_CONF_MUST_Set 0x80 */
    /*!< [3][0]  BMI088_ACC_RANGE 0x41                   scoping register address */
    /*!< [3][1]  BMI088_ACC_RANGE_3G (0x0 << BMI088_ACC_RANGE_SHFITS)   +-3g */
    /*!< [4][0]  BMI088_INT1_IO_CTRL 0x53                INT1 configure address */
    /*!< [4][1]  BMI088_INT1_IO_CTRL_DATA 0x8            BMI088_ACC_INT1_IO_ENABLE (0x1 << BMI088_ACC_INT1_IO_ENABLE_SHFITS): configure INT1 as output pins */ 
    /*!<                                                 | BMI088_ACC_INT1_GPIO_PP (0x0 << BMI088_ACC_INT1_GPIO_MODE_SHFITS): push-pull output */  
    /*!<                                                 | BMI088_ACC_INT1_GPIO_LOW (0x0 << BMI088_ACC_INT1_GPIO_LVL_SHFITS): pull down */
    /*!< [5][0]  BMI088_INT_MAP_DATA 0x58                interrupts mapping address */
    /*!< [5][1]  BMI088_ACC_INT1_DRDY_INTERRUPT (0x1 << BMI088_ACC_INT1_DRDY_INTERRUPT_SHFITS)  interrupts are mapped to INT1 */
    BMI088_Accel_Write_Single_Reg(Accel_Register_ConfigInfo[write_reg_num][0], Accel_Register_ConfigInfo[write_reg_num][1]); 
    /* delay 150us */
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    /* read the configuration */
    BMI088_Accel_Read_Single_Reg(Accel_Register_ConfigInfo[write_reg_num][0], res);
    /* delay 150us */
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    /* check the configuration */
    if (res != Accel_Register_ConfigInfo[write_reg_num][1])
    {
        return (BMI088_Status_e)Accel_Register_ConfigInfo[write_reg_num][2];
    }
  }

  /* no error */
  return BMI088_NO_ERROR;  
}
//------------------------------------------------------------------------------

/**
  * @brief Initializes the gyro.
  * @param None
  * @retval None
  */

static BMI088_Status_e BMI088_Gyro_Init(void)
{
  uint8_t res = 0;

  /* read the gyro ID */
  BMI088_Gyro_Read_Single_Reg(BMI088_GYRO_CHIP_ID, res);
  /* delay 150us */
  Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
  /* read again */
  BMI088_Gyro_Read_Single_Reg(BMI088_GYRO_CHIP_ID, res);
  /* delay 150us */
  Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

  /* software reset */
  BMI088_Gyro_Write_Single_Reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
  /* delay 80ms */
  Delay_ms(BMI088_LONG_DELAY_TIME);

  /* read the gyro ID again ------------------------------------------------*/
  BMI088_Gyro_Read_Single_Reg(BMI088_GYRO_CHIP_ID, res);
  Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
  BMI088_Gyro_Read_Single_Reg(BMI088_GYRO_CHIP_ID, res);
  Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

  /* check the software reset */
  if (res != BMI088_GYRO_CHIP_ID_VALUE)
  {
      return BMI088_NO_SENSOR;
  }

  /* config the gyro */
  for (uint8_t write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++)
  {
      /* Write the configuration values in the internal configuration registers: */
      /*!< [0][0]  BMI088_GYRO_RANGE 0x0F                   angular rate range and resolution address */
      /*!< [0][1]  BMI088_GYRO_2000 (0x0 << BMI088_GYRO_RANGE_SHFITS)  //+-2000°/s */
      /*!< [1][0]  BMI088_GYRO_BANDWIDTH 0x10               bandwidth and output rate address */
      /*!< [1][1]  BMI088_GYRO_2000_532_HZ                  set data transmission rate to 2kHZ, bandwidth to 532hz */
      /*!< [2][0]  BMI088_GYRO_LPM1 0x11                    power mode selection address */
      /*!< [2][1]  BMI088_GYRO_NORMAL_MODE 0x00             normal mode */
      /*!< [3][0]  BMI088_GYRO_CTRL 0x15                    data interrupt trigger address */
      /*!< [3][1]  BMI088_DRDY_ON 0x80                      allow new data to trigger the interrupt */
      /*!< [4][0]  BMI088_GYRO_INT3_INT4_IO_CONF 0x16       interrupt pin configuration address */
      /*!< [4][1]  BMI088_GYRO_INT3_INT4_IO_CONF_DATA 0x0   BMI088_GYRO_INT3_GPIO_PP (0x0 << BMI088_GYRO_INT3_GPIO_MODE_SHFITS): INT3 push-pull output  */
      /*!<                                                  | BMI088_GYRO_INT3_GPIO_LOW (0x0 << BMI088_GYRO_INT3_GPIO_LVL_SHFITS): INT3 pull down  */
      /*!< [5][0]  BMI088_GYRO_INT3_INT4_IO_MAP 0x18        interrupt map address */
      /*!< [5][1]  BMI088_GYRO_DRDY_IO_INT3 0x01            mapping to INT3 */
      BMI088_Gyro_Write_Single_Reg(Gyro_Register_ConfigInfo[write_reg_num][0], Gyro_Register_ConfigInfo[write_reg_num][1]);
      /* delay 150us */
      Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

      /* read the configuration */
      BMI088_Gyro_Read_Single_Reg(Gyro_Register_ConfigInfo[write_reg_num][0], res);
      /* delay 150us */
      Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

      /* check the configuration */
      if (res != Gyro_Register_ConfigInfo[write_reg_num][1])
      {
          return (BMI088_Status_e)Gyro_Register_ConfigInfo[write_reg_num][2];
      }
  }

  /* no error */
  return BMI088_NO_ERROR;
}
//------------------------------------------------------------------------------

/**
  * @brief Update the BMI088 offsets.
  * @param BMI088_Info: pointer to BMI088_Info_Typedef structure that
  *         contains the information of the BMI088.
  * @retval None
  */
static void BMI088_Offset_Update(BMI088_Info_Typedef *BMI088_Info)
{
#if IMU_Calibration_ENABLE /* ENABLE the BMI088 Calibration */

  uint8_t buf[8] = {0,};

  for(uint16_t i = 0; i < 5000; i++)
  {
    /* receive the accelerator values */
    BMI088_Accel_Read_Multi_Reg(BMI088_ACCEL_XOUT_L, buf, 6);
    BMI088_Info->mpu_info.accelx = (int16_t)((buf[1]) << 8) | buf[0];
    BMI088_Info->mpu_info.accely = (int16_t)((buf[3]) << 8) | buf[2];
    BMI088_Info->mpu_info.accelz = (int16_t)((buf[5]) << 8) | buf[4];

    /* receive the gyro values */
    BMI088_Gyro_Read_Multi_Reg(BMI088_GYRO_CHIP_ID, buf, 8);
    /* check the ID */
    if(buf[0] == BMI088_GYRO_CHIP_ID_VALUE)   
    {
      BMI088_Info->mpu_info.gyrox = (int16_t)((buf[3]) << 8) | buf[2];
      BMI088_Info->mpu_info.gyroy = (int16_t)((buf[5]) << 8) | buf[4];
      BMI088_Info->mpu_info.gyroz = (int16_t)((buf[7]) << 8) | buf[6];

      /* update the gyro offsets */
      BMI088_Info->offset_gyrox += BMI088_GYRO_SEN * BMI088_Info->mpu_info.gyrox;
      BMI088_Info->offset_gyroy += BMI088_GYRO_SEN * BMI088_Info->mpu_info.gyroy;
      BMI088_Info->offset_gyroz += BMI088_GYRO_SEN * BMI088_Info->mpu_info.gyroz;
    }
    /* delay 1ms */
    Delay_ms(1);
  }

  BMI088_Info->offset_gyrox = BMI088_Info->offset_gyrox / 5000.f;
  BMI088_Info->offset_gyroy = BMI088_Info->offset_gyroy / 5000.f; 
  BMI088_Info->offset_gyroz = BMI088_Info->offset_gyroz / 5000.f;

#else /* DISABLE the BMI088 Calibration */
	/* store the offsets value */
  BMI088_Info->offset_gyrox = 0.f;
  BMI088_Info->offset_gyroy = 0.f;
  BMI088_Info->offset_gyroz = 0.f;
#endif

  /* set the offset init flag */
  BMI088_Info->offsets_init = true;
}
//------------------------------------------------------------------------------

/**
 * 
  * @brief Initializes the BMI088.
  * @param None
  */
void BMI088_Init(void)
{
	BMI088_Status_e status = BMI088_NO_ERROR;

  /* Initializes the BMI088 */
  do
  {
		status = BMI088_NO_ERROR;
    /* initialize the accelerator */
    status |= BMI088_Accel_Init();
    /* initialize the gyro  */
    status |= BMI088_Gyro_Init();
    /* delay 2ms */
    Delay_ms(2);
  }while(status);
}
//------------------------------------------------------------------------------

/**
  * @brief Update the BMI088 Informations.
  * @param BMI088_Info: pointer to BMI088_Info_Typedef structure that
  *         contains the informations of the BMI088.
  * @retval None
  */
void BMI088_Info_Update(BMI088_Info_Typedef *BMI088_Info)
{
  uint8_t buf[8] = {0,};

  if(false == BMI088_Info->offsets_init)
  {
    /* update the bmi088 offset */
    BMI088_Offset_Update(BMI088_Info);
  }

  /* receive the accelerator values */
  BMI088_Accel_Read_Multi_Reg(BMI088_ACCEL_XOUT_L, buf, 6);
  BMI088_Info->mpu_info.accelx = (int16_t)((buf[1] << 8) | buf[0]);
  BMI088_Info->mpu_info.accely = (int16_t)((buf[3] << 8) | buf[2]);
  BMI088_Info->mpu_info.accelz = (int16_t)((buf[5] << 8) | buf[4]);

  /* convert the accelerator values */
  BMI088_Info->accel[0] = BMI088_ACCEL_SEN * BMI088_Info->mpu_info.accelx;
  BMI088_Info->accel[1] = BMI088_ACCEL_SEN * BMI088_Info->mpu_info.accely;
  BMI088_Info->accel[2] = BMI088_ACCEL_SEN * BMI088_Info->mpu_info.accelz;

  /* receive the temperature value */
  BMI088_Accel_Read_Multi_Reg(BMI088_TEMP_M, buf, 2);
  BMI088_Info->mpu_info.temperature = (int16_t)((buf[0] << 3) | (buf[1] >> 5));
  if (BMI088_Info->mpu_info.temperature > 1023) BMI088_Info->mpu_info.temperature -= 2048;

  /* convert the temperature value */
  BMI088_Info->temperature = BMI088_Info->mpu_info.temperature * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;

  /* receive the gyro values */
  BMI088_Gyro_Read_Multi_Reg(BMI088_GYRO_CHIP_ID, buf, 8);
  /* check the ID */
  if(BMI088_GYRO_CHIP_ID_VALUE == buf[0])   
  {
    BMI088_Info->mpu_info.gyrox = (int16_t)((buf[3] << 8) | buf[2]);
    BMI088_Info->mpu_info.gyroy = (int16_t)((buf[5] << 8) | buf[4]);
    BMI088_Info->mpu_info.gyroz = (int16_t)((buf[7] << 8) | buf[6]);
  }

  /* convert the gyro values */
  BMI088_Info->gyro[0] = BMI088_GYRO_SEN * BMI088_Info->mpu_info.gyrox - BMI088_Info->offset_gyrox;
  BMI088_Info->gyro[1] = BMI088_GYRO_SEN * BMI088_Info->mpu_info.gyroy - BMI088_Info->offset_gyroy;
  BMI088_Info->gyro[2] = BMI088_GYRO_SEN * BMI088_Info->mpu_info.gyroz - BMI088_Info->offset_gyroz;
}
//------------------------------------------------------------------------------

