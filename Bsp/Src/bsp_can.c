/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : bsp_can.c
  * Description        : Implementation of can communication
  ******************************************************************************
  * @author         : YuanBin Yan
  * @date           : 2024/02/23
  * @version        : 1.2.2
  * @attention      : none
  *
  * Copyright 2024 COD USTL.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "bsp_can.h"
#include "can.h"

/* Private function prototypes -----------------------------------------------*/

/**
 * @brief structure contains the CAN received message.
 */
static CAN_RxHeaderTypeDef USER_CAN_RxInstance;
/**
 * @brief array stored the CAN Receive message.
 */
static uint8_t USER_CAN_RxFrameData[8];

/**
 * @brief structure contains the CAN1 transmit message.
 */
static CAN_TxHeaderTypeDef CAN1_FrameTxInstance={
	.DLC = 0x08,
    .IDE = CAN_ID_STD,
    .RTR = CAN_RTR_DATA,
};

/**
 * @brief structure contains the CAN2 transmit message.
 */
static CAN_TxHeaderTypeDef CAN2_FrameTxInstance={
	.DLC = 0x08,
    .IDE = CAN_ID_STD,
    .RTR = CAN_RTR_DATA,
};

/**
  * @brief  Initialize the CAN Reception Filter.
  * @param  None
  * @retval None
  */
void BSP_CAN_Init(void)
{
  CAN_FilterTypeDef CAN_FilterConfig = {0};

  /* Update the CAN1 filter Conifguration */
  CAN_FilterConfig.FilterActivation = ENABLE;
  CAN_FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  CAN_FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  CAN_FilterConfig.FilterIdHigh = 0x0000;
  CAN_FilterConfig.FilterIdLow = 0x0000;
  CAN_FilterConfig.FilterMaskIdHigh = 0x0000;
  CAN_FilterConfig.FilterMaskIdLow = 0x0000;
  CAN_FilterConfig.FilterBank = 0;
  CAN_FilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  CAN_FilterConfig.SlaveStartFilterBank = 0;

  /* configures the CAN1 Reception filter */
  if(HAL_CAN_ConfigFilter(&hcan1, &CAN_FilterConfig) != HAL_OK)
  {	
      Error_Handler();
  }

  /* Start the CAN1 module. */
  HAL_CAN_Start(&hcan1);

  /* Enable CAN1 FIFO0 interrupts */
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  /* Store the CAN2 filter Conifguration */
  CAN_FilterConfig.FilterBank = 14;
  CAN_FilterConfig.SlaveStartFilterBank = 14;

  /* configures the CAN2 Reception filter */
  if(HAL_CAN_ConfigFilter(&hcan2, &CAN_FilterConfig) != HAL_OK)
  {	
      Error_Handler();
  }

  /* Start the CAN2 module. */
  HAL_CAN_Start(&hcan2);

  /* Enable CAN2 FIFO0 interrupts */
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}
//------------------------------------------------------------------------------

/**
  * @brief  USER function to transmit message.
  * @param  data: point to the transmit message
  * @retval None
  */
void USER_CAN1_TxMessage(uint32_t StdId,uint8_t *data,uint8_t length)
{
  uint32_t TxMailbox = 0;
  /* Add a message to the first free Tx mailbox and activate the corresponding transmission request. */
  CAN1_FrameTxInstance.StdId = StdId;
  CAN1_FrameTxInstance.DLC = length;
  HAL_CAN_AddTxMessage(&hcan1, &CAN1_FrameTxInstance, data, &TxMailbox);
}
//------------------------------------------------------------------------------

/**
  * @brief  USER function to transmit message.
  * @param  data: point to the transmit message
  * @retval None
  */
void USER_CAN2_TxMessage(uint32_t StdId,uint8_t *data,uint8_t length)
{
  uint32_t TxMailbox = 0;
  /* Add a message to the first free Tx mailbox and activate the corresponding transmission request. */
  CAN2_FrameTxInstance.StdId = StdId;
  CAN2_FrameTxInstance.DLC = length;
  HAL_CAN_AddTxMessage(&hcan2, &CAN2_FrameTxInstance, data, &TxMailbox);
}
//------------------------------------------------------------------------------

/**
  * @brief  USER function to converting the CAN1 received message.
	* @param  Instance: pointer to the CAN Register base address
	* @param  StdId: Specifies the standard identifier.
	* @param  data: array that contains the received massage.
  * @retval None
  */
static void CAN1_RxFifo0RxHandler(uint32_t *StdId,uint8_t data[8])
{

}
//------------------------------------------------------------------------------

/**
  * @brief  USER function to converting the CAN2 received message.
	* @param  Instance: pointer to the CAN Register base address
	* @param  StdId: Specifies the standard identifier.
	* @param  data: array that contains the received massage.
  * @retval None
  */
static void CAN2_RxFifo0RxHandler(uint32_t *StdId,uint8_t data[8])
{

}
//------------------------------------------------------------------------------

/**
  * @brief  Rx FIFO 0 message pending callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  /* Get an CAN frame from the Rx FIFO zone into the message RAM. */
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &USER_CAN_RxInstance, USER_CAN_RxFrameData);

  /* judge the instance of receive frame data */
  if(hcan->Instance == CAN1)
  {
    CAN1_RxFifo0RxHandler(&USER_CAN_RxInstance.StdId,USER_CAN_RxFrameData);
  }
  else if(hcan->Instance == CAN2)
  {
    CAN2_RxFifo0RxHandler(&USER_CAN_RxInstance.StdId,USER_CAN_RxFrameData);
  }
}
//------------------------------------------------------------------------------
