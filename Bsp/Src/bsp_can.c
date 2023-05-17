/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_can.c
  * @brief          : bsp can functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : Pay attention to enable the can filter
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "bsp_can.h"
#include "can.h"


/* Private variables ---------------------------------------------------------*/
/**
 * @brief the structure that contains the Information of CAN Receive.
 */
static CAN_RxFrameTypeDef USER_CAN_RxInstance;

/**
  * @brief  Configures the CAN Filter.
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

  /* configures the CAN1 filter */
  if(HAL_CAN_ConfigFilter(&hcan1, &CAN_FilterConfig) != HAL_OK)
  {	
      Error_Handler();
  }

  /* Start the CAN1 module. */
  HAL_CAN_Start(&hcan1);

  /* Enable CAN1 FIFO0 interrupts */
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  /* Update the CAN2 filter Conifguration */
  CAN_FilterConfig.FilterBank = 14;
  CAN_FilterConfig.SlaveStartFilterBank = 14;

  /* configures the CAN2 filter */
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
  * @brief  USER function to transmit the Specifies message.
  * @param  TxMessage pointer to the structure that contains the Information of CAN transmit.
  * @retval None
  */
void USER_CAN_TxMessage(CAN_TxFrameTypeDef *TxMessage)
{
  static uint32_t TxMailbox = 0;

  /* Add a message to the first free Tx mailbox and activate the corresponding transmission request. */
	HAL_CAN_AddTxMessage(TxMessage->hcan, &TxMessage->header, TxMessage->data, &TxMailbox);
}
//------------------------------------------------------------------------------

/**
  * @brief  USER function to converting the received message.
  * @param  Instance pointer to the CAN Register base address
  * @param  StdId Specifies the standard identifier.
  * @param  data array that contains the received massage.
  * @retval None
  */
static void USER_CAN_RxMessageHandler(CAN_TypeDef *Instance,uint32_t *StdId,uint8_t data[8])
{
  if(Instance == CAN1)
  {

  }
  else if(Instance == CAN2)
  {

  }
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
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &USER_CAN_RxInstance.header, USER_CAN_RxInstance.data);
  /* update the receive data */
  USER_CAN_RxMessageHandler(USER_CAN_RxInstance.hcan->Instance,&USER_CAN_RxInstance.header.StdId,USER_CAN_RxInstance.data);
}
//------------------------------------------------------------------------------


