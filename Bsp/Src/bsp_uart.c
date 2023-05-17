/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_uart.c
  * @brief          : bsp uart functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : none
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "bsp_uart.h"
#include "usart.h"

uint8_t SBUS_RX_Buf[2][36];


/**
  * @brief  Starts the multi_buffer DMA Transfer with interrupt enabled.
  */
static void USART_RxDMA_MultiBufferStart(UART_HandleTypeDef *huart, uint32_t SrcAddress, uint32_t DstAddress, uint32_t SecondMemAddress, uint32_t DataLength);


/**
  * @brief  Configures the USART.
  * @param  None
  * @retval None
  */
void BSP_USART_Init(void)
{
	USART_RxDMA_MultiBufferStart(&huart3,huart3.Instance->DR,(uint32_t)SBUS_RX_Buf[0],(uint32_t)SBUS_RX_Buf[1],36);
}

/**
  * @brief  Starts the multi_buffer DMA Transfer with interrupt enabled.
  * @param  huart       pointer to a UART_HandleTypeDef structure that contains
  *                     the configuration information for the specified USART Stream.  
  * @param  SrcAddress The source memory Buffer address
  * @param  DstAddress The destination memory Buffer address
  * @param  SecondMemAddress The second memory Buffer address in case of multi buffer Transfer  
  * @param  DataLength The length of data to be transferred from source to destination
  * @retval none
  */
static void USART_RxDMA_MultiBufferStart(UART_HandleTypeDef *huart, uint32_t SrcAddress, uint32_t DstAddress, uint32_t SecondMemAddress, uint32_t DataLength)
{
	/* configuare the receptionType TOIDLE */
	huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
	
	/* configuare the receive size */
	huart->RxXferSize = DataLength;
	
  /* Enable the DMA transfer for the receiver request */
  SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

  /* Enalbe IDLE interrupt */
  __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

  /* Disable DMA */
  do{
      __HAL_DMA_DISABLE(huart->hdmarx);
  }while(huart->hdmarx->Instance->CR & DMA_SxCR_EN);

  /* Configure the source memory Buffer address  */
  huart->hdmarx->Instance->PAR = SrcAddress;

  /* Configure the destination memory Buffer address */
  huart->hdmarx->Instance->M0AR = DstAddress;

  /* Configure DMA Stream destination address */
  huart->hdmarx->Instance->M1AR = SecondMemAddress;

  /* Configure the length of data to be transferred from source to destination */
  huart->hdmarx->Instance->NDTR = DataLength;

  /* Enable double memory buffer */
  SET_BIT(huart->hdmarx->Instance->CR, DMA_SxCR_DBM);

  /* Enable DMA */
  __HAL_DMA_ENABLE(huart->hdmarx);
}

/**
  * @brief  Reception Event Callback (Rx event notification called after use of advanced reception service).
  * @param  huart UART handle
  * @param  Size  Number of data available in application reception buffer (indicates a position in
  *               reception buffer until which, data are available)
  * @retval None
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	/* configuare the receptionType TOIDLE */
	huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
	
  /* Enable the DMA transfer for the receiver request */
  SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
	
  /* Enalbe IDLE interrupt */
  __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

	if(huart == &huart3)
	{
		
	}
}

