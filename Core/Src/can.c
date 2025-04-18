/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
CAN_RxPacketTypeDef packet;
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
    {
      Error_Handler();
    }
  /* USER CODE BEGIN CAN1_Init 2 */
  my_can_filter_init_recv_all(&hcan1);



  if(HAL_CAN_Start(&hcan1) != HAL_OK)
    {
      Error_Handler();
    }

  /* USER CODE END CAN1_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
    {
      /* USER CODE BEGIN CAN1_MspInit 0 */

      /* USER CODE END CAN1_MspInit 0 */
      /* CAN1 clock enable */
      __HAL_RCC_CAN1_CLK_ENABLE();

      __HAL_RCC_GPIOD_CLK_ENABLE();
      /**CAN1 GPIO Configuration
      PD0     ------> CAN1_RX
      PD1     ------> CAN1_TX
      */
      GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
      HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

      /* CAN1 interrupt Init */
      HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
      HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
      /* USER CODE BEGIN CAN1_MspInit 1 */

      /* USER CODE END CAN1_MspInit 1 */
    }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
    {
      /* USER CODE BEGIN CAN1_MspDeInit 0 */

      /* USER CODE END CAN1_MspDeInit 0 */
      /* Peripheral clock disable */
      __HAL_RCC_CAN1_CLK_DISABLE();

      /**CAN1 GPIO Configuration
      PD0     ------> CAN1_RX
      PD1     ------> CAN1_TX
      */
      HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);

      /* CAN1 interrupt Deinit */
      HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
      /* USER CODE BEGIN CAN1_MspDeInit 1 */

      /* USER CODE END CAN1_MspDeInit 1 */
    }
}

/* USER CODE BEGIN 1 */

void my_can_filter_init_recv_all(CAN_HandleTypeDef* _hcan)
{
  //can1 &can2 use same filter config
  CAN_FilterTypeDef		CAN_FilterConfigStructure;

  CAN_FilterConfigStructure.FilterBank = 0;
  CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;
  CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;
  CAN_FilterConfigStructure.FilterIdHigh = 0x0000;
  CAN_FilterConfigStructure.FilterIdLow = 0x0000;
  CAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;
  CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;
  CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FilterFIFO0;
  CAN_FilterConfigStructure.SlaveStartFilterBank = 14;//can1(0-13)?can2(14-27)???????filter
  CAN_FilterConfigStructure.FilterActivation = ENABLE;

  if(HAL_CAN_ConfigFilter(_hcan, &CAN_FilterConfigStructure) != HAL_OK)
    {
      //err_deadloop(); //show error!
    }

}

int i = 0;
float sum = 0;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *canHandle)
{
  if (canHandle->Instance == hcan1.Instance)
    {
      if (HAL_CAN_GetRxMessage(canHandle, CAN_RX_FIFO0, &packet.hdr, packet.Data) == HAL_OK)
        {
          uint8_t id = packet.Data[0];
          switch (id)
            {
            case 1:
              unpack_reply(joint[0].ret,&packet);
              if(i < 50)
                {
                  i++;
                  sum += joint[0].ret[0];
                }
              else if(i == 50)
                {
                  joint[0].p_init = sum / 50.0f;
                }
              break;
            case 2:

              break;
            default:
              break;

            }

        }
      HAL_CAN_ActivateNotification(canHandle, CAN_IT_RX_FIFO0_MSG_PENDING);
    }
}


/* USER CODE END 1 */
