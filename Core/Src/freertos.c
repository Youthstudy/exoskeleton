/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "my_task.h"
#include "TCPsend.h"
#include "control.h"
#include "admittance_control.h"
#include "can.h"
#include "cansend.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAXT 10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId controlTaskHandle;
/* USER CODE END Variables */
osThreadId SendPCHandle;
osThreadId ACTaskHandle;
osThreadId ScheduleHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void SendPCTask(void const * argument);
void ACTask02(void const * argument);
void scheduleTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of SendPC */
  osThreadDef(SendPC, SendPCTask, osPriorityNormal, 0, 256);
  SendPCHandle = osThreadCreate(osThread(SendPC), NULL);

  /* definition and creation of ACTask */
  osThreadDef(ACTask, ACTask02, osPriorityNormal, 0, 256);
  ACTaskHandle = osThreadCreate(osThread(ACTask), NULL);

  /* definition and creation of Schedule */
  osThreadDef(Schedule, scheduleTask, osPriorityNormal, 0, 256);
  ScheduleHandle = osThreadCreate(osThread(Schedule), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	// osThreadDef(Acontrol, vControlOutputTask, osPriorityNormal, 0, 128);
	// controlTaskHandle = osThreadCreate(osThread(Acontrol), NULL);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_SendPCTask */
/**
  * @brief  Function implementing the SendPC thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_SendPCTask */
void SendPCTask(void const * argument)
{
  /* USER CODE BEGIN SendPCTask */
	const TickType_t xFrequency = pdMS_TO_TICKS(5); // ??100ms
	TickType_t xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
		uint8_t buffer[128] = {0};
		uint8_t n = 0;
		n = joint_pack(&joint[0], buffer, n);
    n = joint_pack(&joint[1], buffer, n);
		sendData(buffer,n);
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
  /* USER CODE END SendPCTask */
}

/* USER CODE BEGIN Header_ACTask02 */
/**
* @brief Function implementing the ACTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ACTask02 */
void ACTask02(void const * argument)
{
  /* USER CODE BEGIN ACTask02 */
	const TickType_t xFrequency = pdMS_TO_TICKS(1); // 1000ms
	TickType_t xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
		for(int i = 0; i < MOTOR; i++){
			int flag = i == 0? 1:-1;
			update_AdmittanceController(&ACtrl[i],&joint[i]);
			Admittance_Compute(&ACtrl[i],joint[i].filt_res[2] * flag,0.01,0);
		}
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
  /* USER CODE END ACTask02 */
}

/* USER CODE BEGIN Header_scheduleTask */
/**
* @brief Function implementing the Schedule thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_scheduleTask */
void scheduleTask(void const * argument)
{
  /* USER CODE BEGIN scheduleTask */
	const TickType_t xFrequency = pdMS_TO_TICKS(1); // 1000ms
	TickType_t xLastWakeTime = xTaskGetTickCount();
	int cnt = 0;
  /* Infinite loop */
  for(;;)
  {
		int n[2] = {7, 7}; // 0~ 10

    for(int i = 0; i < MOTOR; i++){
				if(cnt >= n[i] && cnt <= MAXT){ 
					Admittance2joint(&ACtrl[i],&joint[i]);
				}else if(cnt < n[i]){
					joint_set(&joint[i],0,0,0,10,5);
				}
    }

    for(int i = 0; i < MOTOR; i++)
      {
        pack_cmd(joint[i].senddata, joint[i]);
      }
    CAN_Send_Msg(&hcan1,joint[0].senddata,1);
    CAN_Send_Msg(&hcan2,joint[1].senddata,1);
		cnt = cnt > MAXT ? 0 : cnt + 1;
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
  /* USER CODE END scheduleTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
