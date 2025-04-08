#ifndef __MY_TASK_H__
#define __MY_TASK_H__

#include "stm32f4xx_it.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "stdio.h"
#include "string.h"



void usart_send_str(UART_HandleTypeDef *huart, char *string);
void Task1Function(void *parm);
void Task2Function(void *parm);


#endif

