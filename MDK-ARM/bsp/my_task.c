#include "FreeRTOS.h"
#include "task.h"
#include "my_task.h"
#include "projdefs.h"

#include "admittance_control.h"
#include "calculate.h"


void vControlOutputTask(void const * argument){
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t xFrequency = pdMS_TO_TICKS(1000/CONTROL_FRE);
	while(1){
		for(int i = 0; i < MOTOR; i++){
			float Force = ExternalForce_Set(&ACtrl[i],ImuData[0].angle[2]);
			Admittance_Run(&ACtrl[i],&joint[i],Force);
		}
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}


