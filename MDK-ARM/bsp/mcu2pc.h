#ifndef __MCU2PC_H__
#define __MCU2PC_H__

#include "usart.h"
#include "calculate.h"
#include "control.h"


#define TRANMIT_LEN 20

typedef struct{
	uint8_t head;
	uint8_t tail1,tail2;
	float data[TRANMIT_LEN];
	float len;
	uint8_t txData[(TRANMIT_LEN + 1)*4 + 2];
}Pack_TxHandle;




#endif

