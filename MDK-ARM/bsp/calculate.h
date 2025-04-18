#ifndef __CALCULATE_H_
#define __CALCULATE_H_

#include "math.h"
#include "cansend.h"
#include <stdio.h>
#include "string.h"
#include "control.h"

#define BUFFER_LEN 1
#define DATA_FRAME_LEN 29
#define CLASS_DATA 128
#define NEED_DATA 9

typedef struct
{
  uint8_t RecieveBuffer[BUFFER_LEN];
  uint8_t Data_Buffer[CLASS_DATA][DATA_FRAME_LEN];
  uint8_t Class_Cnt;
  uint8_t step;
  uint8_t cnt,buff[DATA_FRAME_LEN];
  float Truth_Data[CLASS_DATA][NEED_DATA];
} Data_HandleTypedef;


extern Data_HandleTypedef ImuData[4];

int Data_Check(uint8_t *buff);
void Data_Process(uint8_t *real_data,float *truth_data);
void RecieveData_Init(Data_HandleTypedef *recievedata);
int Data_Check(uint8_t *buff);
void Receive(Data_HandleTypedef *data_store);
void Receive_pc_debug(joint_control *joint,motor_parameter_typedef *mp);

#endif


