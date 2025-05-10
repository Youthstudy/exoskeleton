#ifndef __CALCULATE_H_
#define __CALCULATE_H_
#include "RouteQ.h"
#include "math.h"
#include "cansend.h"
#include <stdio.h>
#include "string.h"
#include "control.h"


#define BUFFER_LEN 1
#define DATA_FRAME_LEN 29
#define CLASS_DATA 1
#define NEED_DATA 9

#define STARTWINDOW 50.0f

typedef struct{
	uint8_t i;
	float sum[9];
}DataInit_HandleTypedf;

typedef struct
{
  uint8_t RecieveBuffer[BUFFER_LEN];
  uint8_t Class_Cnt;
  uint8_t step;
  uint8_t cnt,buff[DATA_FRAME_LEN];
  float Truth_Data[NEED_DATA];
	float Rotationmatrix[9];
	float angle[9];
	float angle_init[9];
	DataInit_HandleTypedf init;
	RouteQ q;
} Data_HandleTypedef;



extern Data_HandleTypedef ImuData[4];

void DataInit_init(DataInit_HandleTypedf* d);

int Data_Check(uint8_t *buff);
void Data_Process(uint8_t *real_data,float *truth_data);
void RecieveData_Init(Data_HandleTypedef *recievedata);

void Receive(Data_HandleTypedef *data_store);
void Receive_pc_debug(joint_control *joint,motor_parameter_typedef *mp);

void get_angle_init(float* init_matix, float* matrix, DataInit_HandleTypedf* D_i);
void quat2Rotation(float* Td, float* Rm,float* angle, float* angle_init);
#endif


