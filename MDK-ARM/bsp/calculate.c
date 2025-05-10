#include "calculate.h"
#include "stdio.h"
#include <stdlib.h>
#include "RouteQ.h"

Data_HandleTypedef ImuData[4];

void DataInit_init(DataInit_HandleTypedf* d){
	d->i = 0;
	memset(d->sum,0,sizeof(d->sum));
}

void RecieveData_Init(Data_HandleTypedef *recievedata)
{
	memset(recievedata->RecieveBuffer,0,sizeof(recievedata->RecieveBuffer));
	recievedata->Class_Cnt = 0;
	recievedata->step = recievedata->cnt = 0;
	
	memset(recievedata->Truth_Data,0,sizeof(recievedata->Truth_Data));
	memset(recievedata->buff,0,sizeof(recievedata->buff));
	memset(recievedata->Rotationmatrix,0,sizeof(recievedata->Rotationmatrix));
	memset(recievedata->angle,0,sizeof(recievedata->angle));
	memset(recievedata->angle_init,0,sizeof(recievedata->angle_init));
	DataInit_init(&recievedata->init);
	
	Cqueue_Init(&recievedata->q);
}

void Receive(Data_HandleTypedef *ds)
{
	if(Cqueue_empty(&ds->q)){
		return ;
	}
	uint8_t bytedata;
	bytedata = Cqueue_head(&ds->q);
	Cqueue_pop(&ds->q);
	
	switch(ds->step)
	{
		case 0: //包头
			if(bytedata == 0x3A)
			{
				ds->step++;
				ds->cnt = 0;
				ds->buff[ds->cnt++] = bytedata;
			}
			break;
		case 1:	//低位包尾
			if(bytedata == 0x0D)
			{
				ds->step++;
				ds->buff[ds->cnt++] = bytedata;
			}else
			{
				ds->buff[ds->cnt++] = bytedata;
			}
			break;
		case 2: //高位包尾
			if(bytedata == 0x0A)
			{
				ds->step ++;
				ds->buff[ds->cnt++] = bytedata;
			}else
			{
				ds->step = 1;
				ds->buff[ds->cnt++] = bytedata;
			}
			break;
		case 3:  //转移并处理buff数据，重置cnt,buff,step
			if(Data_Check(ds->buff) == 1)
			{
				Data_Process(ds->buff,ds->Truth_Data);
				quat2Rotation(ds->Truth_Data, ds->Rotationmatrix,ds->angle,ds->angle_init);
			}
			ds->cnt = ds->step = 0;
			break;
		default:ds->step=0;break;
	}
}


void Receive_pc_debug(joint_control *joint, motor_parameter_typedef *mp){
	uint8_t bytedata;
	bytedata = mp->RecieveBuffer[0];
	if(bytedata == 0X0A){
		
		mp->step = mp->cnt = 0;
		memset(mp->str,0,sizeof(mp->str));
	}
	else if(bytedata == 0X2C){
		mp->cnt = 0;
		mp->buff[mp->step ++] = (float)atof(mp->str);
		if(mp->step > PC_RECIEVE_LEN){
			mp->step = 0;
		}
		memset(mp->str,0,sizeof(mp->str));
	}
	else{
		mp->str[(mp->cnt ++) % PC_BUFFER] = bytedata;
	}
}
 
void Data_Process(uint8_t *real_data,float *truth_data)
{

	truth_data[0] = (short)(real_data[1] | (real_data[2] << 8));
	truth_data[1] = (double)(real_data[7] | (real_data[8] << 8) | (real_data[9] << 16) | (real_data[10] << 24)) / 400.0 ;
	truth_data[2] = ((short)(real_data[11] | (real_data[12]<<8))) / 10000.0;	//w
	truth_data[3] = ((short)(real_data[13] | (real_data[14]<<8))) / 10000.0;	//x
	truth_data[4] = ((short)(real_data[15] | (real_data[16]<<8))) / 10000.0;	//y
	truth_data[5] = ((short)(real_data[17] | (real_data[18]<<8))) / 10000.0;	//z
	truth_data[6] = ((short)(real_data[19] | (real_data[20]<<8))) / 1000.0;	//acc_x
	truth_data[7] = ((short)(real_data[21] | (real_data[22]<<8))) / 1000.0; //y
	truth_data[8] = ((short)(real_data[23] | (real_data[24]<<8))) / 1000.0; //z
}

int Data_Check(uint8_t *buff)
{
	int flag = 0; //0：校验未通过 1：校验通过
	uint16_t crc = buff[DATA_FRAME_LEN-4] | (buff[DATA_FRAME_LEN-3]<<8);
	uint16_t sum_crc = 0x00;
	for(int i = 1; i < DATA_FRAME_LEN - 4; i++) //校验机制：不算包头，包尾和校验位
	{
		sum_crc += buff[i];
	}
	if(sum_crc == crc)
	{
		flag = 1;
	}
	return flag;
}

void get_angle_init(float* init_matix,float* matrix,DataInit_HandleTypedf* D_i){
	if(D_i->i < STARTWINDOW){
		for(int j = 0; j < sizeof(D_i->sum); j ++){
			D_i->sum[j] += matrix[j];
		}
		D_i->i ++;
	}else if(D_i->i == STARTWINDOW){
		for(int j = 0; j < sizeof(D_i->sum); j ++){
			init_matix[j] = D_i->sum[j] / STARTWINDOW;
		}
		D_i->i ++;
	}
}

void quat2Rotation(float* Td, float* Rm,float* angle, float* angle_init){
	float norm = sqrt(Td[2] * Td[2] +Td[3] * Td[3]  + Td[4] * Td[4]  + Td[5] * Td[5] );
	float w = Td[2]/ norm;
	float x = Td[3]/ norm;
	float y = Td[4]/ norm;
	float z = Td[5]/ norm;

	float xx = x * x;
	float xy = x * y;
	float xz = x * z;
	float xw = x * w;

	float yy = y * y;
	float yz = y * z;
	float yw = y * w;

	float zz = z * z;
	float zw = z * w;

	Rm[0] = 1 - 2 * (yy + zz);
	Rm[1] = 2 * (xy - zw);
	Rm[2] = 2 * (xz + yw);

	Rm[3]  = 2 * (xy + zw);
	Rm[4]  = 1 - 2 * (xx + zz);
	Rm[5] = 2 * (yz - xw);

	Rm[6] = 2 * (xz - yw);
	Rm[7] = 2 * (yz + xw);
	Rm[8] = 1 - 2 * (xx + yy);
	
	for(int i= 0; i < 9; i++){
		angle[i] = acosf(Rm[i]) - angle_init[i];
	}
}

