#include "calculate.h"
#include "stdio.h"
#include <stdlib.h>
#include "RouteQ.h"
#include "usart.h"

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
	uint8_t bytedata;
	if(!Cqueue_pop(&ds->q,&bytedata)){
		return ;
	}
	
	switch(ds->step)
	{
		case 0: //��ͷ
			if(bytedata == 0x3A)
			{
				ds->step = 1;
				ds->cnt = 0;
				ds->buff[ds->cnt++] = bytedata;
			}
			break;
		case 1:	//��λ��β
			if(bytedata == 0x0D && ds->cnt == DATA_FRAME_LEN - 2)
			{
				ds->step = 2;
				ds->buff[ds->cnt++] = bytedata;
			}else if(ds->cnt < DATA_FRAME_LEN)
			{
				ds->buff[ds->cnt++] = bytedata;
			}else{
				ds->cnt = ds->step = 0;
			}
			break;
		case 2: //��λ��β
			if(bytedata == 0x0A && ds->cnt == DATA_FRAME_LEN - 1)
			{
				ds->step = 3;
				ds->buff[ds->cnt++] = bytedata;
			}else if(ds->cnt < DATA_FRAME_LEN)
			{
				ds->buff[ds->cnt++] = bytedata;
			}else{
				ds->cnt = ds->step = 0;
			}
			break;
		case 3:  //ת�Ʋ�����buff���ݣ�����cnt,buff,step
			if(Data_Check(ds->buff) == 1 ){
				HAL_UART_Transmit(&huart6,ds->buff,sizeof(ds->buff),0xffff);
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
	else if(bytedata == 0X2C){ // 处理逗号分隔 1,2,1,/r/n
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
	float mod = 1.0f;
	int index = 11;
	for(int i = 2; i < NEED_DATA; i ++){
		int tmp = i - 2;
		if(tmp < 3){
			mod = 1000.0;
		}else if(3 <= tmp && tmp < 7){
			mod = 10000.0;
		}else if(7 <= tmp && tmp < 10){
			mod = 1000.0;
		}
		truth_data[i] = ((short)(real_data[index] | (real_data[index + 1]<<8))) / mod;
		index += 2;
	}
//	truth_data[2] = ((short)(real_data[11] | (real_data[12]<<8))) / 10000.0;	//w
//	truth_data[3] = ((short)(real_data[13] | (real_data[14]<<8))) / 10000.0;	//x
//	truth_data[4] = ((short)(real_data[15] | (real_data[16]<<8))) / 10000.0;	//y
//	truth_data[5] = ((short)(real_data[17] | (real_data[18]<<8))) / 10000.0;	//w
//	truth_data[6] = ((short)(real_data[13] | (real_data[14]<<8))) / 10000.0;	//x
//	truth_data[7] = ((short)(real_data[15] | (real_data[16]<<8))) / 10000.0;	//y
//	truth_data[8] = ((short)(real_data[17] | (real_data[18]<<8))) / 10000.0;	//z
//	truth_data[9] = ((short)(real_data[19] | (real_data[20]<<8))) / 1000.0;	//acc_x
//	truth_data[10] = ((short)(real_data[21] | (real_data[22]<<8))) / 1000.0; //y
//	truth_data[11] = ((short)(real_data[23] | (real_data[24]<<8))) / 1000.0; //z
}

int Data_Check(uint8_t *buff)
{
	int flag = 0; //0��У��δͨ�� 1��У��ͨ��
	uint16_t crc = buff[DATA_FRAME_LEN-4] | (buff[DATA_FRAME_LEN-3]<<8);
	uint16_t sum_crc = 0x00;
	for(int i = 1; i < DATA_FRAME_LEN - 4; i++) //У����ƣ������ͷ����β��У��λ
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
	int i = 5;
	float norm = sqrt(Td[i] * Td[i] +Td[i+1] * Td[i+1]  + Td[i+2] * Td[i+2]  + Td[i+3] * Td[i+3] );
	float w = Td[i]/ norm;
	float x = Td[i+1]/ norm;
	float y = Td[i+2]/ norm;
	float z = Td[i+3]/ norm;

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
		angle[i] = acosf(Rm[i]);
	}
}

