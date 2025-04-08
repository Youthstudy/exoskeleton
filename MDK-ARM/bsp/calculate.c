#include "calculate.h"

Data_HandleTypedef ImuData[4];


void RecieveData_Init(Data_HandleTypedef *recievedata)
{
	memset(recievedata->RecieveBuffer,0,sizeof(recievedata->RecieveBuffer));
	memset(recievedata->Data_Buffer,0,sizeof(recievedata->Data_Buffer));
	recievedata->Class_Cnt = 0;
	memset(recievedata->Truth_Data,0,sizeof(recievedata->Truth_Data));
	recievedata->step = recievedata->cnt = 0;
	memset(recievedata->buff,0,sizeof(recievedata->buff));
}

void Receive(Data_HandleTypedef *data_store)
{
	uint8_t bytedata;
	bytedata = data_store->RecieveBuffer[0];
	
	switch(data_store->step)
	{
		case 0: //包头
			if(bytedata == 0x3A)
			{
				data_store->step++;
				data_store->cnt = 0;
				data_store->buff[data_store->cnt++] = bytedata;
			}
			break;
		case 1:	//低位包尾
			if(bytedata == 0x0D)
			{
				data_store->step++;
				data_store->buff[data_store->cnt++] = bytedata;
			}else
			{
				data_store->buff[data_store->cnt++] = bytedata;
			}
			break;
		case 2: //高位包尾
			if(bytedata == 0x0A)
			{
				data_store->step ++;
				data_store->buff[data_store->cnt++] = bytedata;
			}else
			{
				data_store->step = 1;
				data_store->buff[data_store->cnt++] = bytedata;
			}
			break;
		case 3:  //转移并处理buff数据，重置cnt,buff,step
			if(Data_Check(data_store->buff) == 1)
			{
				memcpy(data_store->Data_Buffer[data_store->Class_Cnt],data_store->buff,sizeof(data_store->Data_Buffer[data_store->Class_Cnt]));
				Data_Process(data_store->Data_Buffer[data_store->Class_Cnt],data_store->Truth_Data[data_store->Class_Cnt]);

				data_store->Class_Cnt++;
				if(data_store->Class_Cnt == CLASS_DATA)
				{
					data_store->Class_Cnt = 0;
				}
			}
			memset(data_store->buff,0,sizeof(data_store->buff));
			data_store->cnt = data_store->step = 0;
			break;
		default:data_store->step=0;break;
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




