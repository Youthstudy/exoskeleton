#ifndef __CANSEND_H__
#define __CANSEND_H__

#include "can.h"
#include "stm32f4xx_hal_can.h"
#include "control.h"

#define MOTOR 2 

#define PI 3.1415926f
#define P_MIN -4*PI   //0
#define P_MAX 4*PI 
#define V_MIN -30.0f   //30f
#define V_MAX 30.0f
#define KP_MIN 0.0f    //0-500
#define KP_MAX 500.0f
#define KD_MIN 0.0f     //0-100
#define KD_MAX 100.0f
#define T_MIN -15.0f    //-18   18
#define T_MAX 15.0f

typedef struct
{
	uint32_t mailbox;
	CAN_TxHeaderTypeDef hdr;
	uint8_t Data[8];
}CAN_TxPacketTypeDef;

typedef struct
{
	CAN_RxHeaderTypeDef hdr;
	uint8_t Data[8];
	float ret[3];
}CAN_RxPacketTypeDef;

int float_to_uint(float x, float x_min, float x_max, int bits);
double uint_to_double(int x_int, float x_min, float x_max, int bits);
float fmaxf(float x, float y);
float fminf(float x, float y);

void pack_cmd(uint8_t* data, joint_control joint);
void unpack_reply(float ret[3],CAN_RxPacketTypeDef *msg);

void CAN_Send_Msg(CAN_HandleTypeDef *hcan,uint8_t *data, uint8_t id);
void EnterMotorMode(uint8_t id);

void ChangeMotorID(CAN_TxPacketTypeDef *TxMessage,uint8_t old_id,uint8_t new_id);
void ResetMotorID(CAN_HandleTypeDef *hcan,uint8_t id);

#endif



