#ifndef __CANSEND_H__
#define __CANSEND_H__

#include "can.h"
#include "stm32f4xx_hal_can.h"
#include "control.h"

#define MOTOR 2  // 电机数量

#define PI 3.1415926f
 #define P_MIN 0.0f   //0
#define P_MAX 4*PI
#define V_MIN -30.0f   //30f
#define V_MAX 30.0f
#define KP_MIN 0.0f    //0-500
#define KP_MAX 500.0f
#define KD_MIN 0.0f     //0-100
#define KD_MAX 100.0f
#define T_MIN -48.0f    //-18   18
#define T_MAX 48.0f

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

extern CAN_TxPacketTypeDef TxHeader[MOTOR];

int float_to_uint(float x, float x_min, float x_max, int bits);
double uint_to_double(int x_int, float x_min, float x_max, int bits);
float fmaxf(float x, float y);
float fminf(float x, float y);
void pack_cmd(CAN_TxPacketTypeDef* msg, joint_control joint);
void unpack_reply(float ret[3],CAN_RxPacketTypeDef *msg);
void CAN_TxheaderInit(CAN_TxHeaderTypeDef *hdr, uint8_t id,uint8_t len);
void CAN1_Send_Msg(CAN_TxPacketTypeDef *TxMessage, uint8_t id);
void EnterMotorMode(CAN_TxPacketTypeDef *TxMessage,uint8_t id);

#endif

