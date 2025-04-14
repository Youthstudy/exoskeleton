#include "cansend.h"
#include "control.h"

CAN_TxPacketTypeDef TxHeader[MOTOR];



int float_to_uint(float x, float x_min, float x_max, int bits)
{
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
		/// converts unsigned int to float, given range and number of bits ///
		float span = x_max - x_min;
		float offset = x_min;
		return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

float fmaxf(float x, float y){
	/// Returns maximum of x, y ///
	return (((x)>(y))?(x):(y));
}

float fminf(float x, float y){
    /// Returns minimum of x, y ///
    return (((x)<(y))?(x):(y));
}

		
/// 0: [position[15-8]]
/// 1: [position[7-0]] 
/// 2: [velocity[11-4]]
/// 3: [velocity[3-0], kp[11-8]]
/// 4: [kp[7-0]]
/// 5: [kd[11-4]] 
/// 6: [kd[3-0], torque[11-8]]
/// 7: [torque[7-0]]
void pack_cmd(CAN_TxPacketTypeDef* msg, joint_control joint){
     
     /// limit data to be within bounds ///
	float p_des = fminf(fmaxf(P_MIN, joint.p_des + joint.p_init), P_MAX);                    
	float v_des = fminf(fmaxf(V_MIN, joint.v_des), V_MAX);//将角速度限制在有效的范围内
	float kp = fminf(fmaxf(KP_MIN, joint.kp), KP_MAX);
	float kd = fminf(fmaxf(KD_MIN, joint.kd), KD_MAX);
	float t_ff = fminf(fmaxf(T_MIN, joint.t_ff), T_MAX);
	
	/// convert floats to unsigned ints ///
	uint16_t p_cmd = float_to_uint(p_des, P_MIN, P_MAX, 16);            
	uint16_t v_cmd = float_to_uint(v_des, V_MIN, V_MAX, 12);
	uint16_t kp_cmd = float_to_uint(kp, KP_MIN, KP_MAX, 12);
	uint16_t kd_cmd = float_to_uint(kd, KD_MIN, KD_MAX, 12);
	uint16_t t_cmd = float_to_uint(t_ff, T_MIN, T_MAX, 12);
	/// pack ints into the can buffer ///

	msg->Data[0] = p_cmd>>8;
	msg->Data[1] = p_cmd&0xFF;
	msg->Data[2] = v_cmd>>4;
	msg->Data[3] = ((v_cmd&0xF)<<4)|(kp_cmd>>8);
	msg->Data[4] = kp_cmd&0xFF;
	msg->Data[5] = kd_cmd>>4;

	msg->Data[6] = 0x00|(t_cmd>>8);
	msg->Data[7] = t_cmd&0xFF;
}


/// CAN Reply Packet Structure ///
/// 16 bit position, between -4*pi and 4*pi
/// 12 bit velocity, between -30 and + 30 rad/s
/// 12 bit current, between -40 and 40;
/// CAN Packet is 5 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
/// 0: [position[15-8]]
/// 1: [position[7-0]] 
/// 2: [velocity[11-4]]
/// 3: [velocity[3-0], current[11-8]]
/// 4: [current[7-0]]
		 

void unpack_reply(float ret[3],CAN_RxPacketTypeDef *msg)
{

 /// unpack ints from can buffer ///

 uint16_t p_int = (msg->Data[1]<<8)|(msg->Data[2]&0XFF);
 uint16_t v_int = (msg->Data[3]<<4)|(msg->Data[4]>>4);
 uint16_t i_int = ((msg->Data[4]&0xF)<<8)|msg->Data[5];
 /// convert uints to floats ///
 float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
 float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
 float t = uint_to_float(i_int, -T_MAX, T_MAX, 12);
	ret[0] = p;
	ret[1] = v;
	ret[2] = t;
	
}

void CAN1_Send_Msg(CAN_TxPacketTypeDef *TxMessage, uint8_t id)
{
	TxMessage->hdr.StdId = id;
	TxMessage->hdr.ExtId = 0x00;	 // 设置扩展标示符（29位）
  TxMessage->hdr.IDE=0;		  // 使用扩展标识符
  TxMessage->hdr.RTR=0;		  // 消息类型为数据帧，一帧8位
  TxMessage->hdr.DLC = 8;		
	HAL_CAN_AddTxMessage(&hcan1, &TxMessage->hdr, TxMessage->Data, &TxMessage->mailbox) ;	
}


void CAN_TxheaderInit(CAN_TxHeaderTypeDef *hdr, uint8_t id,uint8_t len){
	hdr->StdId = id;	 // 标准标识符为0
  hdr->ExtId = 0x00;	 // 设置扩展标示符（29位）
  hdr->IDE=0;		  // 使用扩展标识符
  hdr->RTR=0;		  // 消息类型为数据帧，一帧8位
  hdr->DLC = len;							 // 发送8个字节
}


void EnterMotorMode(CAN_TxPacketTypeDef *TxMessage,uint8_t id)
{
	//  CanTxMsg TxMessage;

	TxMessage->hdr.StdId = id;
	TxMessage->hdr.ExtId = 0x00;	 // 设置扩展标示符（29位）
  TxMessage->hdr.IDE=0;		  // 使用扩展标识符
  TxMessage->hdr.RTR=0;		  // 消息类型为数据帧，一帧8位
  TxMessage->hdr.DLC = 8;		
	uint8_t qidong[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
	HAL_CAN_AddTxMessage(&hcan1, &TxMessage->hdr, qidong, &TxMessage->mailbox) ;

}



