
#include "TCPsend.h"
#include <string.h>
#include <stdio.h>
#include "stdbool.h"
#include "main.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "crc.h"
#include "control.h"

uint32_t crc32_cal(uint32_t *data, uint32_t len){
	return HAL_CRC_Calculate(&hcrc, data, len);
}

void UART_SendByte(uint8_t b){
	HAL_UART_Transmit(&huart6, (uint8_t *)&b, 1,1);
	return ;
}


void sendData(uint8_t* data,uint8_t len) {
    UART_SendByte(0x3A);   

    UART_SendByte(len);
    for(int i=0;i<len;i++) UART_SendByte(data[i]);
    UART_SendByte(0x0D); 
    UART_SendByte(0x0A);
}


int joint_pack(const joint_control *joint, uint8_t *buffer, int offset)
{
    
    memcpy(buffer + offset, &joint->ret[0], sizeof(float));
    offset += sizeof(float);
    memcpy(buffer + offset, &joint->ret[1], sizeof(float));
    offset += sizeof(float);
    memcpy(buffer + offset, &joint->ret[2], sizeof(float));
    offset += sizeof(float);
    memcpy(buffer + offset, &joint->p_des, sizeof(float));
    offset += sizeof(float);
    memcpy(buffer + offset, &joint->v_des, sizeof(float));
    offset += sizeof(float);
    memcpy(buffer + offset, &joint->kp, sizeof(float));
    offset += sizeof(float);
    memcpy(buffer + offset, &joint->kd, sizeof(float));
    offset += sizeof(float);
    memcpy(buffer + offset, &joint->pos_cmd, sizeof(float));
    offset += sizeof(float);
		memcpy(buffer + offset, &joint->moveflag, sizeof(float));
    offset += sizeof(float);
	return offset;
    
}
