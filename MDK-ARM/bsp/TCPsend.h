/**
  ************************************* Copyright ******************************
  *                 (C) Copyright 2025,lxl,China, GCU.
  *                            All Rights Reserved
  *
  *
  * FileName   : TCPsend.h
  * Version    : v1.0
  * Author     : lxl
  * Date       : 2025-08-25
  * Description:
  * Function List:
  	1. ....
  	   <version>:
  <modify staff>:
  		  <data>:
   <description>:
  	2. ...
  ******************************************************************************
 */



#ifndef __T_C_PSEND_H_
#define __T_C_PSEND_H_

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "control.h"

#define MAX_RETRY 3
#define TIMEOUT_MS 10  // 超时重发时间
#define MAX_LEN 100

// 发送帧结构
typedef struct {
    uint8_t len;
    uint8_t data[MAX_LEN];
    uint32_t checksum;
} Frame;


/**
 * @brief 将joint_control结构体中的指定字段打包到字节数组
 * @param joint 指向joint_control结构体的指针
 * @param buffer 输出缓冲区（至少32字节）
 * @return 打包的字节数（32）
 */
int joint_pack(const joint_control *joint, uint8_t *buffer, int offset);


// 模拟 UART 发送

void UART_SendByte(uint8_t b);
void sendFrame(Frame* frame) ;
void sendData(uint8_t* data,uint8_t len);
uint32_t crc32_cal(uint32_t *data, uint32_t len);
// 打包发送

extern uint32_t seqNum;
// 发送数据函数（带重发）



#endif




