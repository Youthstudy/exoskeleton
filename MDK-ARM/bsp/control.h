#ifndef __CONTROL_H_
#define __CONTROL_H_

#include "pid.h"
#include "math.h"


#define PC_DEBUG_ENABLE 1

typedef struct
{
	float p_des, v_des, kp, kd, t_ff;
	float ret[3];
	PID_TypeDef pid_t;
	float p_init;
}joint_control;

typedef struct
{
	uint8_t RecieveBuffer[1];
  uint8_t step;
  uint8_t cnt;
	char str[30];
	float buff[5];
  float p_des, v_des, kp, kd, t_ff;
} motor_parameter_typedef;

// struct
extern motor_parameter_typedef motor_parameter;
extern joint_control joint[2];


// function
void joint_init(joint_control *joint);

void motor_parameter_init(motor_parameter_typedef *mp);

void joint_set(joint_control *joint,float p_des, float v_des, float t_ff, float kp, float kd);
void joint_pc_set(joint_control *joint, motor_parameter_typedef *parameter);

#endif


