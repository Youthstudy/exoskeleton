#ifndef __CONTROL_H_
#define __CONTROL_H_

#include "pid.h"
#include "math.h"


typedef struct
{
 float p_des, v_des, kp, kd, t_ff;
	float ret[3];
	PID_TypeDef pid_t;
	float p_init;
}joint_control;


extern joint_control joint[1];

void joint_init(joint_control *joint);

void joint_set(joint_control *joint,float p_des, float v_des, float t_ff, float kp, float kd);


#endif


