#include "control.h"
#include <string.h>

joint_control joint[2];
motor_parameter_typedef motor_parameter;

void joint_init(joint_control *joint){
	joint->kp = 15;
	joint->kd = 10;
	joint->p_des = 0;
	joint->p_init = 0;
	memset(joint->ret,0,sizeof(joint->ret));
	joint->t_ff = 0;
	joint->v_des = 0;
	joint->status = 0;
	joint->moveflag = 0;
	for(int i = 0 ; i < 3; i++){
	FILT_init(&joint->filt[i]);
	Kalman_Init(&joint->kfilter[i],0.02,0.001,0.5);
	}
}

void motor_parameter_init(motor_parameter_typedef *mp){
	mp->p_des = 0.0;
	mp->t_ff = 0.0;
	mp->v_des = 0.0;
	mp->kd = 0.0;
	mp->kp = 0.0;
	mp->Force_time = -1;
	memset(mp->buff,1,sizeof(mp->buff));
}


void joint_set(joint_control *joint,float p_des, float v_des, float t_ff, float kp, float kd){
	joint->p_des = p_des;
	joint->v_des = v_des;
	joint->t_ff = t_ff;
	joint->kp = kp;
	joint->kd = kd;
}
 
void joint_pc_set(joint_control *joint,motor_parameter_typedef *mp){
	joint->p_des = mp->buff[0];
	joint->v_des = mp->buff[1];
	joint->t_ff =	mp->buff[2];
	joint->kp = mp->buff[3];
	joint->kd = mp->buff[4];
}




