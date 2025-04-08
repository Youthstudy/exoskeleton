#include "control.h"
#include <string.h>

joint_control joint[1];


void joint_init(joint_control *joint){
	joint->kp = 0;
	joint->kd = 0;
	joint->p_des = 0;
	joint->p_init = 0;
	memset(joint->ret,0,sizeof(joint->ret));
	joint->t_ff = 0;
	joint->v_des = 0;
}

void joint_set(joint_control *joint,float p_des, float v_des, float t_ff, float kp, float kd){
	joint->p_des = p_des;
	joint->v_des = v_des;
	joint->t_ff = t_ff;
	joint->kp = kp;
	joint->kd = kd;
}
 



