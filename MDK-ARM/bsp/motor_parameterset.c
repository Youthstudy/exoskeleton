#include "motor_parameterset.h"
#include "control.h"

motor_parameter_typedef motor_parameter;

void joint_pc_set(joint_control *joint,motor_parameter_typedef *parameter){
	joint->p_des = parameter->p_des;
	joint->v_des = parameter->v_des;
	joint->t_ff = parameter->t_ff;
	joint->kp = parameter->kp;
	joint->kd = parameter->kd;
}


