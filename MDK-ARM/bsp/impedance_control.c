#include "impandance_control.h"
#include "control.h"
#include "string.h"

ImpandanceController Ictrl[2];

void ImpedanceCtrl_Init(ImpandanceController* ctrl,
                        T M, T B,T K,
                        T pos_init,
                        int frequence)
{
	ctrl->M_ = M;
	ctrl->B_ = B;
	ctrl->K_ = K;
	ctrl->frequence = frequence;
	ctrl->pos_init = pos_init;
	ctrl->enable = 1;
}

void update_ImpandanceController(ImpandanceController* ctrl, joint_control* joint){
	ctrl->current_pos = joint->filt_res[0];
	ctrl->current_acc = joint->filt_res[2];
	ctrl->current_v = joint->filt_res[1];
	ctrl->current_twist = joint->filt_res[2];
}

void Impedance_Set(ImpandanceController* ctrl,T M, T B,T K){
	ctrl->M_ = M;
	ctrl->B_ = B;
	ctrl->K_ = K;
}

void ImpedanceCtrl_Compute(ImpandanceController* ctrl, joint_control* joint, T pos_desire){
	T pos_error = pos_desire  - ctrl->current_pos;
	
	ctrl->forces_cmd = ctrl->B_ * ctrl->current_v + ctrl->K_ * pos_error - ctrl->current_twist;

}

void ImpedanceCtrl_Run(ImpandanceController* ctrl, joint_control* joint,T pos_desire){
	if(ctrl->enable == 0 || ctrl->time_end > MAX_T){
	return ;
	}
	if(ctrl->time_end > MAX_T){
		ctrl->enable = 0;
	return ;
	}
	ctrl->time_end += 1/1000.0;
	update_ImpandanceController(ctrl,joint);
	ImpedanceCtrl_Compute(ctrl,joint,pos_desire);
	printf("%f,%f\r\n",ctrl->forces_cmd,pos_desire);
}

void Impedance2joint(ImpandanceController* ctrl, joint_control* joint){
		joint_set(joint,joint->filt_res[0],0,ctrl->forces_cmd,10,5);
}

T tractory_posset(T x, T pos_init){
	return (4.75f * x * x - 5.08f * x + pos_init);
}

T tractory_Vset(T x){
	return (2.0f * 4.75f * x  - 5.08f);
}


void Impedance_pc_set(ImpandanceController* ctrl,motor_parameter_typedef *mp){
	ctrl->M_ = mp->buff[0] == 0? ctrl->M_: mp->buff[0];
	ctrl->B_ = mp->buff[1]== 0? ctrl->B_: mp->buff[1];
	ctrl->K_ = mp->buff[2]== 0? ctrl->K_: mp->buff[2];
	
	ctrl->time_end = mp->buff[3] == 0?  ctrl->time_end: mp->buff[3];
	mp->Force_time = mp->buff[4] == 0? mp->Force_time : mp->buff[4];
	ctrl->enable = mp->buff[5] == 0? ctrl->enable : mp->buff[5];
	memset(mp->buff,0,sizeof(mp->buff));
//	joint->t_ff =	mp->buff[2];
//	joint->kp = mp->buff[3];
//	joint->kd = mp->buff[4];
}
