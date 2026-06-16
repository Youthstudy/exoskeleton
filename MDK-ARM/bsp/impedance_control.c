#include "impandance_control.h"
#include "control.h"
#include "string.h"

ImpandanceController Ictrl[2];

void ImpedanceCtrl_Init(ImpandanceController* ctrl, float M, float B, float K,float flag,int frequence)
{
	ctrl->M_ = M;
	ctrl->B_ = B;
	ctrl->K_ = K;
	ctrl->frequence = frequence;
	ctrl->flag = flag;
}

void Impedance_Set(ImpandanceController* ctrl, float pos,float K, float B){
	ctrl->pos_cmd = pos;
	ctrl->B_ = B;
	ctrl->K_ = K;
}

void Impedance2joint(ImpandanceController* ctrl, joint_control* joint){
	joint_set(joint,ctrl->pos_cmd * ctrl->flag ,0 , 0 ,ctrl->K_, ctrl->B_);
}

void Impedance_update(ImpandanceController* ctrl,float pos,float v){
	ctrl->x = pos;
	ctrl->v = v;
}

void Impedance_pc_set(ImpandanceController* ctrl,motor_parameter_typedef *mp){
	ctrl->B_ = mp->buff[1]== 0 ? ctrl->B_: mp->buff[1];
	ctrl->K_ = mp->buff[2]== 0 ? ctrl->K_: mp->buff[2];
	ctrl->enable = mp->buff[5] == 0 ? ctrl->enable : mp->buff[5];
	memset(mp->buff,0,sizeof(mp->buff));
//	joint->t_ff =	mp->buff[2];
//	joint->kp = mp->buff[3];
//	joint->kd = mp->buff[4];
}
