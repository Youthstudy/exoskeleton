#include "impandance_control.h"
#include "control.h"

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
}

void update_ImpandanceController(ImpandanceController* ctrl, joint_control* joint){
	ctrl->current_pos = joint->filt_res[0];
	ctrl->current_acc = (joint->filt_res[1] - ctrl->current_v)*ctrl->frequence;
	ctrl->current_v = joint->filt_res[1];
	ctrl->current_twist = joint->filt_res[2];
}

void Impedance_Set(ImpandanceController* ctrl,T M, T B,T K){
	ctrl->M_ = M;
	ctrl->B_ = B;
	ctrl->K_ = K;
}

void ImpedanceCtrl_Compute(ImpandanceController* ctrl, joint_control* joint, T pos_desire, T v_desire){
	T pos_error = pos_desire - ctrl->current_pos;
	T v_error = v_desire - ctrl->current_v;
	
	ctrl->forces_cmd = ctrl->M_ * ctrl->current_acc + ctrl->B_ * v_error + ctrl->K_ * pos_error;
}

void ImpedanceCtrl_Run(ImpandanceController* ctrl, joint_control* joint, T pos_end, T v_desire){
	update_ImpandanceController(ctrl,joint);
	ImpedanceCtrl_Compute(ctrl,joint,pos_end,v_desire);
	Impedance2joint(ctrl,joint);
}

void Impedance2joint(ImpandanceController* ctrl, joint_control* joint){
		joint_set(joint,0,0,ctrl->forces_cmd,50,30);
}
