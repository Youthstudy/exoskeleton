#include "impandance_control.h"
#include "control.h"

void ImpedanceCtrl_Init(ImpandanceController* ctrl,
                        T Kp, T Kd,
                        T pos_init,
                        int frequence)
{
	ctrl->Kp = Kp;
	ctrl->Kd = Kd;
	ctrl->frequence = frequence;
	ctrl->pos_init = pos_init;
}

void update_ImpandanceController(ImpandanceController* ctrl, joint_control* joint){
	ctrl->current_pos = joint->filt_res[0];
	ctrl->current_v = joint->filt_res[1];
	ctrl->current_twist = joint->filt_res[2];
}

void Impedance_Set(ImpandanceController* ctrl,T Kp, T Kd){
	ctrl->Kp = Kp;
	ctrl->Kd = Kd;
}

void ImpedanceCtrl_Compute(ImpandanceController* ctrl, joint_control* joint, T pos_end){
	
}

void ImpedanceCtrl_Run(ImpandanceController* ctrl, joint_control* joint, T pos_end){
	
}

