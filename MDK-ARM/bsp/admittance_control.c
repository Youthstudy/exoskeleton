#include "admittance_control.h"
#include "math.h"
#include "filt.h"
#include "string.h"

// 0.01 0.8318 100

AdmittanceController ACtrl[2];
 
void Admittance_init(AdmittanceController* ac,joint_control* joint,T M_a,T D_a,T K_a_){
	ac->M_a_ = M_a;
	ac->D_a_ = D_a;
	ac->K_a_ = K_a_;
	ac->frequency = CONTROL_FRE;
	ac->pos_init = joint->p_init;
	ac->dt = 1.0f/CONTROL_FRE;
	ac->lastpos = 0.0f;
	ac->lastV = joint->filt_res[1];
	ac->pos_cmd = joint->filt_res[0];
	ac->velocity_cmd = 0;
	ac->Force = 0;
	ac->time_end = -1;
	ac->AC_enable = 1;
	ac->start_flag = 1;
	FILT_init(&ac->filt);
}

void Admittance_set(AdmittanceController* ctrl, T M_a, T D_a,T K_a_){
	ctrl->M_a_ = M_a;
	ctrl->D_a_ = D_a;
	ctrl->K_a_ = K_a_;
}

void Admittance_Compute(AdmittanceController* ctrl, T external_force) {
		T dt = ctrl->dt;
		ctrl->velocity_cmd +=  ctrl->acc * dt;
		ctrl->pos_cmd +=  ctrl->velocity_cmd * dt;
    ctrl->acc = (external_force 
											- ctrl->D_a_ * ctrl->velocity_cmd
											- ctrl->K_a_ * (ctrl->pos_cmd - ctrl->lastpos)) / ctrl->M_a_;
    ctrl->lastpos = ctrl->pos_cmd;
		printf("%f,%f,%f\r\n",ctrl->pos_cmd,ctrl->velocity_cmd,ctrl->acc);
}

void update_AdmittanceController(AdmittanceController* ctrl, joint_control* joint){
	ctrl->pos_cmd = joint->filt_res[0];
	ctrl->lastpos = ctrl->pos_cmd;
	ctrl->velocity_cmd = joint->filt_res[1];
	ctrl->real_twist = joint->filt_res[2];
}

void Admittance_Run(AdmittanceController* ctrl, joint_control* joint,T external_force){
	if(ctrl->AC_enable == 1 && ctrl->time_end > 0){
		if(ctrl->start_flag == 1){
			update_AdmittanceController(ctrl,joint);
			ctrl->start_flag = 0;
		}
		Admittance_Compute(ctrl,external_force);
		ctrl->time_end -= ctrl->dt;
		Admittance2joint(ctrl,joint);

	}else{
		ctrl->start_flag = 1;
	}
}

T ExternalForce_Set(AdmittanceController* ctrl, T pos){
	T externalForce = 0.0; // N
	T lastpos = ctrl->lastpos;

	if(fabs(pos - lastpos) > F_DEADBOUND ){
		if(pos > lastpos){
			externalForce = 15.0;
		}else{
			externalForce = -15.0;
		}
	}else{
		externalForce = 0.0;
	}
	
	ctrl->lastpos = pos;
	return externalForce;
}

void Admittance2joint(AdmittanceController* ctrl, joint_control* joint){
	T pos_des = ctrl->pos_cmd;
	joint_set(joint,pos_des,0,0,50,30);
}


void Admittance_pc_set(AdmittanceController* ctrl,motor_parameter_typedef *mp){
	ctrl->M_a_ = mp->buff[0] == 0? ctrl->M_a_: mp->buff[0];
	ctrl->D_a_ = mp->buff[1]== 0? ctrl->D_a_: mp->buff[1];
	ctrl->K_a_ = mp->buff[2]== 0? ctrl->K_a_: mp->buff[2];
	ctrl->Force = mp->buff[3] == 0? ctrl->Force : mp->buff[3];
	mp->Force_time = mp->buff[4] == 0? mp->Force_time : mp->buff[4];
	ctrl->time_end = mp->buff[5] == 0? ctrl->time_end : mp->buff[5];
	memset(mp->buff,0,sizeof(mp->buff));
//	joint->t_ff =	mp->buff[2];
//	joint->kp = mp->buff[3];
//	joint->kd = mp->buff[4];
}
	
	
