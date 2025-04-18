#include "admittance_control.h"
#include "math.h"
#include "control.h"

void AController_init(AdmittanceController* ac,
										T M_a, 
										T D_a,
										int f){
	ac->M_a_ = M_a;
	ac->D_a_ = D_a;
	ac->frequency = f;
	
}
										

// ----- 

void Admittance_Compute(AdmittanceController* ctrl, T external_force) {
		T dt = 1.0f/ctrl->frequency;
    T acceleration = (external_force - ctrl->D_a_ * ctrl->current_velocity ) / ctrl->M_a_;

    ctrl->velocity_cmd = ctrl->current_velocity + acceleration * dt;
    T dpos = ctrl->current_pos + ctrl->velocity_cmd * dt;
		ctrl->pos_cmd = ctrl->current_pos + dpos;
}


void update_Controller(AdmittanceController* ctrl, joint_control* joint){
	ctrl->current_pos = joint->ret[0] - ctrl->init_pos;
	ctrl->current_velocity = joint->ret[1];
	ctrl->real_twist = joint->ret[2];
	
}

void Admittance_Run(){
	

}
	
	
	
	
	
	
