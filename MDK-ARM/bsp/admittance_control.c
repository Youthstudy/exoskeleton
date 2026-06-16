#include "admittance_control.h"
#include "math.h"
#include "filt.h"
#include "string.h"

AdmittanceController ACtrl[2];

void Admittance_init(AdmittanceController* ac,joint_control* joint,float M_a,float D_a,float K_a_)
{
  ac->M_a_ = M_a;
  ac->D_a_ = D_a;
  ac->K_a_ = K_a_;
  ac->frequency = CONTROL_FRE;
	
  ac->pos_init = joint->p_init;
	
  ac->dt = 1.0f/CONTROL_FRE;
  ac->pos_cmd = 0;
  ac->velocity_cmd = 0;
  ac->delta_velocity = 0;
  ac->delta_pos = 0;
  FILT_init(&ac->filt);
	
	ac->kp = 15;
	ac->kd = 5;
}

void Admittance_set(AdmittanceController* ctrl, float M_a, float D_a,float K_a_)
{
  ctrl->M_a_ = M_a;
  ctrl->D_a_ = D_a;
  ctrl->K_a_ = K_a_;
}

void Admittance_PDset(AdmittanceController* ctrl,float p, float d){
	ctrl->kp = p;
	ctrl->kd = d;
}

// M_a * acc = F_ext - D_a * (x_dot) - K_a * (x - x_0)
// x0 ÎªÐéÄâÆ½ºâÎ»ÖÃ
void Admittance_Compute(AdmittanceController* ctrl, float external_force,float x0, float x0_dot)
{
  float dt = ctrl->dt;
	float acc = (external_force
               - ctrl->D_a_ * (ctrl->delta_velocity - x0_dot)
               - ctrl->K_a_ * (ctrl->delta_pos - x0)) / ctrl->M_a_;
  ctrl->delta_velocity += acc * dt;
  ctrl->delta_pos +=  ctrl->delta_velocity * dt;
  ctrl->pos_cmd = x0 + ctrl->delta_pos;
	ctrl->velocity_cmd = x0_dot + ctrl->delta_velocity;
}


void Admittance2joint(AdmittanceController* ctrl, joint_control* joint)
{
  joint_set(joint,ctrl->pos_cmd, 0 ,0,ctrl->kp, ctrl->kd);
}

void Admittance_pc_set(AdmittanceController* ctrl,motor_parameter_typedef *mp, joint_control* joint)
{
  ctrl->M_a_ = mp->buff[0] == 0? ctrl->M_a_: mp->buff[0];
  ctrl->D_a_ = mp->buff[1]== 0? ctrl->D_a_: mp->buff[1];
  ctrl->K_a_ = mp->buff[2]== 0? ctrl->K_a_: mp->buff[2];
  mp->Force_time = mp->buff[4] == 0? mp->Force_time : mp->buff[4];
  joint->moveflag = mp->buff[5] == -1? joint->moveflag  : mp->buff[5];
  memset(mp->buff,0,sizeof(mp->buff));
}


