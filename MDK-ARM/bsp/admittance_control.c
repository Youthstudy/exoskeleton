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
  ac->lastpos = 0.0f;
  ac->lastV = joint->filt_res[1];
  ac->pos_cmd = joint->filt_res[0];
  ac->velocity_cmd = 0;
  ac->delta_velocity = 0;
  ac->delta_pos = 0;
  ac->Force = 0;
  ac->AC_enable = 1;
  ac->start_flag = 1;
  FILT_init(&ac->filt);
}

void Admittance_set(AdmittanceController* ctrl, float M_a, float D_a,float K_a_)
{
  ctrl->M_a_ = M_a;
  ctrl->D_a_ = D_a;
  ctrl->K_a_ = K_a_;
}

// 更改外力
// 更改动力学模型
// 模式切换

// M_a * acc = F_ext - D_a * (x_dot - x_dot_0) - K_a * (x - x_0)
// x0 为虚拟平衡位置，默认为人体直立时位置
void Admittance_Compute(AdmittanceController* ctrl, float external_force,float x0,float xd0)
{
  float dt = ctrl->dt;
  ctrl->acc = (external_force
               - ctrl->D_a_ * (ctrl->velocity_cmd - xd0)
               - ctrl->K_a_ * (ctrl->pos_cmd - x0)) / ctrl->M_a_;
  ctrl->delta_velocity +=  ctrl->acc * dt;
  ctrl->delta_pos +=  ctrl->delta_velocity * dt;

  ctrl->pos_cmd = ctrl->pos_cmd + ctrl->delta_pos;
//		printf("%f,%f,%f\r\n",ctrl->pos_cmd,ctrl->velocity_cmd,ctrl->acc);
}

void update_AdmittanceController(AdmittanceController* ctrl, joint_control* joint)
{
  ctrl->pos_cmd = joint->filt_res[0];
  ctrl->velocity_cmd = joint->filt_res[1];
  ctrl->real_twist = joint->filt_res[2];
}

void Admittance_Run(AdmittanceController* ctrl, joint_control* joint,float external_force)
{

		update_AdmittanceController(ctrl,joint);
		Admittance_Compute(ctrl,external_force,0.01,0);
//      Admittance2joint(ctrl,joint);
}


void Admittance2joint(AdmittanceController* ctrl, joint_control* joint)
{
  joint_set(joint,ctrl->pos_cmd,0,0,10,5);
}


void Admittance_pc_set(AdmittanceController* ctrl,motor_parameter_typedef *mp, joint_control* joint)
{
  ctrl->M_a_ = mp->buff[0] == 0? ctrl->M_a_: mp->buff[0];
  ctrl->D_a_ = mp->buff[1]== 0? ctrl->D_a_: mp->buff[1];
  ctrl->K_a_ = mp->buff[2]== 0? ctrl->K_a_: mp->buff[2];
  ctrl->Force = mp->buff[3] == 0? ctrl->Force : mp->buff[3];
  mp->Force_time = mp->buff[4] == 0? mp->Force_time : mp->buff[4];
  joint->moveflag = mp->buff[5] == -1? joint->moveflag  : mp->buff[5];
  memset(mp->buff,0,sizeof(mp->buff));
//	joint->t_ff =	mp->buff[2];
//	joint->kp = mp->buff[3];
//	joint->kd = mp->buff[4];
}


