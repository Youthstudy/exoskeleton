#ifndef __ADMITTANCE_CONTROL_H__
#define  __ADMITTANCE_CONTROL_H__

#include "control.h"
#include "filt.h"

typedef float T;

#define CONTROL_FRE 1000
#define F_DEADBOUND 0.01

#define POS_CONTROL_MAX 1.2883

typedef struct {
	// -- ADMITTANCE PARAMETERS --- //
	// M_a_  Desired mass(kg)
	// D_a_ Desired damping(N.s/m)
	T M_a_, D_a_, K_a_; 
	int frequency; // hz
	T pos_init; // 初始位置 弧度
	T dt;
	T lastpos;
	T lastV;
	T time_end; // s
	T acc;
	
	int start_flag;
	
	// -- CALLBACK -- //
	T current_pos; // 弧度
	T target_pos;	// 弧度
	T current_velocity; //当前角速度 
	T real_twist;
	T Force;
	T dpos;
	
	// -- OUTPUT COMMANDS --- //
	// final arm desired velocity
	T velocity_cmd;
	T motor_desired_twist_;
	T pos_cmd;
	
	// -- FILT STRUCT --//
	FILT_HandleTypedef filt;
	
	// -- FLAG --
	int AC_enable;
	
}AdmittanceController;

extern AdmittanceController ACtrl[1];

void Admittance_init(AdmittanceController* ac,joint_control* joint,T M_a,T D_a,T K_a_);
void Admittance_set(AdmittanceController* ctrl, T M_a, T D_a,T K_a_);

void Admittance_Compute(AdmittanceController* ctrl, T external_force);
void update_AdmittanceController(AdmittanceController* ctrl, joint_control* joint);
void Admittance_Run(AdmittanceController* ctrl, joint_control* joint,T external_force);

void Admittance2joint(AdmittanceController* ctrl, joint_control* joint);
void Admittance_pc_set(AdmittanceController* ctrl,motor_parameter_typedef *mp);

T ExternalForce_Set(AdmittanceController* ctrl, T pos);
#endif

