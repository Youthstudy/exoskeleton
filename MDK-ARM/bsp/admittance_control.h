#ifndef __ADMITTANCE_CONTROL_H__
#define  __ADMITTANCE_CONTROL_H__

#include "control.h"
#include "filt.h"

#define CONTROL_FRE 1000
#define F_DEADBOUND 0.01

#define POS_CONTROL_MAX 1.2883

typedef struct {
	// -- ADMITTANCE PARAMETERS --- //
	// M_a_  Desired mass(kg)
	// D_a_ Desired damping(N.s/m)
	float M_a_, D_a_, K_a_; 
	int frequency; // hz
	float pos_init; // ≥ı ºŒª÷√ ª°∂»
	float dt;
	int start_flag;
	float delta_velocity;
	float delta_pos;	
	float kp,kd;
	
	// -- OUTPUT COMMANDS --- //
	// final arm desired velocity
	float velocity_cmd;
	float pos_cmd;

	float f_output;
	
	// -- FILT STRUCT --//
	FILT_HandleTypedef filt;
	
	// -- FLAG --
	int AC_enable;
	
}AdmittanceController;

extern AdmittanceController ACtrl[2];

void Admittance_init(AdmittanceController* ac, joint_control* joint,float M_a,float D_a,float K_a_);
void Admittance_set(AdmittanceController* ctrl, float M_a, float D_a,float K_a_);
void Admittance_PDset(AdmittanceController* ctrl,float p, float d);

void Admittance_Compute(AdmittanceController* ctrl, float external_force,float x0, float x0_dot);

void Admittance2joint(AdmittanceController* ctrl, joint_control* joint);
void Admittance_pc_set(AdmittanceController* ctrl,motor_parameter_typedef *mp, joint_control* joint);


#endif

