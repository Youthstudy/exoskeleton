#ifndef __IMPANDANCE_CONTROL_H__
#define __IMPANDANCE_CONTROL_H__

#include "control.h"

typedef struct{
	// -- IMPANDANCE PARAMETER --//
	float B_; 
	float K_;
	float M_;
	int frequence;
	float pos_init;
	int enable;
	
	float x;      // position
	float v;      // velocity

	// -- MOTOR PARAMETER --//
	float pos_cmd;
	float v_cmd;
	float flag;
	
}ImpandanceController;

extern ImpandanceController Ictrl[2];

void ImpedanceCtrl_Init(ImpandanceController* ctrl, float M, float B, float K,float flag,int frequence);

void Impedance_Set(ImpandanceController* ctrl,float pos, float K ,float B);
void Impedance_update(ImpandanceController* ctrl,float pos,float v);

void Impedance2joint(ImpandanceController* ctrl, joint_control* joint);
void Impedance_pc_set(ImpandanceController* ctrl,motor_parameter_typedef *mp);
#endif

