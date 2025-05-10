#ifndef __IMPANDANCE_CONTROL_H__
#define __IMPANDANCE_CONTROL_H__

#include "control.h"
typedef float T;


typedef struct{
	// -- IMPANDANCE PARAMETER --//
	T M_;
	T B_; 
	T K_;
	int frequence;
	T pos_init;
	
	// -- CALLBACK -- //
	T current_pos;
	T current_v;
	T current_acc;
	T current_twist;
	
	// -- MOTOR PARAMETER --//
	T pos_target;
	T v_target;
	
	// -- OUTPUT -- //
	T forces_cmd;
	
}ImpandanceController;

extern ImpandanceController Ictrl[2];

void ImpedanceCtrl_Init(ImpandanceController* ctrl,
                        T M, T B,T K,
                        T pos_init,
                        int frequence);
void update_ImpandanceController(ImpandanceController* ctrl, joint_control* joint);
void Impedance_Set(ImpandanceController* ctrl,T M, T B,T K);
void Impedance2joint(ImpandanceController* ctrl, joint_control* joint);
void ImpedanceCtrl_Run(ImpandanceController* ctrl, joint_control* joint, T pos_end, T v_desire);
void ImpedanceCtrl_Compute(ImpandanceController* ctrl, joint_control* joint, T pos_desire, T v_desire);


#endif

