#ifndef __IMPANDANCE_CONTROL_H__
#define __IMPANDANCE_CONTROL_H__

#include "control.h"
typedef float T;


typedef struct{
	// -- IMPANDANCE PARAMETER --//
	T Kp;
	T Kd; 
	int frequence;
	T pos_init;
	
	// -- CALLBACK -- //
	T current_pos;
	T current_v;
	T current_twist;
	
	// -- MOTOR PARAMETER --//
	T pos_target;
	T v_target;
	
	// -- OUTPUT -- //
	T forces_cmd;
	
}ImpandanceController;

void ImpedanceCtrl_Init(ImpandanceController* ctrl,
                        T Kp, T Kd,
                        T pos_init,
                        int frequence);
void update_ImpandanceController(ImpandanceController* ctrl, joint_control* joint);


#endif

