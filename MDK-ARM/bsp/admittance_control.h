#ifndef __ADMITTANCE_CONTROL_H__
#define  __ADMITTANCE_CONTROL_H__


typedef float T;

#define OUTPUT_MAX 1.2384


typedef struct {
	// -- ADMITTANCE PARAMETERS --- //
	// M_a_  Desired mass(kg)
	// D_a_ Desired damping(N.s/m)

	
	T M_a_, D_a_; 
	int frequency; // hz
	T init_pos; // 初始位置 弧度
	
	// -- CALLBACK -- //
	T current_pos; // 弧度
	T target_pos;	// 弧度
	T current_velocity; //当前角速度 
	T real_twist;
	
	// -- OUTPUT COMMANDS --- //
	// final arm desired velocity
	T velocity_cmd;
	T motor_desired_twist_;
	T pos_cmd;
	
}AdmittanceController;

void Admittance_Compute(AdmittanceController* ctrl, T external_force);

#endif

