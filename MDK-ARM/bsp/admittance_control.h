#ifndef __ADMITTANCE_CONTROL_H__
#define  __ADMITTANCE_CONTROL_H__


typedef double T;

struct AdmittanceController{
	// -- ADMITTANCE PARAMETERS --- //
	// M_a_  Desired mass
	// D_a_ Desired damping
	// K_a_ Desired K
	T M_a_, D_a_ , K_a_; 
	int frequency; // hz
	
	
	
	
	// -- OUTPUT COMMANDS --- //
	// final arm desired velocity
	T motor_desired_twist_;
	
};



#endif

