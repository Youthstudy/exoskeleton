#ifndef __ADMITTANCE_CONTROL_H__
#define  __ADMITTANCE_CONTROL_H__


typedef float T;

typedef struct {
	// -- ADMITTANCE PARAMETERS --- //
	// M_a_  Desired mass
	// D_a_ Desired damping

	T M_a_, D_a_; 
	int frequency; // hz
	
	// -- CALLBACK -- //
	
	
	
	// -- OUTPUT COMMANDS --- //
	// final arm desired velocity
	T motor_desired_twist_;
	
}AdmittanceController;



#endif

