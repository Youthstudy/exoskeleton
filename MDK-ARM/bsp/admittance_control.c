#include "admittance_control.h"
#include "math.h"

void AController_init(AdmittanceController* ac,
										T M_a, 
										T D_a,
										int f){
	ac->M_a_ = M_a;
	ac->D_a_ = D_a;
	ac->frequency = f;
	
}





