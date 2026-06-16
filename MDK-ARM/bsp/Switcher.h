#ifndef __SWITCHER_H_
#define  __SWITCHER_H_

#include "control.h"
#include <string.h>
#include "admittance_control.h"
#include "impandance_control.h"
#include "math.h"

typedef enum { 
	MODE_IMP = 0, 
	MODE_ADM = 1,
	MODE_IDLE = 2
 } ControlMode;

typedef struct 
{
	ControlMode mode;
}Switcher;

typedef struct{
	float M_, B_, K_;
	float x, v;
	
}Virtual_state;

extern Switcher switcher[2];
extern Virtual_state virtual_state[2];


void EnergyConsistentSwitch(Switcher *sw,
                            ImpandanceController *imp,
                            AdmittanceController *adm,
                            ControlMode new_mode);

void Virtual_init(Virtual_state* vs, float M, float B, float K);
float Ac2Vs_Tau(AdmittanceController* adm, Virtual_state* vs);
float Ic2Vs_Tau(ImpandanceController* imp, Virtual_state* vs);
void Virtual_compute(Virtual_state* vs, float tau, float tau_c, float dt);

#endif

