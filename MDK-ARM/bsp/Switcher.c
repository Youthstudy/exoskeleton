#include "Switcher.h"


Switcher switcher[2];
Virtual_state virtual_state[2];

float Ac2Vs_Tau(AdmittanceController* adm, Virtual_state* vs){
    return adm->kp * (adm->delta_pos - vs->x) + adm->kd * (adm->delta_velocity - vs->v);
}

float Ic2Vs_Tau(ImpandanceController* imp, Virtual_state* vs){
    return imp->K_ * (imp->pos_cmd - vs->x) + imp->B_ * (imp->v_cmd - vs->v);
}

void Virtual_compute(Virtual_state* vs, float tau, float tau_c, float dt){
    float acc = (tau - tau_c) / vs->M_;
         //- vs->B_ * vs->v - vs->K_ * vs->x
        
    vs->v += acc * dt;
    vs->x += vs->v * dt;
}

void Virtual_init(Virtual_state* vs, float M, float B, float K){
    vs->M_ = M;
    vs->B_ = B;
    vs->K_ = K;
    vs->x = 0.0f;
    vs->v = 0.0f;
}


// void SmoothFilter_Init(SmoothFilter* f, float Ts, float tau){
//     f->Ts = Ts;
//     f->tau = tau;
//     f->pos = 0.0f;
//     f->vel = 0.0f;
// }

// void SmoothFilter_Update(SmoothFilter* f, float u){
//     float Ts = f->Ts;

//     float acc = (u - f->pos) / f->tau;  // = dx/dt -> smooth track

//     // velocity update
//     f->vel += Ts * acc;

//     // position update
//     f->pos += Ts * f->vel;
// }


/* ---------- Energy-consistent switching routine ---------- */
void EnergyConsistentSwitch(Switcher *sw,
                            ImpandanceController *imp,
                            AdmittanceController *adm,
                            ControlMode new_mode) {
    if (sw->mode == new_mode) return; /* nothing to do */


    if (new_mode == MODE_IMP) {

    } else {
			float F = imp->K_ *(imp->x - imp->pos_cmd) + imp->B_ *(imp->v_cmd - imp->v);
			adm->delta_pos = F / adm->K_a_;
			adm->delta_velocity = 0;
    }
    /* commit switch */
    sw->mode = new_mode;

}
														
