#include "filt.h"
#include "stdio.h"
#include <string.h>
// -- »¬¶¯Æ½¾ùÂË²¨ --// 

LPF joint_lpf[2];

void LPF_Init(LPF* lpf, float cutoff, float dt)
{
    lpf->cutoff = cutoff;
    lpf->dt = dt;
    float RC = 1.0f / (2.0f * 3.1415926f * cutoff);
    lpf->alpha = dt / (RC + dt);
    lpf->y = 0.0f;
}

float LPF_Update(LPF* lpf, float x)
{
    lpf->y = lpf->y + lpf->alpha * (x - lpf->y);
    return lpf->y;
}


float movingAverage(FILT_HandleTypedef* Flit,float newSample) {

    Flit->sum -= Flit->buffer[Flit->index];
    Flit->sum += newSample;
    Flit->buffer[Flit->index] = newSample;
    Flit->index = (Flit->index + 1) % WINDOW_SIZE;
    
    return Flit->sum / WINDOW_SIZE;
}


void FILT_init(FILT_HandleTypedef* Flit){
	Flit->index = 0;
	Flit->sum = 0;
	memset(Flit->buffer,0,sizeof(Flit->buffer));
}


void Kalman_Init(KalmanFilter_HandleTypedef* kf, float LastP,float Q,float R)      
{
	kf->LastP = LastP;
	kf->Now_P = 0;
	kf->out = 0;
	kf->Kg = 0;
	kf->Q = Q;
	kf->R = R;
	
}

float Kalman_Predict(KalmanFilter_HandleTypedef* kfp,float input){
	kfp->Now_P = kfp->LastP + kfp->Q;
	kfp->Kg = kfp->Now_P / (kfp->Now_P + kfp->R);
	kfp->out = kfp->out + kfp->Kg * (input -kfp->out);
	kfp->LastP = (1-kfp->Kg) * kfp->Now_P;
	return kfp->out;
}



