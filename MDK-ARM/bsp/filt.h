#ifndef __FILT_H__
#define __FILT_H__

#include <stdio.h>
#include <stdlib.h>

#define WINDOW_SIZE 20

typedef struct{
	float buffer[WINDOW_SIZE];
	int index;
	float sum;
}FILT_HandleTypedef;

typedef struct{
	// -- Kalman Parameter -- //
	// x: 状态估计值
	// P: 估计误差协方差
	// Q: 过程噪声方差
	// R: 测量噪声方差
	// Kg: 卡尔曼增益
	float LastP;
	float Now_P;
	float out;
	float Kg;
	float Q;
	float R;

}KalmanFilter_HandleTypedef;

typedef struct {
    float cutoff; 
    float dt;  
    float alpha; 
    float y;   
} LPF;


extern LPF joint_lpf[2];

float movingAverage(FILT_HandleTypedef* Flit,float newSample);
void FILT_init(FILT_HandleTypedef* Flit);

void Kalman_Init(KalmanFilter_HandleTypedef* kf, float LastP,float Q,float R);
float Kalman_Predict(KalmanFilter_HandleTypedef* kfp,float input);

void LPF_Init(LPF* lpf, float cutoff, float dt);
float LPF_Update(LPF* lpf, float x);

#endif


