#ifndef __FILT_H__
#define __FILT_H__

#include <stdio.h>
#include <stdlib.h>

#define WINDOW_SIZE 20

typedef float T;

typedef struct{
	T buffer[WINDOW_SIZE];
	int index;
	T sum;
}FILT_HandleTypedef;

typedef struct{
	// -- Kalman Parameter -- //
	// x: ״̬����ֵ
	// P: �������Э����
	// Q: ������������
	// R: ������������
	// Kg: ����������
	T LastP;
	T Now_P;
	T out;
	T Kg;
	T Q;
	T R;

}KalmanFilter_HandleTypedef;

float movingAverage(FILT_HandleTypedef* Flit,float newSample);
void FILT_init(FILT_HandleTypedef* Flit);

void Kalman_Init(KalmanFilter_HandleTypedef* kf, T LastP,T Q,T R);
T Kalman_Predict(KalmanFilter_HandleTypedef* kfp,T input);



#endif


