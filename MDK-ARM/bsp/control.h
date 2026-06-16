#ifndef __CONTROL_H_
#define __CONTROL_H_

#include "pid.h"
#include "math.h"
#include "filt.h"

#define PC_DEBUG_ENABLE 1
#define DT                (0.001f)    /* control timestep [s] */
#define MIN_DWELL_TIME    (0.05f)     /* minimum time [s] to stay in a mode after switching */
#define ENERGY_EPS        (1e-6f)     /* small energy floor to avoid division by zero */
#define SMALL_ENERGY_TH   (1e-4f) 
// pc 输入测试结构体
#define PC_RECIEVE_LEN 6
#define PC_BUFFER 60




typedef struct
{
	float p_des, v_des, kp, kd, t_ff;
	float ret[3];
	float filt_res[3];
	FILT_HandleTypedef filt[3];
	KalmanFilter_HandleTypedef kfilter[3];
	float p_init;
	int moveflag; 
	
	float pos_cmd;
	
	float force_flag;

	uint8_t senddata[8];
}joint_control;



typedef struct
{
	uint8_t RecieveBuffer[1];
	uint8_t step;
	uint8_t cnt;
	char str[PC_BUFFER];
	float buff[PC_RECIEVE_LEN];
	float p_des, v_des, kp, kd, t_ff;
	float Force_time;
} motor_parameter_typedef;

// struct
extern motor_parameter_typedef motor_parameter;
extern joint_control joint[2];


// function
void joint_init(joint_control *joint,int id, float kp, float kd);
void joint_setZero(joint_control *joint);

void joint_set(joint_control *joint,float p_des, float v_des, float t_ff, float kp, float kd);
void joint_pc_set(joint_control *joint, motor_parameter_typedef *parameter);

void motor_parameter_init(motor_parameter_typedef *mp);


#endif


