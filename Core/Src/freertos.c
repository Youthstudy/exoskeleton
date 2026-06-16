/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "my_task.h"
#include "TCPsend.h"
#include "control.h"
#include "admittance_control.h"
#include "impandance_control.h"
#include "can.h"
#include "cansend.h"
#include "Switcher.h"
#include "filt.h"
#include "serial_parser.h"
#include "compliance_ctrl.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MAXT 10

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId controlTaskHandle;
/* USER CODE END Variables */
osThreadId SendPCHandle;
osThreadId ScheduleHandle;
osThreadId ControlHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void SendPCTask(void const * argument);
void scheduleTask(void const * argument);
void ControlTaskHandle(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  serial_parser_init();
  for(int i = 0; i < 2; i++){
    Compliance_GetDefaultParams(&comp_params[i]);
    Compliance_Init(&comp_params[i], &comp_state[i]);
  }
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of SendPC */
  osThreadDef(SendPC, SendPCTask, osPriorityNormal, 0, 512);
  SendPCHandle = osThreadCreate(osThread(SendPC), NULL);

  /* definition and creation of Schedule */
  osThreadDef(Schedule, scheduleTask, osPriorityNormal, 0, 512);
  ScheduleHandle = osThreadCreate(osThread(Schedule), NULL);

  /* definition and creation of Control */
  osThreadDef(Control, ControlTaskHandle, osPriorityNormal, 0, 512);
  ControlHandle = osThreadCreate(osThread(Control), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	// osThreadDef(Acontrol, vControlOutputTask, osPriorityNormal, 0, 128);
	// controlTaskHandle = osThreadCreate(osThread(Acontrol), NULL);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_SendPCTask */
/**
  * @brief  Function implementing the SendPC thread.
  * @param  argument: Not used
  * @retval None
  */
volatile float global_values[8];
/* USER CODE END Header_SendPCTask */
void SendPCTask(void const * argument)
{
  /* USER CODE BEGIN SendPCTask */
	const TickType_t xFrequency = pdMS_TO_TICKS(1); // ??100ms
	TickType_t xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
    uint8_t payload[MAX_PAYLOAD_LEN];
    uint8_t len;
    
    if (serial_parse_frame(payload, &len)) {
        // 收到完整帧，处理数据
        
        // 示例1: 解析为float数组
        
        int count = parse_floats(payload, len, (float*)global_values, 3);
        // 使用 values[0], values[1], ...
        
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
  /* USER CODE END SendPCTask */
}

/* USER CODE BEGIN Header_scheduleTask */
/**
* @brief Function implementing the Schedule thread.
* @param argument: Not used
* @retval None
*/
// 时间范围: t ∈ [0.000000, 1.999000]
//#define T_MIN_P 0.000000
//#define T_MAX_P 2.721000

// 傅里叶系数
// wzh right_up_di
//static const double w = 1.017190706287113e+01;  // 角频率
//static const double a0 = -9.108506712565281e-01;
//static const double a[] = {3.178938902301109e-01, 1.318572500854099e-01, -4.626654956653483e-02};
//static const double b[] = {7.694517550492494e-02, 1.067942984658690e-01, 2.332483507032227e-02};

// wzg right_up_gao
// 傅里叶系数
//static const double w = 9.085002738813143e+00;  // 角频率
//static const double a0 = -8.558713808726979e-01;
//static const double a[] = {3.714930085264309e-01, 1.344914296912033e-01, -1.208698989869265e-02};
//static const double b[] = {-1.073221925928646e-01, -4.284804686540649e-02, 6.363065113276095e-02};


// szy jing
//// 时间范围: t ∈ [0.000000, 0.921000]
//#define T_MIN_P 0.000000
//#define T_MAX_P 2.921000

//// 傅里叶系数
//static const double w = 7.149736970401064e+00;  // 角频率
//static const double a0 = -9.012146016432182e-01;
//static const double a[] = {6.692161768879318e-01, 2.143694606058872e-01, 6.803538346503635e-02};
//static const double b[] = {-2.170490735683210e-01, -1.471214564634978e-01, 1.386343467134605e-01};

// szy yuan
//// 时间范围: t ∈ [0.000000, 1.018000]
//#define T_MIN_P 0.000000
//#define T_MAX_P 2.018000

//// 傅里叶系数
//static const double w = 5.567456806943049e+00;  // 角频率
//static const double a0 = -7.531657271911131e-01;
//static const double a[] = {5.265103300742116e-01, 3.840975729741392e-02, 2.344655208986193e-01, 7.183539333085006e-03};
//static const double b[] = {-6.038621678663676e-01, -1.252499189785593e-02, -9.961424178530858e-03, 1.020178519471053e-01};

////wch di
//#define T_MIN_P 0.000000
//#define T_MAX_P 1.023000

//// 傅里叶系数
//static const double w = 6.301958474628949e+00;  // 角频率
//static const double a0 = 2.355991245050300e-01;
//static const double a[] = {-1.232299712676776e-02, -2.218585643657538e-01, 1.820611909519642e-02, -3.540546088750866e-03};
//static const double b[] = {-3.388379035700089e-01, -3.838551717485243e-02, 9.241659231690086e-02, -1.455855930983151e-02};

//wch gao
// 时间范围: t ∈ [0.000000, 0.910000]
#define T_MIN_P 0.000000
#define T_MAX_P 0.910000

// 傅里叶系数
static const double w = 7.216352212816728e+00;  // 角频率
static const double a0 = 2.723864061044429e-01;
static const double a[] = {1.382235568242246e-01, -1.621382155843857e-01, -1.376901738052186e-01, -1.387745172159359e-02};
static const double b[] = {-4.076819947468329e-01, -1.890605291499078e-01, 7.307708442016677e-03, 7.837771083478309e-02};


// 角度 (rad)
double get_angle(double t) {
    double sum = a0;
    for (int i = 0; i < 4; i++) {
        sum += a[i]*cos((i+1)*w*t) + b[i]*sin((i+1)*w*t);
    }
    return sum;
}

// 速度 (rad/s)
double get_velocity(double t) {
    double sum = 0;
    for (int i = 0; i < 4; i++) {
        double nw = (i+1)*w;
        sum += nw*(-a[i]*sin(nw*t) + b[i]*cos(nw*t));
    }
    return sum;
}

// 加速度 (rad/s²)
double get_acc(double t) {
    double sum = 0;
    for (int i = 0; i < 4; i++) {
        double nw = (i+1)*w;
        sum += -nw*nw*(a[i]*cos(nw*t) + b[i]*sin(nw*t));
    }
    return sum;
}
//typedef enum {
//    COMP_MODE_IMPEDANCE = 1,    // 纯阻抗模式
//    COMP_MODE_ADMITTANCE,       // 纯导纳模式
//    COMP_MODE_HYBRID_FIXED,     // 固定占空比混合
//    COMP_MODE_HYBRID_ADAPT,     // 自适应占空比混合
//    COMP_MODE_PROXY             // 虚拟代理模式
//} ComplianceMode_e;

volatile int control_flag = 0,start_flag = 0;
int n[2] = {5,5};
ComplianceMode_e mode = COMP_MODE_IMPEDANCE;
float cmd[2][3] = {{0,0,0},{0,0,0}};
volatile float p = 40,d = 15;
/* USER CODE END Header_scheduleTask */
void scheduleTask(void const * argument)
{
  /* USER CODE BEGIN scheduleTask */
	const TickType_t xFrequency = pdMS_TO_TICKS(1); // 1ms
	TickType_t xLastWakeTime = xTaskGetTickCount();
	double t = 0;
  /* Infinite loop */
  for(;;)
  {
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
		control_flag = global_values[1];
		mode = (ComplianceMode_e)global_values[2];
		if(control_flag == 0){
			continue;
		}
		if(t >= T_MAX_P){
			control_flag = 0;
			t = 0;
			global_values[1] = 0;

		}
		if(t == 0){
			EnterMotorMode(1);
		}
//		if(start_flag == 0){
//			t = 0;
//			continue;
//		}
		

    for(int i = 0; i < MOTOR; i++){
//      Impedance_update(&Ictrl[i], joint[i].ret[0], joint[i].ret[1]);
//      float flag = i == 0? 1 : -1;
//      float tau_c = joint[i].kp *(virtual_state[i].x - flag * joint[i].ret[0]) 
//        + joint[i].kd * (virtual_state[i].v - flag * joint[i].ret[1]);
//			float tau;
//      if(cnt >= n[i] && cnt <= MAXT){
//        tau = Ac2Vs_Tau(&ACtrl[i], &virtual_state[i]);
//        Virtual_compute(&virtual_state[i], tau, tau_c, 0.001);
//				joint[i].moveflag = MODE_ADM;
//      }else if(cnt < n[i]){
//        tau = Ic2Vs_Tau(&Ictrl[i], &virtual_state[i]);
//        Virtual_compute(&virtual_state[i], tau, tau_c, 0.001);
//				joint[i].moveflag = MODE_IMP;
//      }
			cmd[i][0] = get_angle(t);
			cmd[i][1] = get_velocity(t);
			cmd[i][2] = get_acc(t);
			joint[i].pos_cmd = cmd[i][0];
      RobotFeedback_t fb = fb_creater(joint[i].ret[0], joint[i].ret[1], joint[i].ret[2]);
      DesiredTrajectory_t traj = traj_creater(cmd[i][0], cmd[i][1], cmd[i][2]);
      joint[i].moveflag = Compliance_Update(mode,
                                          &comp_params[i],
                                          &comp_state[i],
                                          &traj,
                                          &fb,
                                          &comp_output[i]);
																					
			joint_set(&joint[i],comp_output[i].pos_cmd,comp_output[i].vel_cmd,0,p,d);
      pack_cmd(joint[i].senddata, joint[i]);
    }

    CAN_Send_Msg(&hcan1, joint[0].senddata, 1);
    CAN_Send_Msg(&hcan2, joint[1].senddata, 1);
    uint8_t buffer[128] = {0};
    uint8_t tmp = 0;
    tmp = joint_pack(&joint[0], buffer, tmp);
    tmp = joint_pack(&joint[1], buffer, tmp);
    sendData(buffer, tmp);
		t += 0.001f;

  }
  /* USER CODE END scheduleTask */
}

/* USER CODE BEGIN Header_ControlTaskHandle */
/**
* @brief Function implementing the Control thread.
* @param argument: Not used
* @retval None
*/


/* USER CODE END Header_ControlTaskHandle */
void ControlTaskHandle(void const * argument)
{
  /* USER CODE BEGIN ControlTaskHandle */
	const TickType_t xFrequency = pdMS_TO_TICKS(1); // 1ms
	TickType_t xLastWakeTime = xTaskGetTickCount();
	double t = 0;
  /* Infinite loop */
  for(;;)
  {
		for(int i = 0; i < MOTOR; i ++){
      float force_flag = joint[i].force_flag;
      float x = Ictrl[i].flag * joint[i].ret[0];
			float pos_des = i == 0 ? 0: 0;
			Admittance_Compute(&ACtrl[i], force_flag * joint[i].ret[2], pos_des, 0.0f);
			Impedance_Set(&Ictrl[i], pos_des , Ictrl[i].K_, Ictrl[i].B_);
		}
		t += 0.001;
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
  /* USER CODE END ControlTaskHandle */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
