/**
 * @file compliance_ctrl.h
 * @brief 柔顺控制器头文件 - 位置/速度输出版本
 */

#ifndef COMPLIANCE_CTRL_H
#define COMPLIANCE_CTRL_H

#include <stdint.h>
#include <stdbool.h>

/* ==================== 控制模式枚举 ==================== */
typedef enum {
    COMP_MODE_IMPEDANCE = 1,    // 纯阻抗模式
    COMP_MODE_ADMITTANCE,       // 纯导纳模式
    COMP_MODE_HYBRID_FIXED,     // 固定占空比混合
    COMP_MODE_HYBRID_ADAPT,     // 自适应占空比混合
    COMP_MODE_PROXY             // 虚拟代理模式
} ComplianceMode_e;

/* ==================== 机器人反馈结构体 ==================== */
typedef struct {
    float pos;      // 当前位置
    float vel;      // 当前速度
    float force;    // 外力/力矩反馈
} RobotFeedback_t;

/* ==================== 期望轨迹结构体 ==================== */
typedef struct {
    float pos;      // 期望位置
    float vel;      // 期望速度
    float acc;      // 期望加速度
} DesiredTrajectory_t;

/* ==================== 参数结构体 ==================== */
typedef struct {
    // 阻抗参数 (目标阻抗特性)
    struct {
        float Md;   // 期望惯性
        float Dd;   // 期望阻尼
        float Kd;   // 期望刚度
    } impedance;
    
    // 导纳滤波器参数
    struct {
        float Ma;   // 虚拟质量
        float Ba;   // 虚拟阻尼
        float Ka;   // 虚拟刚度
    } admittance;
    
    // 位置内环增益 (可选使用)
    struct {
        float kp;   // 位置增益
        float kv;   // 速度增益
    } pos_gains;
    
    // 虚拟代理参数
    struct {
        float Mv;   // 代理虚拟质量
        float Kc;   // 耦合刚度
        float Bc;   // 耦合阻尼
    } proxy;
    
    // 模式切换参数
    struct {
        float Ts;   // 切换周期
        float duty; // 导纳占空比 (0~1)
    } switching;
    
    // 系统参数
    float Mr;       // 机器人等效质量
    float dt;       // 控制周期
} ComplianceParams_t;

/* ==================== 状态结构体 ==================== */
typedef struct {
    // 导纳滤波器状态 (用于导纳模式)
    float x_adm;        // 导纳位置
    float v_adm;        // 导纳速度
    float a_adm;        // 导纳加速度
    
    // 阻抗滤波器状态 (用于阻抗模式，存储偏移量)
    float x_imp;        // 位置偏移量
    float v_imp;        // 速度偏移量
    float a_imp;        // 加速度偏移量
    
    // 虚拟代理状态
    float x_proxy;      // 代理位置
    float v_proxy;      // 代理速度
    float a_proxy;      // 代理加速度
    
    // 时间相关
    float tau;          // 当前周期内时间
    float t_total;      // 总运行时间
    
    // 模式状态
    uint8_t active_mode;    // 当前激活子模式 (1=阻抗, 2=导纳)
    bool initialized;       // 初始化标志
} ComplianceState_t;

/* ==================== 输出结构体 (修改为位置/速度输出) ==================== */
typedef struct {
    float pos_cmd;      // 期望位置指令 (给底层PD控制器)
    float vel_cmd;      // 期望速度指令 (给底层PD控制器)
    uint8_t mode_active; // 当前激活模式
} ComplianceOutput_t;

/* ==================== 全局变量声明 ==================== */
extern ComplianceParams_t comp_params[2];
extern ComplianceState_t comp_state[2];
extern ComplianceOutput_t comp_output[2];

/* ==================== API函数声明 ==================== */

/**
 * @brief 创建机器人反馈结构体
 */
RobotFeedback_t fb_creater(float pos, float vel, float force);

/**
 * @brief 创建期望轨迹结构体
 */
DesiredTrajectory_t traj_creater(float pos, float vel, float acc);

/**
 * @brief 获取默认参数
 */
void Compliance_GetDefaultParams(ComplianceParams_t *params);

/**
 * @brief 初始化控制器
 */
void Compliance_Init(ComplianceParams_t *params, ComplianceState_t *state);

/**
 * @brief 重置控制器状态
 */
void Compliance_Reset(ComplianceState_t *state, 
                      const RobotFeedback_t *feedback,
                      const DesiredTrajectory_t *traj);

/**
 * @brief 柔顺控制更新 (位置/速度输出)
 * 
 * @param mode      控制模式
 * @param params    控制参数
 * @param state     控制状态
 * @param traj      期望轨迹
 * @param feedback  机器人反馈
 * @param output    输出 (pos_cmd, vel_cmd)
 * @return          当前激活模式
 */
int Compliance_Update(ComplianceMode_e mode,
                      const ComplianceParams_t *params,
                      ComplianceState_t *state,
                      const DesiredTrajectory_t *traj,
                      const RobotFeedback_t *feedback,
                      ComplianceOutput_t *output);

/**
 * @brief 设置阻抗参数
 */
void Compliance_SetImpedance(ComplianceParams_t *params, 
                             float Md, float Dd, float Kd);

/**
 * @brief 设置导纳参数
 */
void Compliance_SetAdmittance(ComplianceParams_t *params,
                              float Ma, float Ba, float Ka);

/**
 * @brief 设置占空比
 */
void Compliance_SetDutyCycle(ComplianceParams_t *params, float duty);

#endif /* COMPLIANCE_CTRL_H */

