/**
 * @file compliance_ctrl_pd.c
 * @brief 柔顺控制器 - 阻抗/导纳混合实现 (位置/速度输出版本)
 * 
 * 修改说明:
 * - 底层为PD位置控制器，所有模式直接输出期望位置(pos_cmd)和期望速度(vel_cmd)
 * - 不再输出力矩Fc
 * 
 * 模式说明:
 * 1. 阻抗模式: 根据外力修正期望轨迹，表现目标阻抗特性
 * 2. 导纳模式: 导纳滤波器输出修正位置/速度
 * 3. 混合模式: 周期性切换，兼顾两者优点
 * 4. 代理模式: 虚拟质量块解耦，输出代理位置/速度
 */

#include "compliance_ctrl.h"
#include <math.h>
#include <string.h>

ComplianceParams_t comp_params[2];
ComplianceState_t comp_state[2];
ComplianceOutput_t comp_output[2];

/* ==================== 私有宏 ==================== */
#define CLAMP(x, lo, hi)    ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
#define EPS                 (1e-6f)

/* ==================== 私有函数声明 ==================== */
static void compute_impedance_pd(const ComplianceParams_t *p,
                                 ComplianceState_t *s,
                                 const DesiredTrajectory_t *traj,
                                 const RobotFeedback_t *fb,
                                 float *pos_cmd, float *vel_cmd);

static void compute_admittance_pd(const ComplianceParams_t *p,
                                  ComplianceState_t *s,
                                  const DesiredTrajectory_t *traj,
                                  const RobotFeedback_t *fb,
                                  float *pos_cmd, float *vel_cmd);

static void compute_proxy_pd(const ComplianceParams_t *p,
                             ComplianceState_t *s,
                             const DesiredTrajectory_t *traj,
                             const RobotFeedback_t *fb,
                             float *pos_cmd, float *vel_cmd);

static float fmod_positive(float x, float y);

/* ==================== API实现 ==================== */

RobotFeedback_t fb_creater(float pos, float vel, float force)
{
    RobotFeedback_t fb;
    fb.pos = pos;
    fb.vel = vel;
    fb.force = force;
    return fb;
}

DesiredTrajectory_t traj_creater(float pos, float vel, float acc)
{
    DesiredTrajectory_t traj;
    traj.pos = pos;
    traj.vel = vel;
    traj.acc = acc;
    return traj;
}

void Compliance_GetDefaultParams(ComplianceParams_t *params)
{
    if (!params) return;
    
    // 阻抗参数 (用于阻抗模式的导纳滤波)
    // Md*a + Dd*v + Kd*x = Fext
    params->impedance.Md = 5.0f;   // 期望惯性
    params->impedance.Dd = 50.0f;  // 期望阻尼
    params->impedance.Kd = 100.0f; // 期望刚度
    
    // 导纳滤波器参数
    // Ma*a + Ba*v + Ka*x = Fext
    params->admittance.Ma = 2.0f;   // 虚拟质量
    params->admittance.Ba = 30.0f;  // 虚拟阻尼
    params->admittance.Ka = 100.0f; // 虚拟刚度
    
    // 位置内环增益 (保留，可用于某些计算)
    params->pos_gains.kp = 15.0f;
    params->pos_gains.kv = 5.0f;
    
    // 虚拟代理参数
    params->proxy.Mv = 1.0f;   // 代理虚拟质量
    params->proxy.Kc = 50.0f;  // 耦合刚度
    params->proxy.Bc = 10.0f;  // 耦合阻尼
    
    // 时间切换参数
    params->switching.Ts = 0.01f;   // 切换周期
    params->switching.duty = 0.5f;  // 导纳占空比
    
    // 系统参数
    params->Mr = 3.0f;      // 机器人等效质量
    params->dt = 0.001f;    // 控制周期
}

void Compliance_Init(ComplianceParams_t *params, ComplianceState_t *state)
{
    if (!params || !state) return;
    memset(state, 0, sizeof(ComplianceState_t));
    state->initialized = false;
}

void Compliance_Reset(ComplianceState_t *state, 
                      const RobotFeedback_t *feedback,
                      const DesiredTrajectory_t *traj)
{
    if (!state) return;
    
    // 用当前状态初始化滤波器
    if (feedback) {
        // 导纳滤波器状态
        state->x_adm = feedback->pos;
        state->v_adm = feedback->vel;
        // 阻抗滤波器状态 (偏移量初始化为0)
        state->x_imp = 0.0f;
        state->v_imp = 0.0f;
        // 代理状态
        state->x_proxy = feedback->pos;
        state->v_proxy = feedback->vel;
    } else if (traj) {
        state->x_adm = traj->pos;
        state->v_adm = traj->vel;
        state->x_imp = 0.0f;
        state->v_imp = 0.0f;
        state->x_proxy = traj->pos;
        state->v_proxy = traj->vel;
    }
    
    state->a_adm = 0.0f;
    state->a_imp = 0.0f;
    state->a_proxy = 0.0f;
    state->tau = 0.0f;
    state->t_total = 0.0f;
    state->active_mode = 1;
    state->initialized = true;
}

/**
 * @brief 柔顺控制更新函数 (位置/速度输出版本)
 * 
 * @param mode      控制模式
 * @param params    控制参数
 * @param state     控制状态
 * @param traj      期望轨迹
 * @param feedback  机器人反馈 (包含位置、速度、外力)
 * @param output    输出结构体 (pos_cmd, vel_cmd)
 * @return          当前激活模式，0表示错误
 */
int Compliance_Update(ComplianceMode_e mode,
                      const ComplianceParams_t *params,
                      ComplianceState_t *state,
                      const DesiredTrajectory_t *traj,
                      const RobotFeedback_t *feedback,
                      ComplianceOutput_t *output)
{
    if (!params || !state || !traj || !feedback || !output) return 0;
    
    // 首次自动初始化
    if (!state->initialized) {
        Compliance_Reset(state, feedback, traj);
    }
    
    float pos_cmd = traj->pos;
    float vel_cmd = traj->vel;
    uint8_t active = 0;
    
    // 更新周期内时间
    state->tau = fmod_positive(state->t_total, params->switching.Ts);
    
    switch (mode) {
        
        /* ========== 模式1: 纯阻抗 ========== */
        case COMP_MODE_IMPEDANCE:
            compute_impedance_pd(params, state, traj, feedback, &pos_cmd, &vel_cmd);
            active = 1;
            break;
        
        /* ========== 模式2: 纯导纳 ========== */
        case COMP_MODE_ADMITTANCE:
            compute_admittance_pd(params, state, traj, feedback, &pos_cmd, &vel_cmd);
            active = 2;
            break;
        
        /* ========== 模式3: 固定占空比混合 ========== */
        case COMP_MODE_HYBRID_FIXED:
        {
            float t_switch = 0.5f * params->switching.Ts;  // 固定50%
            
            if (state->tau < t_switch) {
                compute_impedance_pd(params, state, traj, feedback, &pos_cmd, &vel_cmd);
                active = 1;
            } else {
                compute_admittance_pd(params, state, traj, feedback, &pos_cmd, &vel_cmd);
                active = 2;
            }
        }
        break;
        
        /* ========== 模式4: 自适应占空比混合 ========== */
        case COMP_MODE_HYBRID_ADAPT:
        {
            float t_switch = (1.0f - params->switching.duty) * params->switching.Ts;
            
            if (state->tau < t_switch) {
                compute_impedance_pd(params, state, traj, feedback, &pos_cmd, &vel_cmd);
                active = 1;
            } else {
                compute_admittance_pd(params, state, traj, feedback, &pos_cmd, &vel_cmd);
                active = 2;
            }
        }
        break;
        
        /* ========== 模式5: 虚拟代理 ========== */
        case COMP_MODE_PROXY:
            compute_proxy_pd(params, state, traj, feedback, &pos_cmd, &vel_cmd);
            active = state->active_mode;
            break;
        
        default:
            pos_cmd = traj->pos;
            vel_cmd = traj->vel;
            active = 0;
            break;
    }
    
    // 更新时间
    state->t_total += params->dt;
    state->active_mode = active;
    
    // 输出期望位置和速度给底层PD控制器
    output->pos_cmd = pos_cmd;
    output->vel_cmd = vel_cmd;
    output->mode_active = active;
    
    return active;
}

void Compliance_SetImpedance(ComplianceParams_t *params, 
                             float Md, float Dd, float Kd)
{
    if (!params) return;
    params->impedance.Md = Md;
    params->impedance.Dd = Dd;
    params->impedance.Kd = Kd;
}

void Compliance_SetAdmittance(ComplianceParams_t *params,
                              float Ma, float Ba, float Ka)
{
    if (!params) return;
    params->admittance.Ma = Ma;
    params->admittance.Ba = Ba;
    params->admittance.Ka = Ka;
}

void Compliance_SetDutyCycle(ComplianceParams_t *params, float duty)
{
    if (!params) return;
    params->switching.duty = CLAMP(duty, 0.0f, 1.0f);
}

/* ==================== 私有函数实现 ==================== */

/**
 * @brief 阻抗控制 - 位置/速度输出
 * 
 * 原理: 通过导纳滤波器将外力转换为位置偏移量
 *       Md * ẍ_offset + Dd * ẋ_offset + Kd * x_offset = Fext
 *       输出: pos_cmd = traj_pos + x_offset
 *             vel_cmd = traj_vel + v_offset
 * 
 * 阻抗模式特点: 参数设置使系统表现出目标阻抗特性
 */
static void compute_impedance_pd(const ComplianceParams_t *p,
                                 ComplianceState_t *s,
                                 const DesiredTrajectory_t *traj,
                                 const RobotFeedback_t *fb,
                                 float *pos_cmd, float *vel_cmd)
{
    float Md = p->impedance.Md;
    float Dd = p->impedance.Dd;
    float Kd = p->impedance.Kd;
    
    if (fabsf(Md) < EPS) Md = EPS;
    
    // 导纳滤波器: Md*a + Dd*v + Kd*x = Fext
    // 计算位置偏移量的加速度
    s->a_imp = (fb->force - Dd * s->v_imp - Kd * s->x_imp) / Md;
    
    // 积分得到速度和位置偏移量
    s->v_imp += s->a_imp * p->dt;
    s->x_imp += s->v_imp * p->dt;
    
    // 输出 = 期望轨迹 + 偏移量
    *pos_cmd = traj->pos + s->x_imp;
    *vel_cmd = traj->vel + s->v_imp;
}

/**
 * @brief 导纳控制 - 位置/速度输出
 * 
 * 原理: 导纳滤波器直接生成修正后的轨迹
 *       Ma * (ẍ_adm - ẍd) + Ba * (ẋ_adm - ẋd) + Ka * (x_adm - xd) = Fext
 *       输出: pos_cmd = x_adm
 *             vel_cmd = v_adm
 */
static void compute_admittance_pd(const ComplianceParams_t *p,
                                  ComplianceState_t *s,
                                  const DesiredTrajectory_t *traj,
                                  const RobotFeedback_t *fb,
                                  float *pos_cmd, float *vel_cmd)
{
    float Ma = p->admittance.Ma;
    float Ba = p->admittance.Ba;
    float Ka = p->admittance.Ka;
    
    if (fabsf(Ma) < EPS) Ma = EPS;
    
    // 相对于期望轨迹的偏差
    float e_pos = s->x_adm - traj->pos;
    float e_vel = s->v_adm - traj->vel;
    
    // 导纳动力学: Ma*(a_adm - ad) + Ba*(v_adm - vd) + Ka*(x_adm - xd) = Fext
    s->a_adm = (fb->force - Ba * e_vel - Ka * e_pos) / Ma + traj->acc;
    
    // 积分更新状态
    s->v_adm += s->a_adm * p->dt;
    s->x_adm += s->v_adm * p->dt;
    
    // 直接输出导纳滤波器的位置和速度
    *pos_cmd = s->x_adm;
    *vel_cmd = s->v_adm;
}

/**
 * @brief 虚拟代理控制 - 位置/速度输出
 * 
 * 原理: 通过虚拟质量块(代理)解耦机器人与环境
 *       代理与机器人通过虚拟弹簧-阻尼耦合
 *       输出代理的位置和速度作为机器人的期望
 */
static void compute_proxy_pd(const ComplianceParams_t *p,
                             ComplianceState_t *s,
                             const DesiredTrajectory_t *traj,
                             const RobotFeedback_t *fb,
                             float *pos_cmd, float *vel_cmd)
{
    float Kc = p->proxy.Kc;
    float Bc = p->proxy.Bc;
    float Mv = p->proxy.Mv;
    
    if (fabsf(Mv) < EPS) Mv = EPS;
    
    // 机器人与代理之间的耦合力 (代理感受到的)
    float F_couple = Kc * (fb->pos - s->x_proxy) + Bc * (fb->vel - s->v_proxy);
    
    // 代理上的等效驱动力
    float F_drive = 0.0f;
    float t_switch = (1.0f - p->switching.duty) * p->switching.Ts;
    
    if (s->tau < t_switch) {
        // 阻抗阶段: 代理跟踪期望轨迹，考虑耦合力
        float Md = p->impedance.Md;
        float Dd = p->impedance.Dd;
        float Kd = p->impedance.Kd;
        
        if (fabsf(Md) < EPS) Md = EPS;
        
        float e_pos = s->x_proxy - traj->pos;
        float e_vel = s->v_proxy - traj->vel;
        
        // 代理的目标阻抗响应
        F_drive = -Kd * e_pos - Dd * e_vel + Mv * traj->acc;
        
        s->active_mode = 1;
    } else {
        // 导纳阶段: 代理根据耦合力调整
        float Ma = p->admittance.Ma;
        float Ba = p->admittance.Ba;
        float Ka = p->admittance.Ka;
        
        if (fabsf(Ma) < EPS) Ma = EPS;
        
        float e_pos = s->x_proxy - traj->pos;
        float e_vel = s->v_proxy - traj->vel;
        
        // 导纳响应
        F_drive = (F_couple - Ba * e_vel - Ka * e_pos) / Ma * Mv + Mv * traj->acc;
        
        s->active_mode = 2;
    }
    
    // 代理动力学: Mv * a_proxy = F_drive + F_couple
    s->a_proxy = (F_drive + F_couple) / Mv;
    s->v_proxy += s->a_proxy * p->dt;
    s->x_proxy += s->v_proxy * p->dt;
    
    // 输出代理的位置和速度作为期望
    *pos_cmd = s->x_proxy;
    *vel_cmd = s->v_proxy;
}

/**
 * @brief 正数取模
 */
static float fmod_positive(float x, float y)
{
    if (fabsf(y) < EPS) return 0.0f;
    float r = fmodf(x, y);
    return (r < 0.0f) ? (r + y) : r;
}

