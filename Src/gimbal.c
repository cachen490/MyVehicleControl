#include "gimbal.h"
#include "robot_config.h"
#include "motor_dji.h"
#include "imu.h"
#include "pid.h"
#include "chassis.h"
#include <math.h>

static PID_t s_pid_yaw;
static PID_t s_pid_pitch;

static float s_wz_f;        /* yaw 陀螺角速度低通值，rad/s */
static float s_wp_f;        /* pitch 陀螺角速度低通值，rad/s */

#define TWO_PI  6.28318531f

static float   s_yaw_t_cmd;         /* yaw 目标角速度（限速后），rad/s */
static float   s_yaw_rate_cap;      /* yaw 目标角速度上限，rad/s */

static float   s_yaw_world;         /* 云台大地系航向估计 θh，rad */
static float   s_yaw_tgt;           /* 云台大地系目标角 θt，rad */
static float   s_yaw_imu_last;      /* 上一拍陀螺航向 */
static float   s_yaw_rel_last;      /* 上一拍相对角 */
static float   s_yaw_anchor;        /* 锚定基准 (θh − rel) */
static uint8_t s_yaw_tgt_armed;     /* 1=θt 已对齐过 */
static uint8_t s_yaw_init_done;

static uint8_t s_anchor_en = 1;     /* 编码器锚定总开关 */

static float   s_yaw_pos_i;         /* 位置环积分项，rad/s */

static uint16_t s_stall_cnt;        /* 堵转连续计数，ms */
static uint8_t  s_stall_guard;      /* 1=判定堵转，冻住积分 */

static float    s_lim_sign;         /* 指令正方向对应机械角增大的符号，由 YAW_LIM_DIR_FORCE 钉死 */
static float    s_lim_room;         /* 到行程边界的剩余余量，rad */
static float    s_lim_outsign;      /* "往外"方向在指令上的符号 */

static float    s_end_lo;           /* 行程窗口下端点（机械角） */
static float    s_end_hi;           /* 行程窗口上端点（机械角） */

static uint8_t  s_edge_on;          /* 1=已切到"贴边底盘系保持" */
static float    s_off_hold;         /* 贴边保持的目标机械角 */

static uint8_t  s_ff_on;            /* 上一拍摇杆是否在推 */

/* 云台相对底盘的角，rad */
static float Yaw_RelAngle(void)
{
    return (float)g_motor6020[M_YAW].ecd * ECD_TO_RAD * SIGN_ENC_YAW;
}

static float Angle_Wrap(float d)
{
    while (d >  TWO_PI * 0.5f) d -= TWO_PI;
    while (d < -TWO_PI * 0.5f) d += TWO_PI;
    return d;
}

/* 大地系航向 θh：底盘静止时用编码器锚定，转动时靠陀螺积分 */
static void Yaw_WorldUpdate(void)
{
    float y   = IMU_GetYaw() * SIGN_GZ;
    float rel = Yaw_RelAngle();
    float drel;

    if (!s_yaw_init_done)
    {
        s_yaw_world     = rel;
        s_yaw_imu_last  = y;
        s_yaw_rel_last  = rel;
        s_yaw_anchor    = 0.0f;
        s_yaw_init_done = 1;
    }

    s_yaw_world += Angle_Wrap(y - s_yaw_imu_last);
    s_yaw_imu_last = y;

    drel = Angle_Wrap(rel - s_yaw_rel_last);
    s_yaw_rel_last = rel;

    /* 编码器跳变（上电首帧）：只重设基准，不做修正 */
    if (fabsf(drel) > YAW_ANCHOR_JUMP)
    {
        s_yaw_anchor = s_yaw_world - rel;
        return;
    }

    /* 底盘和云台都"没在转"时，把 (θh − rel) 一阶拉回基准，修陀螺零漂 */
    if (s_anchor_en &&
        YAW_ANCHOR_ENABLE &&
        fabsf(Chassis_GetWzMeas()) < YAW_ANCHOR_CH_W &&
        fabsf(s_wz_f)              < YAW_ANCHOR_G_W)
    {
        float d = Angle_Wrap((s_yaw_world - rel) - s_yaw_anchor);

        s_yaw_world -= YAW_ANCHOR_K * CTRL_DT * d;
    }
    else
    {
        s_yaw_anchor = s_yaw_world - rel;
    }
}

float Gimbal_GetYawRel(void)
{
    return Yaw_RelAngle();
}

float Gimbal_GetYawRate(void)
{
    return s_wz_f;
}

float Gimbal_GetYawRateCmd(void)
{
    return s_yaw_t_cmd;
}

void Gimbal_SetYawRateCap(float cap)
{
    if (cap <= 0.0f) return;

    if (cap > GIMBAL_RATE_MAX) cap = GIMBAL_RATE_MAX;

    s_yaw_rate_cap = cap;
}

/* 机械角：直接来自 ecd，不乘任何符号，范围 [0, 2π) */
static float Yaw_MechOffset(void)
{
    float m = (float)g_motor6020[M_YAW].ecd * ECD_TO_RAD - YAW_MECH_OFFSET_RAD;

    while (m <  0.0f)   m += TWO_PI;
    while (m >= TWO_PI) m -= TWO_PI;

    return m;
}

void Gimbal_SetYawAnchor(uint8_t on)
{
    s_anchor_en = on ? 1u : 0u;
}

void Gimbal_Init(void)
{
    PID_Init(&s_pid_yaw,   YAW_RATE_KP, GIMBAL_RATE_KI, 0.0f, GIMBAL_RATE_I_MAX, GIMBAL_VOLT_MAX);
    PID_Init(&s_pid_pitch, PITCH_KP, PITCH_KI, 0.0f, PITCH_I_MAX, GIMBAL_VOLT_MAX);

    s_wz_f = 0.0f;
    s_wp_f = 0.0f;

    s_anchor_en    = 1;
    s_yaw_t_cmd    = 0.0f;
    s_yaw_rate_cap = GIMBAL_RATE_MAX;

    s_yaw_world     = 0.0f;
    s_yaw_tgt       = 0.0f;
    s_yaw_imu_last  = 0.0f;
    s_yaw_rel_last  = 0.0f;
    s_yaw_anchor    = 0.0f;
    s_yaw_tgt_armed = 0;
    s_yaw_init_done = 0;

    /* 限位方向：编译期钉死，不再在线自学习 */
    s_lim_sign = (YAW_LIM_DIR_FORCE > 0) ? 1.0f : -1.0f;

    s_lim_room    = TWO_PI * 0.5f;
    s_lim_outsign = 1.0f;

    s_end_lo = 0.0f;
    s_end_hi = YAW_MECH_SPAN_RAD;

    s_edge_on    = 0;
    s_off_hold   = 0.0f;
    s_ff_on      = 0;
    s_yaw_pos_i  = 0.0f;
    s_stall_cnt  = 0;
    s_stall_guard = 0;
}

void Gimbal_Stop(void)
{
    PID_Reset(&s_pid_yaw);
    PID_Reset(&s_pid_pitch);
    s_yaw_pos_i   = 0.0f;
    s_stall_cnt   = 0;
    s_stall_guard = 0;
    s_wz_f = 0.0f;
    s_wp_f = 0.0f;

    s_yaw_t_cmd = 0.0f;

    s_yaw_rate_cap = GIMBAL_RATE_MAX;

    s_yaw_tgt_armed = 0;

    Motor_6020_Stop();
}

void Gimbal_Task(const Remote_t *rc)
{
    float v[2];
    float yaw_t, pit_abs;

    /* 陀螺 Z 轴低通 */
    s_wz_f += GYRO_LPF_ALPHA * (IMU_GetGyroZ() * SIGN_GZ - s_wz_f);

    Yaw_WorldUpdate();

    float ff = rc->rx * SIGN_RC_YAW_LOCK * s_yaw_rate_cap;

    /* 堵转判定 */
    if ((fabsf(s_yaw_t_cmd) > YAW_STALL_CMD) &&
        (fabsf(s_wz_f)     < YAW_STALL_W))
    {
        if (s_stall_cnt < 0xFFFFu) s_stall_cnt++;
    }
    else
    {
        s_stall_cnt = 0;
    }

    s_stall_guard = (s_stall_cnt > (uint16_t)YAW_STALL_MS) ? 1u : 0u;

    /* 大地系行程窗口：θt 只允许在软限位内侧活动 */
    float win_mid  = (s_end_lo + s_end_hi) * 0.5f;
    float win_half = (s_end_hi - s_end_lo) * 0.5f - YAW_SOFT_MARGIN;

    uint8_t just_rel = 0;

    if (!s_yaw_tgt_armed)
    {
        s_yaw_tgt       = s_yaw_world;
        s_yaw_tgt_armed = 1;
        s_edge_on       = 0;
        s_ff_on         = 0;
        s_yaw_pos_i     = 0.0f;
        PID_Reset(&s_pid_yaw);
    }

    s_yaw_tgt += ff * CTRL_DT;

    /* 摇杆回中那一拍：把 θt 对齐到 θh，不留下"领跑"出来的债 */
    {
        uint8_t ff_on = (ff != 0.0f) ? 1u : 0u;

        if (s_ff_on && !ff_on)
        {
            just_rel  = 1;
            s_yaw_tgt = s_yaw_world;
        }
        s_ff_on = ff_on;
    }

    /* θt 超前量单向钳位：只限制"推杆侧"，不搬动被外力造成的偏差 */
    {
        float lag = Angle_Wrap(s_yaw_tgt - s_yaw_world);

        if (ff > 0.0f && lag >  YAW_TGT_LAG_MAX) s_yaw_tgt = s_yaw_world + YAW_TGT_LAG_MAX;
        if (ff < 0.0f && lag < -YAW_TGT_LAG_MAX) s_yaw_tgt = s_yaw_world - YAW_TGT_LAG_MAX;
    }

    /* 限位判定：算余量，并决定要不要切到贴边保持 */
    {
        float off_now  = Yaw_MechOffset();
        float d_cur    = Angle_Wrap(off_now - win_mid);
        float room     = win_half - fabsf(d_cur);
        float out_sign = (d_cur >= 0.0f) ? s_lim_sign : -s_lim_sign;
        float lo       = win_mid - win_half;
        float hi       = win_mid + win_half;

        s_lim_room    = room;
        s_lim_outsign = out_sign;

        if (!s_edge_on)
        {
            if (room < YAW_EDGE_ENTER)
            {
                s_edge_on  = 1;
                s_off_hold = (out_sign > 0.0f) ? hi : lo;

                /* 交接点不许离当前位置太远，避免进档那一拍指令跳变 */
                {
                    float dev = Angle_Wrap(s_off_hold - off_now);

                    if (dev >  YAW_EDGE_ENTER) s_off_hold = off_now + YAW_EDGE_ENTER;
                    if (dev < -YAW_EDGE_ENTER) s_off_hold = off_now - YAW_EDGE_ENTER;
                    if (s_off_hold < lo) s_off_hold = lo;
                    if (s_off_hold > hi) s_off_hold = hi;
                }

                s_pid_yaw.i_term = 0.0f;
                s_yaw_pos_i      = 0.0f;
            }
        }
        else if (room > YAW_LIM_EXIT)
        {
            s_edge_on = 0;
            s_yaw_tgt = s_yaw_world;
            s_pid_yaw.i_term = 0.0f;
            s_yaw_pos_i      = 0.0f;
        }

        /* 贴边保持：目标搬进编码器域（底盘系），速度按 √(2·a·e) 规划 */
        if (s_edge_on)
        {
            float e_off;
            float sign_d = (d_cur >= 0.0f) ? 1.0f : -1.0f;

            s_off_hold += s_lim_sign * ff * CTRL_DT;

            if (just_rel) s_off_hold = off_now;

            /* 松杆后目标不许继续往外跑 */
            if ((ff == 0.0f) && ((off_now - s_off_hold) * sign_d < 0.0f))
            {
                s_off_hold = off_now;
            }

            if (s_off_hold < lo) s_off_hold = lo;
            if (s_off_hold > hi) s_off_hold = hi;

            e_off = Angle_Wrap(s_off_hold - off_now);

            if      (e_off >  YAW_EDGE_DB) e_off -= YAW_EDGE_DB;
            else if (e_off < -YAW_EDGE_DB) e_off += YAW_EDGE_DB;
            else                           e_off  = 0.0f;

            if      (e_off >  0.0f) yaw_t =  sqrtf(2.0f * YAW_LIM_ACC *   e_off) * s_lim_sign;
            else if (e_off <  0.0f) yaw_t = -sqrtf(2.0f * YAW_LIM_ACC * (-e_off)) * s_lim_sign;
            else                    yaw_t = 0.0f;
        }
    }

    /* 锁大地位置环：在速度环外面套 θt 闭环 */
    if (!s_edge_on)
    {
        float e = Angle_Wrap(s_yaw_tgt - s_yaw_world);

        if      (e >  YAW_LOCK_DB) e -= YAW_LOCK_DB;
        else if (e < -YAW_LOCK_DB) e += YAW_LOCK_DB;
        else                       e  = 0.0f;

        if ((!s_stall_guard) && ((e > 0.0f) || (e < 0.0f)))
        {
            s_yaw_pos_i += YAW_LOCK_KI * e * CTRL_DT;

            if (s_yaw_pos_i >  YAW_LOCK_I_MAX) s_yaw_pos_i =  YAW_LOCK_I_MAX;
            if (s_yaw_pos_i < -YAW_LOCK_I_MAX) s_yaw_pos_i = -YAW_LOCK_I_MAX;
        }
        else
        {
            /* 进死区或堵转时把积分漏回 0，不能冻住，否则会有慢速极限环 */
            float tau = s_stall_guard ? YAW_STALL_I_TAU : YAW_LOCK_I_TAU;

            s_yaw_pos_i -= s_yaw_pos_i * (CTRL_DT / tau);
        }

        yaw_t = ff + YAW_LOCK_KP * e + s_yaw_pos_i;
    }

    /* 限位墙：只压"往外"的一半，往里永远放行 */
    if (!s_edge_on)
    {
        float room     = s_lim_room;
        float out_sign = s_lim_outsign;
        float u        = yaw_t * out_sign;

        {
            float re, u_max;

            if      (room >  YAW_LIM_DB) re = room - YAW_LIM_DB;
            else if (room < -YAW_LIM_DB) re = room + YAW_LIM_DB;
            else                         re = 0.0f;

            if (re <= 0.0f)
            {
                u_max = 0.0f;
            }
            else
            {
                u_max = sqrtf(2.0f * YAW_LIM_ACC * re);
                if (u_max > s_yaw_rate_cap) u_max = s_yaw_rate_cap;
            }

            if (u > u_max)
            {
                yaw_t = u_max * out_sign;
                
                s_yaw_tgt = s_yaw_world;
            }
        }
    }

    if (yaw_t >  s_yaw_rate_cap) yaw_t =  s_yaw_rate_cap;
    if (yaw_t < -s_yaw_rate_cap) yaw_t = -s_yaw_rate_cap;

    /* 目标角速度限加速度，防阶跃 */
    {
        float step = YAW_RATE_SLEW * CTRL_DT;
        float dy   = yaw_t - s_yaw_t_cmd;

        if (dy >  step) dy =  step;
        if (dy < -step) dy = -step;

        s_yaw_t_cmd += dy;
    }

    /* yaw 速度内环 */
    s_pid_yaw.freeze = s_stall_guard;

    v[M_YAW] = PID_Calc(&s_pid_yaw, s_yaw_t_cmd, s_wz_f, CTRL_DT) * SIGN_YAW;

    if (s_stall_guard)
    {
        s_pid_yaw.i_term -= s_pid_yaw.i_term * (CTRL_DT / YAW_STALL_I_TAU);
    }

    /* pitch：目标恒为 PITCH_ANGLE_CENTER，只做姿态保持 */
    pit_abs = IMU_GetRoll() * SIGN_PITCH_ABS - PITCH_MOUNT_OFFSET;

    s_wp_f += GYRO_LPF_ALPHA * (IMU_GetGyroX() * SIGN_GYRO_PITCH - s_wp_f);

    {
        float err  = PITCH_ANGLE_CENTER - pit_abs;
        float pi;
        float duty;
        float d_term;

        PID_Calc(&s_pid_pitch, PITCH_ANGLE_CENTER, pit_abs, CTRL_DT);

        pi = s_pid_pitch.kp * err + s_pid_pitch.i_term;

        d_term = PITCH_KD * s_wp_f;
        if (d_term >  PITCH_D_MAX) d_term =  PITCH_D_MAX;
        if (d_term < -PITCH_D_MAX) d_term = -PITCH_D_MAX;

        duty = pi - d_term;

        if (duty >  GIMBAL_VOLT_MAX) duty =  GIMBAL_VOLT_MAX;
        if (duty < -GIMBAL_VOLT_MAX) duty = -GIMBAL_VOLT_MAX;

        v[M_PITCH] = duty * SIGN_PIT;
    }

    Motor_6020_SetVoltage(v);
}
