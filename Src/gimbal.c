#include "gimbal.h"
#include "robot_config.h"
#include "motor_dji.h"
#include "imu.h"
#include "pid.h"
#include <math.h>

static PID_t s_pid_yaw;
static PID_t s_pid_pitch;

static float s_wz_f;
static float s_wy_f;

static float s_pitch_target;

#define TWO_PI  6.28318531f

static uint8_t s_yaw_follow;    /* 0 = 锁大地，1 = 跟随底盘 */
static uint8_t s_yaw_realign;   /* 进跟随时：0 = 原地接管，1 = 转回车头 */
static float   s_yaw_rel_tgt;   /* 跟随模式下"云台相对底盘"的目标角，rad */
static float   s_yaw_t_cmd;     /* 限速之后的目标角速度，rad/s */

/**
 * @brief 云台相对底盘的角度，0 ~ 2π
 *
 * 用 ecd（单圈绝对值 0~8191）而不是 angle（上电起累计）：
 * ecd 由 6020 内部霍尔决定，跟"上电时云台停在哪"无关，每次开机都一致；
 * angle 每次上电都从 0 开始，做不了绝对的位置环。
 */
static float Yaw_RelAngle(void)
{
    return (float)g_motor6020[M_YAW].ecd * ECD_TO_RAD * SIGN_ENC_YAW;
}

/** @brief 把角度差折到 (-π, π]，否则经过 0/2π 交界时会突然跳 360° */
static float Angle_Wrap(float d)
{
    while (d >  TWO_PI * 0.5f) d -= TWO_PI;
    while (d < -TWO_PI * 0.5f) d += TWO_PI;
    return d;
}

void Gimbal_SetYawFollow(uint8_t on)
{
    if (on && !s_yaw_follow)                 /* 只在"进跟随"的那一拍做事 */
    {
        /* 目标角从哪起步：0 = 原地接管（不跳变），1 = 转回车头。
         * 用普通 if 而不是 #if：宏只是"上电默认值"，
         * 运行中还能用 Gimbal_SetYawRealign() 改，而且只有一份代码，看得清 */
        if (s_yaw_realign) s_yaw_rel_tgt = YAW_ENC_HEAD;
        else               s_yaw_rel_tgt = Yaw_RelAngle();

        PID_Reset(&s_pid_yaw);               /* 别把上一个模式的积分带过来 */
    }

    s_yaw_follow = on;
}

void Gimbal_SetYawRealign(uint8_t on)
{
    s_yaw_realign = on;
}

uint8_t Gimbal_IsYawFollow(void)
{
    return s_yaw_follow;
}

void Gimbal_Init(void)
{
    /* ---- 内环（速度环）：反馈是角速度(rad/s)、输出是电压(V)，
     * kp 的量纲是 V/(rad/s)。6020 空载 24V 约 300rpm ≈ 31rad/s；
     * kp=0.3 意味着 1rad/s 的误差给 0.3V。
     * 起调 0.3，ki/kd 先给 0（为什么 pitch 可能要加 ki 见文档 6.7） ---- */
    PID_Init(&s_pid_yaw,   0.30f, GIMBAL_RATE_KI, 0.0f, GIMBAL_RATE_I_MAX, GIMBAL_VOLT_MAX);
    PID_Init(&s_pid_pitch, 0.30f, GIMBAL_RATE_KI, 0.0f, GIMBAL_RATE_I_MAX, GIMBAL_VOLT_MAX);

    s_wz_f = 0.0f;
    s_wy_f = 0.0f;

    s_pitch_target = 0.0f;

    s_yaw_follow  = 0;
    s_yaw_realign = (uint8_t)YAW_FOLLOW_REALIGN;   /* 宏只当"上电默认值"用 */
    s_yaw_rel_tgt = 0.0f;
    s_yaw_t_cmd   = 0.0f;
}

void Gimbal_Stop(void)
{
    PID_Reset(&s_pid_yaw);
    PID_Reset(&s_pid_pitch);
    s_wz_f = 0.0f;
    s_wy_f = 0.0f;
    s_pitch_target = 0.0f;      /* 下次恢复先把目标拉回水平，别接着上次的角度冲过去 */

    s_yaw_follow  = 0;          /* 掉线恢复后一律从"锁大地"重新开始， */
    s_yaw_t_cmd   = 0.0f;       /* 下次进跟随时会重新同步目标角，不会拿着旧目标暴冲 */

    Motor_6020_Stop();
}

/**
 * @brief 云台 1ms 任务
 *
 * yaw  : 两种模式，共用同一个速度环（反馈都是陀螺 Z 轴 s_wz_f）：
 *        锁大地 → 没有外环，目标角速度 = 摇杆，松杆为 0 = 相对大地不转；
 *        跟随   → 加一个外环 P，反馈是 6020 编码器给的"云台相对底盘角"。
 *        换模式只是换"目标从哪来"，内环一个字都不用改。
 * pitch: 串级。外环把"角度误差"变成"目标角速度"，内环再把目标角速度变成电压。
 *        外环用 Mahony 解出来的绝对角（跟得住水平），内环用陀螺（响应快、不抖）。
 *
 * ★ 三个符号必须一致，否则内环变正反馈，一上电就顶死：
 *   1. SIGN_GYRO_PITCH：手往上抬头时 IMU_GetGyroY()*SIGN_GYRO_PITCH 应为正
 *   2. SIGN_PITCH_ABS ：手往上抬头时 IMU_GetPitch()*SIGN_PITCH_ABS 应变大
 *   3. SIGN_PIT       ：给正电压时云台真的往"抬头"方向走
 */
void Gimbal_Task(const Remote_t *rc)
{
    float v[2];
    float yaw_t, rate_t, pit_abs;

    /* ================= yaw：两种模式共用一个速度环 ================= */
    s_wz_f += GYRO_LPF_ALPHA * (IMU_GetGyroZ() * SIGN_GZ - s_wz_f);

    if (!s_yaw_follow)
    {
        /* ---- 锁大地：没有外环。
         * 陀螺测的是惯性系角速度，所以目标给 0 等于"相对大地不转"。
         * 底盘怎么转都会被速度环当成扰动顶回去，这就是"不跟随"。---- */
        yaw_t = rc->rx * GIMBAL_RATE_MAX;
    }
    else
    {
        /* ---- 跟随底盘：外环 P，反馈换成编码器的"相对底盘角" ----
         * 摇杆改的是"目标相对角"（和 pitch 外环同一个套路，不是改角速度） */
        s_yaw_rel_tgt = Angle_Wrap(s_yaw_rel_tgt + rc->rx * GIMBAL_RATE_MAX * CTRL_DT);

        yaw_t = YAW_FOLLOW_KP * Angle_Wrap(s_yaw_rel_tgt - Yaw_RelAngle());

        /* 外环输出就是内环的目标角速度，必须限制在机构能做到的范围里，
         * 否则一个 90° 的偏差会算出 12 rad/s 的目标，内环一直顶在限幅上 */
        if (yaw_t >  GIMBAL_RATE_MAX) yaw_t =  GIMBAL_RATE_MAX;
        if (yaw_t < -GIMBAL_RATE_MAX) yaw_t = -GIMBAL_RATE_MAX;
    }

    /* 目标角速度限速：模式切换、猛推摇杆都会让上面的 yaw_t 阶跃，
     * 直接喂给内环会听到"咚"一下，也会让 6020 瞬间吃到很大的力矩 */
    {
        float step = YAW_RATE_SLEW * CTRL_DT;
        float dy   = yaw_t - s_yaw_t_cmd;

        if (dy >  step) dy =  step;
        if (dy < -step) dy = -step;

        s_yaw_t_cmd += dy;
    }

    v[M_YAW] = PID_Calc(&s_pid_yaw, s_yaw_t_cmd, s_wz_f, CTRL_DT) * SIGN_YAW;

    /* ============ pitch 外环：摇杆累加的是"目标角"，不是角速度 ============ */
    s_pitch_target += rc->ry * GIMBAL_PITCH_RATE * CTRL_DT;

    /* 先钳目标，不让它跑到机构到不了的地方（否则内环会一直顶着限幅出力） */
    if (s_pitch_target > PITCH_ANGLE_MAX) s_pitch_target = PITCH_ANGLE_MAX;
    if (s_pitch_target < PITCH_ANGLE_MIN) s_pitch_target = PITCH_ANGLE_MIN;

    pit_abs = IMU_GetPitch() * SIGN_PITCH_ABS;

    /* 外环只用 P：角度差 → 目标角速度。
     * GIMBAL_PITCH_KP = 2.0 表示偏 10°(0.175rad) 就要 20°/s 的修正速度。
     * 太小会软绵绵回不到水平，太大会在水平位置附近来回荡 */
    rate_t = GIMBAL_PITCH_KP * (s_pitch_target - pit_abs);

    /* ============ pitch 内环：目标角速度 → 电压，反馈仍是陀螺 ============ */
    s_wy_f += GYRO_LPF_ALPHA * (IMU_GetGyroY() * SIGN_GYRO_PITCH - s_wy_f);

    v[M_PITCH] = PID_Calc(&s_pid_pitch, rate_t, s_wy_f, CTRL_DT) * SIGN_PIT;

    /* ============ 机械软限位 ============
     * 目标角上面已经钳过了，这里防的是"实际角度"跑飞（比如云台被人手掰过去、
     * 或者 Mahony 发散）。留 0.25rad ≈ 14° 余量，避免开局收敛过程误触发。
     * 注意：这里判的是绝对角，不再用 6020 的编码器角度 ——
     * 编码器角是相对量，上电位置不同读数不同，不能拿来当水平基准 */
    if (pit_abs < PITCH_ANGLE_MIN - 0.25f || pit_abs > PITCH_ANGLE_MAX + 0.25f)
    {
        v[M_PITCH] = 0.0f;
        PID_Reset(&s_pid_pitch);
        s_pitch_target = 0.0f;      /* 一起拉回水平，否则解除限位后会猛冲一下 */
    }

    Motor_6020_SetVoltage(v);
}
