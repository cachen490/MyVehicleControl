#include "gimbal.h"
#include "robot_config.h"
#include "motor_dji.h"
#include "imu.h"
#include "pid.h"
#include <math.h>

static PID_t s_pid_yaw;
static PID_t s_pid_pitch;

/* 低通的状态量（一阶 IIR） */
static float s_wz_f;
static float s_wy_f;

/* pitch 串级控制的外环状态：目标角（rad，相对水平，0 = 水平） */
static float s_pitch_target;

void Gimbal_Init(void)
{
    /* ---- 内环（速度环）：反馈是角速度(rad/s)、输出是电压(V)，
     * kp 的量纲是 V/(rad/s)。6020 空载 24V 约 300rpm ≈ 31rad/s；
     * kp=0.3 意味着 1rad/s 的误差给 0.3V。
     * 起调 0.3，ki/kd 先给 0（为什么 pitch 可能要加 ki 见文档 6.7） ---- */
    PID_Init(&s_pid_yaw,   0.30f, 0.0f, 0.0f, 0.0f, GIMBAL_VOLT_MAX);
    PID_Init(&s_pid_pitch, 0.30f, 0.0f, 0.0f, 0.0f, GIMBAL_VOLT_MAX);

    s_wz_f = 0.0f;
    s_wy_f = 0.0f;

    /* 目标角初值 0 = 水平。"保持水平"就是让这个 0 一直不动，
     * 摇杆只在需要低头/抬头时临时改它。
     * 注意这里不能写 IMU_GetPitch()：Gimbal_Init() 可能在 IMU 收敛前调用，
     * 那样等于把当时的歪斜当成"水平"记下来了 */
    s_pitch_target = 0.0f;
}

void Gimbal_Stop(void)
{
    PID_Reset(&s_pid_yaw);
    PID_Reset(&s_pid_pitch);
    s_wz_f = 0.0f;
    s_wy_f = 0.0f;
    s_pitch_target = 0.0f;      /* 下次恢复先把目标拉回水平，别接着上次的角度冲过去 */
    Motor_6020_Stop();
}

/**
 * @brief 云台 1ms 任务
 *
 * yaw  : 纯速度环。要的是"转多快"，反馈用陀螺 Z 轴，目标角速度为 0 = 锁住不转。
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

    /* ================= yaw：速度环 ================= */
    s_wz_f += GYRO_LPF_ALPHA * (IMU_GetGyroZ() * SIGN_GZ - s_wz_f);

    yaw_t    = rc->rx * GIMBAL_RATE_MAX;
    v[M_YAW] = PID_Calc(&s_pid_yaw, yaw_t, s_wz_f, CTRL_DT) * SIGN_YAW;

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
