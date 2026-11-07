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

void Gimbal_Init(void)
{
    /* 反馈是角速度(rad/s)、输出是电压(V)，所以 kp 的量纲是 V/(rad/s)。
     * 6020 空载 24V 约 300rpm ≈ 31rad/s；kp=0.3 意味着 1rad/s 的误差给 0.3V。
     * 起调 0.3，ki/kd 先给 0（为什么 pitch 可能要加 ki 见文档 6.7） */
    PID_Init(&s_pid_yaw,   0.30f, 0.0f, 0.0f, 0.0f, GIMBAL_VOLT_MAX);
    PID_Init(&s_pid_pitch, 0.30f, 0.0f, 0.0f, 0.0f, GIMBAL_VOLT_MAX);

    s_wz_f = 0.0f;
    s_wy_f = 0.0f;
}

void Gimbal_Stop(void)
{
    PID_Reset(&s_pid_yaw);
    PID_Reset(&s_pid_pitch);
    s_wz_f = 0.0f;
    s_wy_f = 0.0f;
    Motor_6020_Stop();
}

void Gimbal_Task(const Remote_t *rc)
{
    float v[2];
    float wz_raw, wy_raw;
    float yaw_t, pitch_t;

    /* ---- 1. 陀螺仪角速度，乘符号统一到"正 = 逆时针 / 抬头" ---- */
    wz_raw = IMU_GetGyroZ() * SIGN_GZ;
    wy_raw = IMU_GetGyroY() * SIGN_GY;

    /* ---- 2. 一阶低通。速度环最怕噪声：噪声直接变成电压抖动 → 电机啸叫 ---- */
    s_wz_f += GYRO_LPF_ALPHA * (wz_raw - s_wz_f);
    s_wy_f += GYRO_LPF_ALPHA * (wy_raw - s_wy_f);

    /* ---- 3. 目标角速度：只有遥控，没有前馈项 ---- */
    yaw_t   = rc->rx * GIMBAL_RATE_MAX;
    pitch_t = rc->ry * GIMBAL_RATE_MAX;

    /* ---- 4. 速度环：反馈 = 陀螺仪 ---- */
    v[M_YAW] = PID_Calc(&s_pid_yaw, yaw_t, s_wz_f, CTRL_DT) * SIGN_YAW;

#if GYRO_ON_YAW_PLATFORM
    /* A 板跟着 yaw 转：陀螺仪本来就在云台坐标系里，直接当反馈 */
    v[M_PITCH] = PID_Calc(&s_pid_pitch, pitch_t, s_wy_f, CTRL_DT) * SIGN_PIT;
#else
    /* A 板在定子上：pitch 只能用电机转速，再补上底盘投影过来的分量 */
    {
        float a   = g_motor6020[M_YAW].angle * SIGN_YAW;
        float cwx = IMU_GetGyroX() * SIGN_GX;
        float own = g_motor6020[M_PITCH].speed * SIGN_PIT;            /* rad/s */
        float fb  = own + (sinf(a) * cwx - cosf(a) * wy_raw);         /* 世界系角速度 */

        v[M_PITCH] = PID_Calc(&s_pid_pitch, pitch_t, fb, CTRL_DT) * SIGN_PIT;
    }
#endif

    /* ---- 5. Pitch 软限位：出界就断电压，防堵转烧线圈 ---- */
    if (g_motor6020[M_PITCH].angle < PITCH_ANGLE_MIN ||
        g_motor6020[M_PITCH].angle > PITCH_ANGLE_MAX)
    {
        v[M_PITCH] = 0.0f;
        PID_Reset(&s_pid_pitch);
    }

    Motor_6020_SetVoltage(v);
}
