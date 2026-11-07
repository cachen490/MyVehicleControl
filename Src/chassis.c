#include "chassis.h"
#include "robot_config.h"
#include "motor_dji.h"
#include "pid.h"

/* 四个轮子的安装方向。机械装反了就把 robot_config.h 里对应的 SIGN_xx 改成 -1 */
static const float kSign[4] = { SIGN_FL, SIGN_FR, SIGN_RL, SIGN_RR };

static PID_t s_pid[4];

void Chassis_Init(void)
{
    for (int i = 0; i < 4; i++) {
        PID_Init(&s_pid[i],
                 CHASSIS_SPEED_KP, CHASSIS_SPEED_KI, 0.0f,
                 CHASSIS_CUR_MAX * 0.5f,    /* 积分限幅取输出限幅的一半 */
                 CHASSIS_CUR_MAX);
    }
}

void Chassis_Stop(void)
{
    for (int i = 0; i < 4; i++) {
        PID_Reset(&s_pid[i]);
    }
    Motor_3508_Stop();
}

void Chassis_Task(const Remote_t *rc)
{
    /* 1. 遥控 → 车体三个速度 */
    float vx = rc->ly * CHASSIS_V_MAX;      /* 左摇杆上下 → 前后 */
    float vy = rc->lx * CHASSIS_V_MAX;      /* 左摇杆左右 → 左右 */
    float wz = rc->wheel * CHASSIS_W_MAX;   /* 拨轮 → 自转 */

    float g = COS45 / WHEEL_RADIUS;
    float k = HALF_TRACK_X + HALF_TRACK_Y;

    float target[4];
    float cur[4];

    /* 2. 逆解：车体速度 → 四个轮子的角速度 */
    target[0] = g * ( vx - vy - k * wz );   /* 左前 */
    target[1] = g * ( vx + vy + k * wz );   /* 右前 */
    target[2] = g * ( vx + vy - k * wz );   /* 左后 */
    target[3] = g * ( vx - vy + k * wz );   /* 右后 */

    /* 3. 每个轮子一个独立的速度环 */
    for (int i = 0; i < 4; i++)
    {
        /* 反馈是转子 rad/s，除以减速比变成轮子 rad/s，再乘安装方向 */
        float meas = g_motor3508[i].speed / GEAR_RATIO_3508 * kSign[i];

        /* PID 一直工作在"正数 = 前进"的统一坐标里，安装方向在外面乘，
         * 这样四个轮子的 PID 参数可以完全一样 */
        cur[i] = PID_Calc(&s_pid[i], target[i], meas, CTRL_DT) * kSign[i];
    }

    Motor_3508_SetCurrent(cur);
}
