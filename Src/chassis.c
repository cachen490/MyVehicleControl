#include "chassis.h"
#include "gimbal.h"
#include "robot_config.h"
#include "motor_dji.h"
#include "pid.h"

#include <math.h>

static const float kSign[4] = { SIGN_FL, SIGN_FR, SIGN_RL, SIGN_RR };  /* 四个轮子的装向符号 */

static PID_t s_pid[4];      /* 每个轮子一个速度环 */

#define TWO_PI  6.28318531f

static float   s_follow_ss;     /* 进档软起步系数，0→1 */
static float   s_follow_i;      /* 跟随环位置积分项，rad/s */

/* 把角度折到 (−π, π] */
static float Wrap_Pi(float d)
{
    while (d >  TWO_PI * 0.5f) d -= TWO_PI;
    while (d < -TWO_PI * 0.5f) d += TWO_PI;
    return d;
}

void Chassis_Init(void)
{
    for (int i = 0; i < 4; i++) {
        PID_Init(&s_pid[i],
                 CHASSIS_SPEED_KP, CHASSIS_SPEED_KI, 0.0f,
                 CHASSIS_CUR_MAX * 0.5f,
                 CHASSIS_CUR_MAX);
    }

    s_follow_ss    = 0.0f;
    s_follow_i     = 0.0f;
}

void Chassis_Stop(void)
{
    for (int i = 0; i < 4; i++) {
        PID_Reset(&s_pid[i]);
    }

    s_follow_ss    = 0.0f;
    s_follow_i     = 0.0f;

    Motor_3508_Stop();
}

/* 麦轮逆解 + 四轮速度环，输出电流给 3508 */
static void Chassis_Drive(float vx, float vy, float wz)
{
    float g = 1 / WHEEL_RADIUS;
    float k = HALF_TRACK_X + HALF_TRACK_Y;

    float target[4];    /* 各轮目标线速度，m/s */
    float cur[4];       /* 各轮输出电流，A */

    target[0] = g * ( vx - vy - k * wz );
    target[1] = g * ( vx + vy + k * wz );
    target[2] = g * ( vx + vy - k * wz );
    target[3] = g * ( vx - vy + k * wz );

    for (int i = 0; i < 4; i++)
    {
        float meas = g_motor3508[i].speed / GEAR_RATIO_3508 * kSign[i];

        cur[i] = PID_Calc(&s_pid[i], target[i], meas, CTRL_DT) * kSign[i];
    }

    Motor_3508_SetCurrent(cur);
}

/* 由四轮转速反算底盘自转角速度，rad/s */
float Chassis_GetWzMeas(void)
{
    float g = 1.0f / WHEEL_RADIUS;
    float k = HALF_TRACK_X + HALF_TRACK_Y;
    float w[4];
    float s;

    for (int i = 0; i < 4; i++)
    {
        w[i] = g_motor3508[i].speed / GEAR_RATIO_3508 * kSign[i];
    }

    s = -w[0] + w[1] - w[2] + w[3];

    return s / (4.0f * k * g);
}

/* 手动档：摇杆直接给平移，拨轮给自转，不含任何跟随 */
void Chassis_Task(const Remote_t *rc)
{
    float vx = rc->ly * CHASSIS_V_MAX;
    float vy = rc->lx * CHASSIS_V_MAX * SIGN_RC_VY;
    float wz = rc->wheel * CHASSIS_W_MAX;

    s_follow_ss = 0.0f;
    s_follow_i  = 0.0f;

    Chassis_Drive(vx, vy, wz);
}

/* 跟随档：平移来自摇杆，自转由"云台相对底盘角 rel"闭环算出来 */
void Chassis_Follow_Task(const Remote_t *rc)
{
    float vx = rc->ly * CHASSIS_V_MAX;
    float vy = rc->lx * CHASSIS_V_MAX * SIGN_RC_VY;

    float rel    = Gimbal_GetYawRel();
    float err    = Wrap_Pi(YAW_ENC_HEAD - rel);   /* 车头角 − 当前相对角 */
    float err_db = err;

    /* 位置死区 */
    if (err_db < CHASSIS_FOLLOW_DB && err_db > -CHASSIS_FOLLOW_DB) err_db = 0.0f;

    /* 软起步：进档第一拍按偏差决定要不要从 0 抬到 1 */
    if (s_follow_ss == 0.0f)
    {
        if (fabsf(err) < (CHASSIS_FOLLOW_SS_SKIP * DEG2RAD))
            s_follow_ss = 1.0f;
        else
            s_follow_ss = CTRL_DT / CHASSIS_FOLLOW_SS_T;
    }
    else if (s_follow_ss < 1.0f)
    {
        s_follow_ss += CTRL_DT / CHASSIS_FOLLOW_SS_T;
        if (s_follow_ss > 1.0f) s_follow_ss = 1.0f;
    }

    float wz_m = Gimbal_GetYawRate();       /* 云台实测角速度，rad/s */
    float ff   = CHASSIS_FOLLOW_FF;
    float w_ff = ff * wz_m;                 /* 前馈项 */
    float w_meas = Chassis_GetWzMeas() * SIGN_CHASSIS_FOLLOW;

    /* 阻尼项：只压 (轮速 − 前馈) 的差，稳态时为 0 */
    float w_d = - CHASSIS_FOLLOW_KD * s_follow_ss * (w_meas - wz_m);

    /* 减速侧钳位 */
    {
        float d_lim = ff * fabsf(wz_m);

        if (w_d < -d_lim) w_d = -d_lim;
    }

    float w_pd = - CHASSIS_FOLLOW_KP * s_follow_ss * err_db + w_d;

    float w_sum = w_ff + w_pd + s_follow_i;
    uint8_t sat = (w_sum >= CHASSIS_FOLLOW_W_MAX) ||
                  (w_sum <= -CHASSIS_FOLLOW_W_MAX);

    /* 位置积分：上限跟着云台转速指令走，饱和或进死区时停止积分 */
    float wz_c  = Gimbal_GetYawRateCmd();
    float i_cap = CHASSIS_FOLLOW_I_RATE * fabsf(wz_c);

    if (i_cap > CHASSIS_FOLLOW_I_MAX) i_cap = CHASSIS_FOLLOW_I_MAX;

    if ((s_follow_ss >= 1.0f) && (err_db != 0.0f) && (!sat))
    {
        s_follow_i -= CHASSIS_FOLLOW_KI * err_db * CTRL_DT;
    }

    if (s_follow_i >  i_cap) s_follow_i =  i_cap;
    if (s_follow_i < -i_cap) s_follow_i = -i_cap;

    float w = w_ff + w_pd + s_follow_i;

    /* 静止闸门：误差在死区且云台没在转 → 输出干净的 0，避免底盘微动 */
    if ((err_db == 0.0f) && (fabsf(w_ff) < CHASSIS_FOLLOW_QUIET_W))
    {
        w = 0.0f;
    }

    if (w >  CHASSIS_FOLLOW_W_MAX) w =  CHASSIS_FOLLOW_W_MAX;
    if (w < -CHASSIS_FOLLOW_W_MAX) w = -CHASSIS_FOLLOW_W_MAX;

    Chassis_Drive(vx, vy, w * SIGN_CHASSIS_FOLLOW);
}
