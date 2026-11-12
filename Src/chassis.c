#include "chassis.h"
#include "gimbal.h"
#include "robot_config.h"
#include "motor_dji.h"
#include "pid.h"

/* 四个轮子的安装方向。机械装反了就把 robot_config.h 里对应的 SIGN_xx 改成 -1 */
static const float kSign[4] = { SIGN_FL, SIGN_FR, SIGN_RL, SIGN_RR };

static PID_t s_pid[4];

#define TWO_PI  6.28318531f

/* 跑飞保护标志。0 = 正常，1 = 这一拍在卸力（底盘不再自转，平移照给）。
 * 由 |err| 超门限置位、跌回 CHASSIS_FOLLOW_FAULT_CLEAR 以内自动清除；
 * Chassis_Stop() 也会清一次。 */
static uint8_t s_follow_fault;

/* ==================== 角度工具 ==================== */

/** @brief 角度差折到 (−π, π]，否则过 0/2π 交界时会跳 360°
 *
 *  和 gimbal.c 里那个 Angle_Wrap 完全等价，但那边是 static。
 *  为了一个 5 行的纯数学函数去改 gimbal.h 的接口不划算，这里各自留一份。 */
static float Wrap_Pi(float d)
{
    while (d >  TWO_PI * 0.5f) d -= TWO_PI;
    while (d < -TWO_PI * 0.5f) d += TWO_PI;
    return d;
}

/* ==================== 初始化 / 停止 ==================== */

void Chassis_Init(void)
{
    for (int i = 0; i < 4; i++) {
        PID_Init(&s_pid[i],
                 CHASSIS_SPEED_KP, CHASSIS_SPEED_KI, 0.0f,
                 CHASSIS_CUR_MAX * 0.5f,    /* 积分限幅取输出限幅的一半 */
                 CHASSIS_CUR_MAX);
    }

    s_follow_fault = 0;
}

void Chassis_Stop(void)
{
    for (int i = 0; i < 4; i++) {
        PID_Reset(&s_pid[i]);
    }

    s_follow_fault = 0;   /* 停车顺带解闩锁，见上面那个注释 */

    Motor_3508_Stop();
}

/* ==================== 麦轮运动学（手动 / 跟随共用）====================
 *
 * 把"底盘三轴速度"翻译成四个轮子的电流，两个任务只在这之前分叉：
 *   手动档：wz 来自拨轮；
 *   跟随档：wz 来自跟随环。
 * 分叉点收在这里，是为了让"四轮解算 + 每轮 PID + 安装方向"只有一份，
 * 以后调麦轮参数（半径、轮距、符号）不用两个地方同时改。 */
static void Chassis_Drive(float vx, float vy, float wz)
{
    float g = 1 / WHEEL_RADIUS;
    float k = HALF_TRACK_X + HALF_TRACK_Y;

    float target[4];
    float cur[4];

    target[0] = g * ( vx - vy - k * wz );   /* 左前 */
    target[1] = g * ( vx + vy + k * wz );   /* 右前 */
    target[2] = g * ( vx + vy - k * wz );   /* 左后 */
    target[3] = g * ( vx - vy + k * wz );   /* 右后 */

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

/* ==================== 模式一：纯手动 ==================== */

void Chassis_Task(const Remote_t *rc)
{
    float vx = rc->ly * CHASSIS_V_MAX;      /* 左摇杆上下 → 前后 */
    float vy = rc->lx * CHASSIS_V_MAX * SIGN_RC_VY;  /* 左摇杆左右 → 左右（方向见 SIGN_RC_VY） */
    float wz = rc->wheel * CHASSIS_W_MAX;   /* 拨轮 → 自转 */

    Chassis_Drive(vx, vy, wz);
}

/* ==================== 模式二：底盘跟随云台 ====================
 *
 * 目标：rel（云台相对底盘角）恒定在 YAW_ENC_HEAD，也就是车头那 90° 位置。
 *
 * 和 gimbal.c 里 s_yaw_follow 的区别不是"写法不同"，是**基准不同**：
 * 那边以底盘为基准、拿 6020 去追；这边以云台的绝对指向为基准、拿轮子去追。
 * 所以本模式下云台必须保持"锁大地"（app.c 恒定传 Gimbal_SetYawFollow(0)），
 * 否则两个环闭环同一个 rel，会互相抢。
 *
 * ★ 拨轮（rc->wheel）在这个模式下**故意不参与**：
 *   如果把拨轮直接加进 wz，跟随环会把它当扰动顶回来 —— 你转底盘，
 *   环检测到 rel 变了，又把底盘转回去，手感像"拨轮坏了"。
 *   要原地自转请推 yaw 摇杆：云台转 → 底盘跟着转，这才是本模式的自转方式。
 *
 * ★ 平移（vx/vy）不受影响，可以边平移边跟随。
 */
void Chassis_Follow_Task(const Remote_t *rc)
{
    float vx = rc->ly * CHASSIS_V_MAX;
    float vy = rc->lx * CHASSIS_V_MAX * SIGN_RC_VY;   /* 和 Chassis_Task 用同一个符号：
                                                       * 两个模式的平移手感必须一致，
                                                       * 否则切档时车身会朝反方向窜一下 */

    /* ---- ① 位置误差：车头角 − 当前相对角 ----
     * 必须 Wrap：rel 在 0/2π 交界处会跳，不折的话误差会瞬间变成 ±360°，
     * 跟随环一脚把底盘甩出去。 */
    float rel    = Gimbal_GetYawRel();
    float err    = Wrap_Pi(YAW_ENC_HEAD - rel);
    float err_db = err;
    float wz_cmd;

    /* ---- ② 死区 ----
     * 没有死区的话，云台停住后底盘会在目标附近反复小幅来回蹭：
     * 轮子有摩擦、有减速箱间隙，误差进不到 0 就一直在修。
     * 1° 之内直接当作 0，让它彻底安静。 */
    if (err_db < CHASSIS_FOLLOW_DB && err_db > -CHASSIS_FOLLOW_DB) err_db = 0.0f;

    /* ---- ③ 跑飞保护（临时卸力 + 自动恢复）----
     * |err| 超门限只有两种可能：SIGN_CHASSIS_FOLLOW 反了在正反馈跑飞，
     * 或者底盘已经把云台顶过了挡块。两种都必须立刻停转，否则挡块会一直吃力矩。
     * ★ 必须带回差自动恢复：门限若是一锁到底，正常操作中把 yaw 拧过门限一次
     *   就会让底盘永久不随动，且看不出任何异常。真跑飞时 err 只会一路顶到
     *   机械极限，不会自己回到 CLEAR 以内，所以自动恢复不吃亏。 */
    if (err > CHASSIS_FOLLOW_FAULT_ANGLE || err < -CHASSIS_FOLLOW_FAULT_ANGLE)
    {
        s_follow_fault = 1;
    }
    else if (err < CHASSIS_FOLLOW_FAULT_CLEAR && err > -CHASSIS_FOLLOW_FAULT_CLEAR)
    {
        s_follow_fault = 0;
    }

    if (s_follow_fault)
    {
        wz_cmd = 0.0f;           /* 只停自转，平移仍然给，人还能把车开走 */
    }
    else
    {
        float wz_g = Gimbal_GetYawRate();

        /* ★ 前馈源死区（见 robot_config.h 的 CHASSIS_FOLLOW_FF_DB）：
         *   云台没在转的时候，这个值只剩陀螺残留，直接前馈给底盘就是"慢慢蹭"。
         *   必须挡在乘 FF 之前，挡在之后没意义（FF 就是 1.0 也照样放大残留）。 */
        if (wz_g < CHASSIS_FOLLOW_FF_DB && wz_g > -CHASSIS_FOLLOW_FF_DB) wz_g = 0.0f;

        float w = CHASSIS_FOLLOW_FF * wz_g
                - CHASSIS_FOLLOW_KP * err_db;

        if (w >  CHASSIS_FOLLOW_W_MAX) w =  CHASSIS_FOLLOW_W_MAX;
        if (w < -CHASSIS_FOLLOW_W_MAX) w = -CHASSIS_FOLLOW_W_MAX;

        /* ★ 输出死区：位置已在死区内、前馈也被挡住时，这里就是 0；
         *   万一还有更小的残余（KP×err_db 的尾数），一并清掉。
         *   0.03 rad/s 比轮子摩擦死区还小，清掉不影响任何真实运动。 */
        if (w < CHASSIS_FOLLOW_W_DB && w > -CHASSIS_FOLLOW_W_DB) w = 0.0f;

        wz_cmd = w * SIGN_CHASSIS_FOLLOW;
    }

    Chassis_Drive(vx, vy, wz_cmd);
}
