#include "app.h"
#include "main.h"
#include "robot_config.h"
#include "bsp_can.h"
#include "motor_dji.h"
#include "remote.h"
#include "imu.h"
#include "chassis.h"
#include "gimbal.h"
#include "iwdg.h"

#define SW_RIGHT    0   /* g_rc.s 的下标：0 = 右拨杆，1 = 左拨杆 */
#define SW_LEFT     1

/* 右拨杆三个位置对应的工作模式。三个档位的数值在 robot_config.h 里 */
typedef enum {
    MODE_STOP = 0,      /* 中位：整车停 */
    MODE_LOCK_EARTH,    /* 上  ：云台锁大地，不跟底盘转 */
    MODE_FOLLOW         /* 下  ：云台跟随底盘转 */
} RobotMode_t;

/**
 * @brief 把右拨杆翻译成工作模式
 *
 * 上 = 锁大地、中 = 停、下 = 跟随。
 * 遥控掉线、档位数值对不上、拨杆正在两档之间抖动，一律按停车处理 ——
 * 所有"看不懂"的情况都倒向最安全的一侧。
 */
static RobotMode_t Robot_GetMode(void)
{
    if (g_rc.lost_ms > REMOTE_LOST_MS) return MODE_STOP;

    if (g_rc.s[SW_RIGHT] == SW_UP_VAL)   return MODE_LOCK_EARTH;
    if (g_rc.s[SW_RIGHT] == SW_DOWN_VAL) return MODE_FOLLOW;

    return MODE_STOP;
}

/**
 * @brief 左拨杆翻译成"进跟随时要不要转回车头"
 *
 * 上      → 1：目标角取车头方向 YAW_ENC_HEAD，切进跟随时云台自己转正
 * 中 / 下 → 0：目标角取当前相对角，原地接管，云台一点都不会动
 *
 * 只有"上"这一档会让云台自动转，中、下都是安全的静止档 ——
 * 故意这么设计：只有你明确把开关推到上，才允许云台自己甩一下。
 *
 * 注意它只在【进入跟随的那一拍】被读取（Gimbal_SetYawFollow 的上升沿），
 * 所以运行中扳它不会立刻生效，也不会半路把云台拽走。
 * 想让"正在进行的这次跟随"回正：
 *     左拨杆推到上 → 右拨杆扳回中位 → 右拨杆再扳到下位。
 */
static uint8_t Robot_GetYawRealign(void)
{
    return (g_rc.s[SW_LEFT] == SW_UP_VAL) ? 1u : 0u;
}

/** @brief 硬件是否齐全。缺一个就整车停，不留"一半在跑"的状态 */
static uint8_t Robot_Hardware_Ok(void)
{
    for (int i = 0; i < 4; i++) {
        if (g_motor3508[i].lost_ms > MOTOR_LOST_MS) return 0;
    }
    for (int i = 0; i < 2; i++) {
        if (g_motor6020[i].lost_ms > MOTOR_LOST_MS) return 0;
    }
    if (!IMU_IsOnline()) return 0;

    return 1;
}

void App_Init(void)
{
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_5, GPIO_PIN_SET);
    HAL_Delay(100);

    Motor_Init();
    BSP_CAN_Init();
    Remote_Init();
    IMU_Init();
    IMU_CalibrateGyro();        /* 约 0.5 秒，这段时间车必须静止 */

    Chassis_Init();
    Gimbal_Init();

    __HAL_DBGMCU_FREEZE_IWDG();
}

/**
 * @brief 1ms 周期的全部控制逻辑
 *
 * 顺序有讲究：
 *   1. 先取数据（电机反馈 / 遥控 / IMU），三个都必须每 1ms 刷一次；
 *   2. 再判模式，看不懂就停；
 *   3. 最后才允许出力和发指令。
 *
 * ★ IMU_Update() 以前只在上电标定时被调用过，主循环里没人调 ——
 *   那样陀螺和姿态永远是上电那一瞬间的值，云台看着"不动"其实是瞎的。
 */
void App_Task_1ms(void)
{
    RobotMode_t mode;

    Motor_Update();
    Remote_Update();
    IMU_Update();

    mode = Robot_GetMode();

    if (mode == MODE_STOP || !Robot_Hardware_Ok())
    {
        Gimbal_Stop();
        Chassis_Stop();
        return;
    }

    /* ★ 这两行的顺序不能换：
     *   Gimbal_SetYawFollow() 在"进入跟随的那一拍"会去读 s_yaw_realign，
     *   所以必须先把左拨杆的状态写进去，再告诉它要不要跟随。
     *   写反的后果：扳上左拨杆进跟随，用的还是上一拍（或上电时的）旧值 ——
     *   看着就是"回正开关失效"，而且很难查。
     *
     * 两个函数都是幂等的：值没变就什么也不做，可以每周期无脑调。 */
    Gimbal_SetYawRealign(Robot_GetYawRealign());
    Gimbal_SetYawFollow(mode == MODE_FOLLOW);

    Chassis_Task(&g_rc);
    Gimbal_Task(&g_rc);
}

void App_Loop(void)
{
    static uint32_t last_tick = 0;

    if ((HAL_GetTick() - last_tick) >= 1)
    {
        last_tick = HAL_GetTick();

        App_Task_1ms();

        /* 喂狗。放在任务的最后一行：
         * 如果 App_Task_1ms 卡死了，就永远走不到这里，超时后看门狗复位。
         *
         * ★ 这里必须先判空：当前 .ioc 里 IWDG 还没激活（Step 7 才开），
         *   此时 hiwdg.Instance 是 NULL，直接调 HAL_IWDG_Refresh() 会往
         *   地址 0x000000AA 写东西 → 立刻 HardFault。等 Step 7 把 CubeMX 的
         *   IWDG 勾回来、重新 Generate，MX_IWDG_Init() 会填好 Instance，
         *   这段判断就自动开始真正喂狗，不用改代码。 */
        if (hiwdg.Instance != NULL) {
            HAL_IWDG_Refresh(&hiwdg);
        }
    }
}

/* ==========================================================================
 * Step 3 的验证代码（已停用，需要再测轮子时把 App_Task_1ms 换回这一段）：
 *
 *     static int cnt = 0;
 *     cnt++;
 *     if (cnt < 3000) {                    // 只让 vx = 0.3 m/s，持续 3 秒
 *         float cur[4];
 *         for (int i = 0; i < 4; i++) {
 *             float meas = g_motor3508[i].speed / GEAR_RATIO_3508;
 *             cur[i] = (3.95f - meas) * 0.35f;
 *         }
 *         Motor_3508_SetCurrent(cur);
 *     } else if (cnt > 5000) {
 *         cnt = 0;
 *     }
 * ========================================================================== */
