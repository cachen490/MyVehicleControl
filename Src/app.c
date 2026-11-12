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

#define SW_RIGHT    0   /* g_rc.s 的下标：0 = 右拨杆 */

/* 右拨杆三个位置对应的工作模式。三个档位的数值在 robot_config.h 里 */
typedef enum {
    MODE_STOP = 0,      /* 中位：整车停 */
    MODE_LOCK_EARTH,    /* 上  ：云台锁大地，底盘手控（云台不跟底盘转） */
    MODE_FOLLOW         /* 下  ：跟随。谁追谁由 YAW_FOLLOW_IMPL 决定，
                         *       0（默认）= 底盘追云台，1 = 云台追底盘 */
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

#if YAW_FOLLOW_IMPL == 0
    /* ---- 底盘追云台（默认）----
     * ★ 这里恒定传 0，不是写错：跟随时该出力的是**轮子**，不是 6020。
     *   云台只负责"锁大地"（陀螺速度环把 ω 压到 0），绝对指向由摇杆决定；
     *   底盘看到 rel 偏了就自己转过去贴合。
     *   如果这里传 1，云台的 yaw 外环会把 rel 也闭环一遍 —— 两个环抢同一个量，
     *   而且它会把目标锁在"进模式那一刻的 rel"上，和跟随环的目标（车头角）冲突。 */
    Gimbal_SetYawFollow(0);
#else
    Gimbal_SetYawFollow(mode == MODE_FOLLOW);
#endif

    /* 底盘两套任务互斥，二选一 —— 它们都要算出 wz，同时跑就是后者覆盖前者 */
#if YAW_FOLLOW_IMPL == 0
    if (mode == MODE_FOLLOW) Chassis_Follow_Task(&g_rc);
    else                     Chassis_Task(&g_rc);
#else
    Chassis_Task(&g_rc);
#endif

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
