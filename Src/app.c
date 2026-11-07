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
#define SW_ENABLE   1   /* 使能位置。现场试出来：最上面一档是 1 还是 3 */

/**
 * @brief 安全检查。返回 1 才允许输出
 *
 * 四条，任何一条不满足就整车停。顺序按"越致命越靠前"
 */
static uint8_t Robot_Is_Safe(void)
{
    /* 1. 遥控必须在线。掉线了没人能控制它，必须停 */
    if (g_rc.lost_ms > REMOTE_LOST_MS) {
        return 0;
    }

    /* 2. 拨杆必须在使能位。这是"总开关"，
     *    上电默认不是使能位，必须手动拨过去 —— 防止上电就窜出去 */
    if (g_rc.s[SW_RIGHT] != SW_ENABLE) {
        return 0;
    }

    /* 3. 六个电机都要有反馈。某个电机掉线了还在给电流，
     *    说明我们发出去的东西没人接，行为不可预测 */
    for (int i = 0; i < 4; i++) {
        if (g_motor3508[i].lost_ms > MOTOR_LOST_MS) return 0;
    }
    for (int i = 0; i < 2; i++) {
        if (g_motor6020[i].lost_ms > MOTOR_LOST_MS) return 0;
    }

    /* 4. 陀螺仪必须在线。没陀螺仪就没自稳，
     *    云台会跟着车乱甩，不如不给电 */
    if (!IMU_IsOnline()) {
        return 0;
    }

    return 1;
}

void App_Init(void)
{
    /* 1. 先把 24V 打开。电调上电后需要约 100ms 自检 */
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_5, GPIO_PIN_SET);
    HAL_Delay(100);

    /* 2. 各模块初始化。顺序有讲究：
     *    CAN 要先启动，后面电机才能收发；
     *    IMU 里有 HAL_Delay，所以放在 1ms 任务开始之前 */
    Motor_Init();
    BSP_CAN_Init();
    Remote_Init();
    IMU_Init();
    IMU_CalibrateGyro();    /* 500ms：上电这一刻车必须是静止的 */

    /* 3. 控制器初始化（内部是 PID 参数） */
    Chassis_Init();
    Gimbal_Init();

    /* 4. 让看门狗在调试断点时暂停，不然你打断点它会一直复位。
     *    这一行只影响调试体验，不影响运行时行为 */
    __HAL_DBGMCU_FREEZE_IWDG();
}

void App_Task_1ms(void)
{
    /* 1. 输入：把这一毫秒内收到的所有帧都处理掉 */
    Motor_Update();
    Remote_Update();
    IMU_Update();

    /* 2. 判断：安全才输出，不安全就全部断电 */
    if (Robot_Is_Safe()) {
        Chassis_Task(&g_rc);
        Gimbal_Task(&g_rc);
    } else {
        Chassis_Stop();     /* 内部会 Reset PID，避免重新使能时积分跳变 */
        Gimbal_Stop();
    }
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
         *   地址 0x000000AA 写东西 → 立刻 HardFault。等你到 Step 7 把
         *   CubeMX 的 IWDG 勾回来、重新 Generate，MX_IWDG_Init() 会填好
         *   Instance，这段判断就自动开始真正喂狗，不用改代码。 */
        if (hiwdg.Instance != NULL) {
            HAL_IWDG_Refresh(&hiwdg);
        }
    }
}
