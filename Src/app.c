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
    if (g_rc.lost_ms > REMOTE_LOST_MS) {
        return 0;
    }

    if (g_rc.s[SW_RIGHT] != SW_ENABLE) {
        return 0;
    }

    for (int i = 0; i < 4; i++) {
        if (g_motor3508[i].lost_ms > MOTOR_LOST_MS) return 0;
    }
    for (int i = 0; i < 2; i++) {
        if (g_motor6020[i].lost_ms > MOTOR_LOST_MS) return 0;
    }

    if (!IMU_IsOnline()) {
        return 0;
    }

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
    IMU_CalibrateGyro();

    Chassis_Init();
    Gimbal_Init();

    __HAL_DBGMCU_FREEZE_IWDG();
}

void App_Task_1ms(void)
{
    static int cnt = 0;
    float cur[4] = {0};
    float meas[4];

    Motor_Update();

    /* 只让 vx = 0.3 m/s，vy = wz = 0，持续 3 秒，然后停 2 秒 */
    cnt++;
    if (cnt < 3000) {
        float g = COS45 / WHEEL_RADIUS;
        for (int i = 0; i < 4; i++) {
            meas[i] = g_motor3508[i].speed / GEAR_RATIO_3508;
            /* 用 P 控制让轮子转到 0.3/0.076 ≈ 3.95 rad/s */
            cur[i] = (3.95f - meas[i]) * 0.35f;
        }
    } else if (cnt > 5000) {
        cnt = 0;            /* 循环重来，方便反复看 */
    }

    Motor_3508_SetCurrent(cur);
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
