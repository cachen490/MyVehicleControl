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

typedef enum {
    MODE_STOP = 0,      /* 停车档：全部断电 */
    MODE_LOCK_EARTH,    /* 锁大地档：云台不跟底盘，底盘手动 */
    MODE_FOLLOW         /* 跟随档：底盘跟着云台转 */

} RobotMode_t;

/* 读右拨杆判定档位；遥控掉线或档位不认识都按停车处理 */
static RobotMode_t Robot_GetMode(void)
{
    if (g_rc.lost_ms > REMOTE_LOST_MS) return MODE_STOP;

    if (g_rc.sw == SW_UP_VAL)   return MODE_LOCK_EARTH;
    if (g_rc.sw == SW_DOWN_VAL) return MODE_FOLLOW;

    return MODE_STOP;
}

/* 所有电机在线、IMU 在线才算硬件正常 */
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
    /* 拉高 CAN 收发器使能脚 */
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
    RobotMode_t mode;

    Motor_Update();
    Remote_Update();
    IMU_Update();

    mode = Robot_GetMode();

    /* 停车档或硬件异常：电机全部断电后直接返回 */
    if (mode == MODE_STOP || !Robot_Hardware_Ok())
    {
        Gimbal_Stop();
        Chassis_Stop();
        return;
    }

    /* 跟随档限制云台自己的转速，给底盘留出追赶余量 */
    Gimbal_SetYawRateCap(mode == MODE_FOLLOW ? GIMBAL_YAW_RATE_FOLLOW
                                             : GIMBAL_RATE_MAX);

    /* 只有锁大地档才用编码器锚定航向 */
    Gimbal_SetYawAnchor(mode != MODE_FOLLOW);

    if (mode == MODE_FOLLOW) Chassis_Follow_Task(&g_rc);
    else                     Chassis_Task(&g_rc);

    Gimbal_Task(&g_rc);
}

void App_Loop(void)
{
    static uint32_t last_tick = 0;

    if ((HAL_GetTick() - last_tick) >= 1)
    {
        last_tick = HAL_GetTick();

        App_Task_1ms();

        HAL_IWDG_Refresh(&hiwdg);
    }
}
