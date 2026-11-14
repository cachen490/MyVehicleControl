#include "motor_dji.h"
#include "robot_config.h"
#include "bsp_can.h"
#include <string.h>

Motor_t g_motor3508[4];
Motor_t g_motor6020[2];

static const uint8_t kLogicMotorId[4] = { 2, 1, 3, 4 };   /* 逻辑序号 → 3508 的 CAN ID 尾号 */
static uint8_t s_slotToLogic[4];                          /* CAN ID 尾号 → g_motor3508 下标 */

/* 解析一帧 7 字节反馈：ecd / 转速 */
static void Motor_Decode(Motor_t *m, const uint8_t *d)
{
    int16_t ecd   = (int16_t)(((uint16_t)d[0] << 8) | d[1]);
    int16_t speed = (int16_t)(((uint16_t)d[2] << 8) | d[3]);

    m->ecd     = ecd;
    m->speed   = (float)speed * RPM_TO_RAD;
    m->lost_ms = 0;
}

/* 每 1ms 调一次：超时计数递加 + 取走 CAN 收到的所有反馈帧 */
void Motor_Update(void)
{
    CAN_RxFrame_t f;
    uint32_t i;

    for (i = 0; i < 4; i++) {
        if (g_motor3508[i].lost_ms < 0xFFFF) g_motor3508[i].lost_ms++;
    }
    for (i = 0; i < 2; i++) {
        if (g_motor6020[i].lost_ms < 0xFFFF) g_motor6020[i].lost_ms++;
    }

    while (BSP_CAN_PopFrame(&f))
    {
        /* 0x201~0x204 是四个 3508，0x205/0x206 是两个 6020 */
        if (f.id >= 0x201 && f.id <= 0x204) {
            Motor_Decode(&g_motor3508[s_slotToLogic[f.id - 0x201]], f.data);
        }
        else if (f.id == 0x204 + ID_YAW_6020) {
            Motor_Decode(&g_motor6020[0], f.data);
        }
        else if (f.id == 0x204 + ID_PITCH_6020) {
            Motor_Decode(&g_motor6020[1], f.data);
        }
    }
}

void Motor_Init(void)
{
    memset(g_motor3508, 0, sizeof(g_motor3508));
    memset(g_motor6020, 0, sizeof(g_motor6020));

    for (int s = 0; s < 4; s++) s_slotToLogic[s] = (uint8_t)s;
    for (int i = 0; i < 4; i++) s_slotToLogic[kLogicMotorId[i] - 1] = (uint8_t)i;
}

/* 四个 3508 电流打包进 0x200，按 CAN ID 尾号排槽位 */
void Motor_3508_SetCurrent(const float current[4])
{
    uint8_t buf[8];

    for (int i = 0; i < 4; i++)
    {
        float raw_f = current[i] * C620_RAW_PER_AMP;
        if (raw_f >  16384.0f) raw_f =  16384.0f;
        if (raw_f < -16384.0f) raw_f = -16384.0f;

        int16_t raw = (int16_t)raw_f;

        int slot = (int)kLogicMotorId[i] - 1;

        buf[2 * slot]     = (uint8_t)((uint16_t)raw >> 8);
        buf[2 * slot + 1] = (uint8_t)((uint16_t)raw & 0xFF);
    }

    BSP_CAN_SendStd(0x200, buf, 8);
}

void Motor_3508_Stop(void)
{
    float zero[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    Motor_3508_SetCurrent(zero);
}

/* 两个 6020 电压打包进 0x1FF，占前两个槽位 */
void Motor_6020_SetVoltage(const float volt[2])
{
    uint8_t buf[8] = {0};
    for (int i = 0; i < 2; i++)
    {
        float raw_f = volt[i] * GM6020_RAW_PER_VOLT;
        if (raw_f >  30000.0f) raw_f =  30000.0f;
        if (raw_f < -30000.0f) raw_f = -30000.0f;

        int16_t raw = (int16_t)raw_f;
        buf[2 * i]     = (uint8_t)((uint16_t)raw >> 8);
        buf[2 * i + 1] = (uint8_t)((uint16_t)raw & 0xFF);
    }

    BSP_CAN_SendStd(0x1FF, buf, 8);
}

void Motor_6020_Stop(void)
{
    float zero[2] = {0.0f, 0.0f};
    Motor_6020_SetVoltage(zero);
}
