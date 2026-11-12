#include "motor_dji.h"
#include "robot_config.h"
#include "bsp_can.h"
#include <string.h>

Motor_t g_motor3508[4];
Motor_t g_motor6020[2];

/* ==================== 电机 ID → 逻辑位置 的映射 ====================
 * 这张表解决的问题：**物理电机设的 ID** 和 **代码认定的位置** 对不上。
 *
 * 下标是"逻辑位置"，值是该位置上电机【实际的 CAN ID（1~4）】：
 *     0 = 左前   1 = 右前   2 = 左后   3 = 右后
 *
 * chassis.c 里的 target[] / kSign[] / s_pid[] 只认逻辑下标，
 * 物理 ID 由这张表翻译，两边各管各的。改这片代码时不用再担心接线顺序。
 *
 * ★ 当前实车（2026-11-09 实测）：
 *       左前 = ID 2     右前 = ID 1     左后 = ID 3     右后 = ID 4
 *   左右两个前轮是反的，所以下面填 {2, 1, 3, 4}。
 *
 * ★ 如果以后用 C620 把 ID 重设成"左前1 右前2 左后3 右后4"，
 *   这张表就改回 {1, 2, 3, 4}（恒等映射）。
 *
 * ★ 填错的表现：推前进摇杆车往后走 / 原地打转 / 斜着走。
 *   每改一次这张表，务必按"单独给 index 0 一个电流，看是不是左前轮转"验证一遍。
 */
static const uint8_t kLogicMotorId[4] = { 2, 1, 3, 4 };

/* 反向表：CAN 槽位（0 = ID1、1 = ID2 …）→ 逻辑下标。
 * 由 kLogicMotorId 在 Motor_Init() 里自动推出来，**不要手填** ——
 * 正反两张表分别手填，早晚会填得不一致，而且这种 bug 极难看出来。 */
static uint8_t s_slotToLogic[4];

/**
 * @brief 把一帧 8 字节解成一个电机的状态
 * @param raw_per_amp 该型号电机"实际电流"的换算系数
 */
static void Motor_Decode(Motor_t *m, const uint8_t *d, float raw_per_amp)
{
    int16_t ecd   = (int16_t)(((uint16_t)d[0] << 8) | d[1]);
    int16_t speed = (int16_t)(((uint16_t)d[2] << 8) | d[3]);
    int16_t curr  = (int16_t)(((uint16_t)d[4] << 8) | d[5]);

    m->ecd     = ecd;
    m->speed   = (float)speed * RPM_TO_RAD;
    m->current = (float)curr / raw_per_amp;
    m->temp    = d[6];
    m->lost_ms = 0;
}

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
        if (f.id >= 0x201 && f.id <= 0x204) {
            /* 0x201→槽位0（ID1）、0x202→槽位1（ID2）…
             * 再经 s_slotToLogic 翻成 chassis.c 认的逻辑下标 */
            Motor_Decode(&g_motor3508[s_slotToLogic[f.id - 0x201]], f.data, C620_RAW_PER_AMP);
        }
        else if (f.id == 0x204 + ID_YAW_6020) {
            Motor_Decode(&g_motor6020[0], f.data, GM6020_RAW_PER_AMP);
        }
        else if (f.id == 0x204 + ID_PITCH_6020) {
            Motor_Decode(&g_motor6020[1], f.data, GM6020_RAW_PER_AMP);
        }
    }
}

void Motor_Init(void)
{
    memset(g_motor3508, 0, sizeof(g_motor3508));
    memset(g_motor6020, 0, sizeof(g_motor6020));

    /* 先置成恒等映射（槽位 i ↔ 逻辑 i），再按 kLogicMotorId 覆盖一遍。
     * 这样只要 kLogicMotorId 是一个合法的 1~4 排列，s_slotToLogic 一定是对的。 */
    for (int s = 0; s < 4; s++) s_slotToLogic[s] = (uint8_t)s;
    for (int i = 0; i < 4; i++) s_slotToLogic[kLogicMotorId[i] - 1] = (uint8_t)i;
}

void Motor_3508_SetCurrent(const float current[4])
{
    uint8_t buf[8];

    for (int i = 0; i < 4; i++)
    {
        float raw_f = current[i] * C620_RAW_PER_AMP;
        if (raw_f >  16384.0f) raw_f =  16384.0f;
        if (raw_f < -16384.0f) raw_f = -16384.0f;

        int16_t raw = (int16_t)raw_f;

        /* ★ 0x200 这一帧里，字节位置对应的是【ID 顺序】不是逻辑顺序：
         *     buf[0..1] → ID1、buf[2..3] → ID2、buf[4..5] → ID3、buf[6..7] → ID4
         *   所以 current[逻辑下标] 必须按 kLogicMotorId 反查一次槽位，
         *   否则左前和右前的电流会交叉喂到对方电机上（车会原地打转）。 */
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
