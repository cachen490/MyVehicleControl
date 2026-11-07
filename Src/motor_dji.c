#include "motor_dji.h"
#include "robot_config.h"
#include "bsp_can.h"
#include <string.h>

Motor_t g_motor3508[4];
Motor_t g_motor6020[2];

/**
 * @brief 把一帧 8 字节解成一个电机的状态
 * @param raw_per_amp 该型号电机"实际电流"的换算系数
 */
static void Motor_Decode(Motor_t *m, const uint8_t *d, float raw_per_amp)
{
    int16_t ecd   = (int16_t)(((uint16_t)d[0] << 8) | d[1]);
    int16_t speed = (int16_t)(((uint16_t)d[2] << 8) | d[3]);
    int16_t curr  = (int16_t)(((uint16_t)d[4] << 8) | d[5]);
    int16_t diff;

    if (!m->valid) {
        m->last_ecd = ecd;
        m->valid = 1;
    }

    diff = (int16_t)(ecd - m->last_ecd);
    if (diff >  ECD_MAX / 2) diff -= (ECD_MAX + 1);
    if (diff < -ECD_MAX / 2) diff += (ECD_MAX + 1);

    m->total_ecd += diff;
    m->last_ecd   = ecd;
    m->ecd        = ecd;

    m->angle   = (float)m->total_ecd * ECD_TO_RAD;
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
            Motor_Decode(&g_motor3508[f.id - 0x201], f.data, C620_RAW_PER_AMP);
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

        buf[2 * i]     = (uint8_t)((uint16_t)raw >> 8);
        buf[2 * i + 1] = (uint8_t)((uint16_t)raw & 0xFF);
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
