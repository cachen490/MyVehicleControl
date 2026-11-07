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
    /* 1. 大端合成三个 16 位数 */
    int16_t ecd   = (int16_t)(((uint16_t)d[0] << 8) | d[1]);
    int16_t speed = (int16_t)(((uint16_t)d[2] << 8) | d[3]);
    int16_t curr  = (int16_t)(((uint16_t)d[4] << 8) | d[5]);
    int16_t diff;

    /* 2. 第一帧没有"上一帧"可减，先把 last 对齐，否则会算出一个巨大的假角度 */
    if (!m->valid) {
        m->last_ecd = ecd;
        m->valid = 1;
    }

    /* 3. 编码器过零处理：这是新手最容易写错的地方
     *    编码器 0~8191 是一个环。上一帧 8190、这一帧 3，真实增量是 +5，
     *    直接相减得到 3-8190 = -8187，方向完全反了。
     *    判据：差值的绝对值超过半圈，就说明跨过了零点，要补一整圈。 */
    diff = (int16_t)(ecd - m->last_ecd);
    if (diff >  ECD_MAX / 2) diff -= (ECD_MAX + 1);
    if (diff < -ECD_MAX / 2) diff += (ECD_MAX + 1);

    /* 4. 累加 + 写回 */
    m->total_ecd += diff;
    m->last_ecd   = ecd;
    m->ecd        = ecd;

    /* 5. 物理量换算 */
    m->angle   = (float)m->total_ecd * ECD_TO_RAD;
    m->speed   = (float)speed * RPM_TO_RAD;
    m->current = (float)curr / raw_per_amp;
    m->temp    = d[6];
    m->lost_ms = 0;              /* 收到帧就清零在线计时 */
}

void Motor_Update(void)
{
    CAN_RxFrame_t f;
    uint32_t i;

    /* 1. 先把所有电机的"离线计时"加 1 毫秒。收到帧的那几个下面会被清零 */
    for (i = 0; i < 4; i++) {
        if (g_motor3508[i].lost_ms < 0xFFFF) g_motor3508[i].lost_ms++;
    }
    for (i = 0; i < 2; i++) {
        if (g_motor6020[i].lost_ms < 0xFFFF) g_motor6020[i].lost_ms++;
    }

    /* 2. 把队列里积压的帧全部处理掉（可能不止一帧） */
    while (BSP_CAN_PopFrame(&f))
    {
        if (f.id >= 0x201 && f.id <= 0x204) {
            Motor_Decode(&g_motor3508[f.id - 0x201], f.data, C620_RAW_PER_AMP);
        }
        /* 6020 的反馈帧 = 0x204 + ID。用宏而不是写死 0x205，
         * 这样 ID 改了只动 robot_config.h，这里不用碰 */
        else if (f.id == 0x204 + ID_YAW_6020) {
            Motor_Decode(&g_motor6020[0], f.data, GM6020_RAW_PER_AMP);
        }
        else if (f.id == 0x204 + ID_PITCH_6020) {
            Motor_Decode(&g_motor6020[1], f.data, GM6020_RAW_PER_AMP);
        }
        /* 其它 ID（比如裁判系统）忽略 */
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
        /* 1. 电流(A) → 原始值。先限幅，防止上层给的数把 int16 撑溢出 */
        float raw_f = current[i] * C620_RAW_PER_AMP;
        if (raw_f >  16384.0f) raw_f =  16384.0f;
        if (raw_f < -16384.0f) raw_f = -16384.0f;

        int16_t raw = (int16_t)raw_f;

        /* 2. 拆成两字节，高字节在前（大端）。写反了电机收到的就是别的数 */
        buf[2 * i]     = (uint8_t)((uint16_t)raw >> 8);
        buf[2 * i + 1] = (uint8_t)((uint16_t)raw & 0xFF);
    }

    /* 3. 0x200 这一帧同时管 ID 1~4 */
    BSP_CAN_SendStd(0x200, buf, 8);
}

void Motor_3508_Stop(void)
{
    float zero[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    Motor_3508_SetCurrent(zero);
}

void Motor_6020_SetVoltage(const float volt[2])
{
    /* 注意这里初始化成 {0}：ID 3、4 上没插电机，那两对字节必须给 0 */
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

    /* 0x1FF 这一帧管 ID 1~4 的 6020 */
    BSP_CAN_SendStd(0x1FF, buf, 8);
}

void Motor_6020_Stop(void)
{
    float zero[2] = {0.0f, 0.0f};
    Motor_6020_SetVoltage(zero);
}
