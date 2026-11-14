#ifndef MOTOR_DJI_H
#define MOTOR_DJI_H

#include "main.h"

/* 反馈原始值 → 物理量 */
#define C620_RAW_PER_AMP    (16384.0f / 20.0f)
#define GM6020_RAW_PER_VOLT (25000.0f / 24.0f)

#define RPM_TO_RAD          (6.28318531f / 60.0f)
#define ECD_TO_RAD          (6.28318531f / 8192.0f)

typedef struct {
    int16_t  ecd;       /* 编码器值，0~8191 */
    float    speed;     /* 转速，rad/s（减速箱输出轴） */
    uint16_t lost_ms;   /* 距上次收到反馈的毫秒数 */
} Motor_t;

extern Motor_t g_motor3508[4];      /* 底盘四个 3508 */
extern Motor_t g_motor6020[2];      /* 云台两个 6020，下标用 M_YAW / M_PITCH */

void Motor_Init(void);
void Motor_Update(void);

void Motor_3508_SetCurrent(const float current[4]);
void Motor_3508_Stop(void);

void Motor_6020_SetVoltage(const float volt[2]);
void Motor_6020_Stop(void);

#endif
