#ifndef MOTOR_DJI_H
#define MOTOR_DJI_H

#include "main.h"

/* C620 电调：CAN 原始值 ±16384 对应 ±20 A */
#define C620_RAW_PER_AMP    (16384.0f / 20.0f)

/* GM6020 电压控制：±30000 对应 ±24V，但用 25000/24 留一点余量，
 * 避免摇杆推满时长期顶在满量程上。跟框架工程保持一致。 */
#define GM6020_RAW_PER_VOLT (25000.0f / 24.0f)

/* GM6020 电流控制（0x1FE）：±16384 对应 ±3 A */
#define GM6020_RAW_PER_AMP  (16384.0f / 3.0f)

#define ECD_MAX             8191            /* 单圈编码器最大值 */
#define RPM_TO_RAD          (6.28318531f / 60.0f)   /* rpm → rad/s */
#define ECD_TO_RAD          (6.28318531f / 8192.0f) /* 编码器刻度 → rad */

/* 一个电机的全部运行状态。数组下标就是电机序号 */
typedef struct {
    int16_t  ecd;        /* 本帧单圈编码器 0~8191 */
    float    speed;      /* 转子角速度，rad/s */
    float    current;    /* 实际电流，A */
    uint8_t  temp;       /* 温度，摄氏度 */
    uint16_t lost_ms;    /* 距离上次收到反馈的毫秒数 */
} Motor_t;

extern Motor_t g_motor3508[4];   /* 下标 0~3 对应 ID 1~4（顺序按 Step 1.0 的点动表） */
extern Motor_t g_motor6020[2];   /* 下标 0 = yaw，1 = pitch（ID 见 robot_config.h） */

void Motor_Init(void);           /* 清零状态 */
void Motor_Update(void);         /* 1ms 调一次：取帧 + 解码 + 在线计时 */

/**
 * @brief 给 4 个 3508 发电流指令
 * @param current 目标电流，单位 A，顺序对应 ID 1~4
 */
void Motor_3508_SetCurrent(const float current[4]);

/** @brief 四个电机电流全给 0。注意：这等于"不出力"，不等于刹车 */
void Motor_3508_Stop(void);

/**
 * @brief 给 2 个 6020 发电压指令
 * @param volt 目标电压，单位 V，顺序对应 ID 1~2（ID 3/4 自动补 0）
 */
void Motor_6020_SetVoltage(const float volt[2]);

void Motor_6020_Stop(void);

#endif /* MOTOR_DJI_H */
