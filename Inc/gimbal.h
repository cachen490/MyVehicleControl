#ifndef GIMBAL_H
#define GIMBAL_H

#include "remote.h"

#define M_YAW       0       /* g_motor6020 / 电压数组里的 yaw 下标 */
#define M_PITCH     1       /* g_motor6020 / 电压数组里的 pitch 下标 */

void Gimbal_Init(void);
void Gimbal_Task(const Remote_t *rc);
void Gimbal_Stop(void);

/* 1=允许编码器锚定大地航向（只有锁大地档需要） */
void Gimbal_SetYawAnchor(uint8_t on);

float Gimbal_GetYawRel(void);       /* 云台相对底盘角，rad */
float Gimbal_GetYawRate(void);      /* 云台实测角速度，rad/s（陀螺低通值） */
float Gimbal_GetYawRateCmd(void);   /* 云台目标角速度，rad/s */

/* 限制云台自己的目标角速度（跟随档用来给底盘留追赶余量） */
void Gimbal_SetYawRateCap(float cap);

#endif
