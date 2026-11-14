#ifndef IMU_H
#define IMU_H

#include "main.h"

void    IMU_Init(void);
void    IMU_CalibrateGyro(void);    /* 静止 500ms 标零偏 + 定初始姿态 */

void    IMU_Update(void);           /* 每 1ms 调一次 */

float   IMU_GetGyroX(void);         /* 陀螺 X/Z 轴，rad/s，已减零偏 */
float   IMU_GetGyroZ(void);

uint8_t IMU_IsOnline(void);

/* 欧拉角，rad。本机 pitch 关节对应的是 Roll 轴 */
float   IMU_GetRoll(void);
float   IMU_GetYaw(void);

#endif
