#ifndef IMU_H
#define IMU_H

#include "main.h"

void    IMU_Init(void);
void    IMU_CalibrateGyro(void); /* 上电静止时调一次（500ms），扣掉零偏 */
void    IMU_Update(void);       /* 1ms 调一次 */
float   IMU_GetGyroX(void);     /* rad/s，已扣零偏 */
float   IMU_GetGyroY(void);
float   IMU_GetGyroZ(void);
uint8_t IMU_IsOnline(void);
float   IMU_GetPitch(void);     /* rad，Mahony 解出来的俯仰角，正 = 抬头，0 = 水平 */
float   IMU_GetRoll(void);      /* rad，绕 A 板 X 轴的倾角。
                                 * ★ 本机云台 pitch 关节的转轴对应的是这一个，
                                 * 所以 pitch 自稳环的反馈取它，不是 IMU_GetPitch() */

#endif /* IMU_H */
