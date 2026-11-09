#ifndef GIMBAL_H
#define GIMBAL_H

#include "remote.h"

#define M_YAW       0   /* g_motor6020 的下标 */
#define M_PITCH     1

void Gimbal_Init(void);
void Gimbal_Task(const Remote_t *rc);
void Gimbal_Stop(void);

/**
 * @brief 切换 yaw 的工作模式
 * @param on 0 = 锁大地（反馈陀螺，目标 0 就是"相对大地不转"）
 *           1 = 跟随底盘（反馈 6020 编码器，云台相对车体的角度被闭环）
 *
 * 只在模式真的发生变化的那一拍有副作用（同步目标角 + 清积分），
 * 所以可以每 1ms 无脑调用，不需要自己做边沿检测。
 * 6020 还没收到过反馈时，即使传 1 也不会进跟随。
 */
void Gimbal_SetYawFollow(uint8_t on);

/** @brief 当前是不是跟随模式。1 = 跟随，0 = 锁大地 */
uint8_t Gimbal_IsYawFollow(void);

/**
 * @brief 进跟随时，目标角从哪里起步
 * @param on 0 = 原地接管当前角度（默认，云台不跳变）
 *           1 = 取 YAW_ENC_HEAD，即自己转回车头
 *
 * 只在"下一次进跟随"时才生效；上电默认值由 robot_config.h 的
 * YAW_FOLLOW_REALIGN 决定。用 YAW_ENC_HEAD 之前必须先实测它。
 */
void Gimbal_SetYawRealign(uint8_t on);

#endif /* GIMBAL_H */
