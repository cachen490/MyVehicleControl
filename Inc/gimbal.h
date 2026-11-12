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

/**
 * @brief 云台相对底盘的角度，rad，0~2π（Yaw_RelAngle 坐标，带 SIGN_ENC_YAW）
 *
 * 给底盘跟随环当**反馈**用。正方向与 Gimbal_GetYawRate() 一致，
 * 所以两者可以直接在同一个式子里相加减（推导见 gimbal.c 里那个注释块）。
 *
 * ★ 它乘过 SIGN_ENC_YAW，符号不符合直觉是正常的；
 *   要判"云台在机构行程的哪个位置"，用 gimbal.c 内部的机械角（不乘任何符号）。
 */
float Gimbal_GetYawRel(void);

/**
 * @brief 云台相对大地的 yaw 角速度，rad/s（已乘 SIGN_GZ、已低通）
 *
 * 给底盘跟随环当**前馈**用。给 0 就退回纯 P，云台转快时底盘会明显滞后。
 */
float Gimbal_GetYawRate(void);

#endif /* GIMBAL_H */
