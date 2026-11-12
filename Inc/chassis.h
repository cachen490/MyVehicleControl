#ifndef CHASSIS_H
#define CHASSIS_H

#include "remote.h"

void Chassis_Init(void);
void Chassis_Task(const Remote_t *rc);   /* 1ms 调一次：纯手动，拨轮自转 */
void Chassis_Stop(void);                 /* 停车 + 清积分 + 解跟随闩锁 */

/**
 * @brief 底盘跟随云台，1ms 调一次
 *
 * 目标：让"云台相对底盘角"恒定在车头（行程中点 90°）。
 * 云台由摇杆控制着相对大地转，底盘用轮子转过去贴合它 —— 也就是
 * **云台指向是基准，底盘在下面摆正**。
 *
 * ★ 和 Chassis_Task 二选一，别同时调（两套 wz 会互相覆盖）。
 * ★ 必须配合 Gimbal_SetYawFollow(0)：云台保持锁大地，跟随由底盘完成。
 *   两套跟随同时开会闭环同一个 rel，互相抢。
 * ★ 本模式下拨轮不参与自转；要原地转请推 yaw 摇杆。
 */
void Chassis_Follow_Task(const Remote_t *rc);

#endif /* CHASSIS_H */
