#ifndef CHASSIS_H
#define CHASSIS_H

#include "remote.h"

void Chassis_Init(void);
void Chassis_Task(const Remote_t *rc);          /* 手动档：摇杆 + 拨轮 */
void Chassis_Stop(void);

void Chassis_Follow_Task(const Remote_t *rc);   /* 跟随档：自转由跟随环算 */

/* 由四轮转速反算的底盘自转角速度，rad/s */
float Chassis_GetWzMeas(void);

#endif
