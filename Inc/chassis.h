#ifndef CHASSIS_H
#define CHASSIS_H

#include "remote.h"

void Chassis_Init(void);
void Chassis_Task(const Remote_t *rc);   /* 1ms 调一次 */
void Chassis_Stop(void);                 /* 停车 + 清积分 */

#endif /* CHASSIS_H */
