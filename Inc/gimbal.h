#ifndef GIMBAL_H
#define GIMBAL_H

#include "remote.h"

#define M_YAW       0   /* g_motor6020 的下标 */
#define M_PITCH     1

void Gimbal_Init(void);
void Gimbal_Task(const Remote_t *rc);
void Gimbal_Stop(void);

#endif /* GIMBAL_H */
