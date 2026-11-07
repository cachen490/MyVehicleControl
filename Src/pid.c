#include "pid.h"

void PID_Init(PID_t *p, float kp, float ki, float kd, float i_max, float out_max)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->i_max   = i_max;
    p->out_max = out_max;
    p->err_last = 0.0f;
    p->i_term   = 0.0f;
    p->out      = 0.0f;
}

void PID_Reset(PID_t *p)
{
    p->err_last = 0.0f;
    p->i_term   = 0.0f;
    p->out      = 0.0f;
}

/**
 * @brief 位置式 PID（带积分限幅）
 * @param target  目标值
 * @param measure 当前测量值
 * @param dt      周期，单位秒。本工程固定 0.001
 */
float PID_Calc(PID_t *p, float target, float measure, float dt)
{
    float err = target - measure;
    float d;

    p->i_term += p->ki * err * dt;
    if (p->i_term >  p->i_max) p->i_term =  p->i_max;
    if (p->i_term < -p->i_max) p->i_term = -p->i_max;

    d = (err - p->err_last) / dt;
    p->err_last = err;

    p->out = p->kp * err + p->i_term + p->kd * d;
    if (p->out >  p->out_max) p->out =  p->out_max;
    if (p->out < -p->out_max) p->out = -p->out_max;

    return p->out;
}
