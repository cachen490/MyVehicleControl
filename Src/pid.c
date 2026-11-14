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
    p->freeze   = 0;
}

void PID_Reset(PID_t *p)
{
    p->err_last = 0.0f;
    p->i_term   = 0.0f;
    p->out      = 0.0f;
    p->freeze   = 0;
}

/* 位置式 PID。freeze=1 时冻结积分（堵转保护用） */
float PID_Calc(PID_t *p, float target, float measure, float dt)
{
    float err = target - measure;
    float d;

    if (!p->freeze)
    {
        p->i_term += p->ki * err * dt;
        if (p->i_term >  p->i_max) p->i_term =  p->i_max;
        if (p->i_term < -p->i_max) p->i_term = -p->i_max;
    }

    /* D 项对误差差分（测量值没做滤波，靠调用方的低通压噪） */
    d = (err - p->err_last) / dt;
    p->err_last = err;

    p->out = p->kp * err + p->i_term + p->kd * d;
    if (p->out >  p->out_max) p->out =  p->out_max;
    if (p->out < -p->out_max) p->out = -p->out_max;

    return p->out;
}
