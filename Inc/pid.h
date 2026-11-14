#ifndef PID_H
#define PID_H

typedef struct {
    float kp, ki, kd;   /* 三个增益 */
    float i_max;        /* 积分项限幅 */
    float out_max;      /* 输出限幅 */
    float err_last;     /* 上一拍误差，算 D 项用 */
    float i_term;       /* 积分项累加值 */
    float out;          /* 上一拍输出 */

    unsigned char freeze;   /* 1=冻结积分（堵转保护） */
} PID_t;

void  PID_Init(PID_t *p, float kp, float ki, float kd, float i_max, float out_max);
float PID_Calc(PID_t *p, float target, float measure, float dt);
void  PID_Reset(PID_t *p);

#endif
