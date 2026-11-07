#ifndef PID_H
#define PID_H

typedef struct {
    float kp, ki, kd;
    float i_max;     /* 积分项限幅，防止积分饱和 */
    float out_max;   /* 输出限幅 */
    float err_last;  /* 上一次误差，算微分用 */
    float i_term;    /* 积分累加值 */
    float out;       /* 上一次输出，方便观察 */
} PID_t;

void  PID_Init(PID_t *p, float kp, float ki, float kd, float i_max, float out_max);
float PID_Calc(PID_t *p, float target, float measure, float dt);
void  PID_Reset(PID_t *p);

#endif /* PID_H */
