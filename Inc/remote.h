#ifndef REMOTE_H
#define REMOTE_H

#include "main.h"

typedef struct {
    int16_t  ch[4];     /* 四个通道原始值，364~1684（调试看这个） */
    float    rx;        /* 右摇杆左右，[-1, 1] */
    float    lx, ly;    /* 左摇杆左右 / 上下，[-1, 1] */
    float    wheel;     /* 左拨轮，[-1, 1]，锁大地档用来手转底盘 */
    uint8_t  sw;        /* 右拨杆档位，1/2/3 */
    uint16_t frame_cnt; /* 收到过的帧数，用来判掉线 */
    uint16_t lost_ms;   /* 距上次收到帧的毫秒数 */
} Remote_t;

extern Remote_t g_rc;

void Remote_Init(void);
void Remote_Update(void);

void Remote_OnIdle(void);

#endif
