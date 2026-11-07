#ifndef REMOTE_H
#define REMOTE_H

#include "main.h"

typedef struct {
    int16_t ch[4];      /* 四个摇杆的原始值 364~1684 */
    float   rx, ry;     /* 右摇杆，-1 ~ 1 */
    float   lx, ly;     /* 左摇杆，-1 ~ 1 */
    float   wheel;      /* 左拨轮，-1 ~ 1 */
    uint8_t s[2];       /* s[0]=右拨杆，s[1]=左拨杆。数值 1/2/3，位置要现场确认 */
    uint16_t frame_cnt; /* 收到的帧计数，用来判断有没有新数据 */
    uint16_t lost_ms;   /* 距离上一帧的毫秒数，>50 就算掉线 */
} Remote_t;

extern Remote_t g_rc;

void Remote_Init(void);     /* 启动 DMA 接收 + 打开 IDLE 中断 */
void Remote_Update(void);   /* 1ms 调一次，维护 lost_ms */

/** @brief 供 USART1_IRQHandler 调用：线路空闲说明一帧收完（本工程 HAL 版本没有 UARTEx API） */
void Remote_OnIdle(void);

#endif /* REMOTE_H */
