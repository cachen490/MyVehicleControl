#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "main.h"

typedef struct {
    uint16_t id;        /* 标准帧 ID */
    uint8_t  data[8];   /* 数据域 */
} CAN_RxFrame_t;

void BSP_CAN_Init(void);

/* 发一帧标准帧，成功返回 1 */
uint8_t BSP_CAN_SendStd(uint16_t std_id, const uint8_t *data, uint8_t len);

/* 从接收队列取一帧，队列空返回 0 */
uint8_t BSP_CAN_PopFrame(CAN_RxFrame_t *out);

#endif
