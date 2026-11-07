//
// Created by lenovo on 2026/11/7.
//

#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "main.h"

/* 一帧接收到的报文。Step 2 才会用到，但接口先定义好，免得以后改头文件 */
typedef struct {
    uint16_t id;        /* 标准帧 ID */
    uint8_t  data[8];   /* 数据，不足 8 字节的部分是无效值 */
} CAN_RxFrame_t;

/**
 * @brief 初始化 CAN1：配滤波器 + 启动 + 打开接收中断
 *
 * 为什么还没收数据也要配滤波器？
 * 因为 STM32 的 CAN 外设如果没配滤波器，所有报文都会被丢弃。
 * 后面 Step 2 要收数据时会得到"什么都收不到"的假故障，很难查。
 */
void BSP_CAN_Init(void);

/**
 * @brief 发一帧标准数据帧
 * @param std_id 标准 ID（电机是 0x200 / 0x1FF / 0x2FF）
 * @param data   数据缓冲
 * @param len    长度，电机固定 8
 * @return 1 成功，0 失败（邮箱满或出错）
 */
uint8_t BSP_CAN_SendStd(uint16_t std_id, const uint8_t *data, uint8_t len);

/**
 * @brief 从接收队列里取一帧出来（Step 2 实现，这里先把声明放好）
 * @return 1 取到了，0 队列是空的
 */
uint8_t BSP_CAN_PopFrame(CAN_RxFrame_t *out);

#endif /* BSP_CAN_H */
