#include "remote.h"
#include "usart.h"
#include <string.h>

#define DBUS_LEN     18
#define DMA_BUF_LEN  36     /* 留两帧余量，防止 DMA 半帧对齐 */

Remote_t g_rc;

static uint8_t s_dma[DMA_BUF_LEN];
static uint8_t s_frame[DBUS_LEN];

static volatile uint8_t s_rearm;    /* 1=串口出错后需要重开 DMA */

void Remote_Init(void)
{
    memset(&g_rc, 0, sizeof(g_rc));
    g_rc.lost_ms = 0xFFFF;

    HAL_UART_Receive_DMA(&huart1, s_dma, DMA_BUF_LEN);

    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}

/* 从字节流里取任意位置、任意长度的位段（DBUS 是小端位序） */
static uint16_t Get_Bits(const uint8_t *d, uint16_t bit_off, uint8_t bit_len)
{
    uint32_t byte = bit_off >> 3;
    uint32_t bit  = bit_off & 7;
    uint32_t acc  = 0;

    for (int i = 0; i < 4; i++) {
        if ((uint32_t)(byte + i) >= DBUS_LEN) break;
        acc |= (uint32_t)d[byte + i] << (8 * i);
    }
    acc >>= bit;

    return (uint16_t)(acc & ((1u << bit_len) - 1));
}

/* 18 字节 DBUS 帧 → 摇杆归一化 + 右拨杆档位 */
static void Remote_Decode(const uint8_t *d)
{
    g_rc.ch[0] = (int16_t)Get_Bits(d, 0,  11);
    g_rc.ch[1] = (int16_t)Get_Bits(d, 11, 11);
    g_rc.ch[2] = (int16_t)Get_Bits(d, 22, 11);
    g_rc.ch[3] = (int16_t)Get_Bits(d, 33, 11);

    /* 中位 1024，满量程 ±660，归一化到 [-1, 1] */
    g_rc.rx = (float)(g_rc.ch[0] - 1024) / 660.0f;
    g_rc.lx = (float)(g_rc.ch[2] - 1024) / 660.0f;
    g_rc.ly = (float)(g_rc.ch[3] - 1024) / 660.0f;

    /* 摇杆回中死区 */
    if (g_rc.rx > -0.02f && g_rc.rx < 0.02f) g_rc.rx = 0.0f;
    if (g_rc.lx > -0.02f && g_rc.lx < 0.02f) g_rc.lx = 0.0f;
    if (g_rc.ly > -0.02f && g_rc.ly < 0.02f) g_rc.ly = 0.0f;

    g_rc.sw = (uint8_t)Get_Bits(d, 44, 2);

    g_rc.wheel = (float)((int16_t)Get_Bits(d, 128, 11) - 1024) / 660.0f;
    if (g_rc.wheel > -0.05f && g_rc.wheel < 0.05f) g_rc.wheel = 0.0f;

    g_rc.frame_cnt++;
    g_rc.lost_ms = 0;
}

/* 串口空闲中断：取 DMA 缓冲区最后 18 字节当一帧，然后重开 DMA */
void Remote_OnIdle(void)
{
    uint16_t remain;
    uint16_t size;
    if (huart1.hdmarx == NULL) {
        return;
    }
    remain = (uint16_t)__HAL_DMA_GET_COUNTER(huart1.hdmarx);
    size   = (uint16_t)(DMA_BUF_LEN - remain);

    if (size >= DBUS_LEN && size <= DMA_BUF_LEN) {
        memcpy(s_frame, &s_dma[size - DBUS_LEN], DBUS_LEN);
        Remote_Decode(s_frame);
    }

    HAL_UART_AbortReceive(&huart1);
    HAL_UART_Receive_DMA(&huart1, s_dma, DMA_BUF_LEN);
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
}

/* 串口错误（溢出/噪声）后只置标志，实际重启放到任务里做 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) return;

    huart->ErrorCode = HAL_UART_ERROR_NONE;

    s_rearm = 1;
}

/* 每 1ms：处理 DMA 重开 + 维护掉线计数 */
void Remote_Update(void)
{
    static uint16_t last_cnt = 0;

    if (s_rearm) {
        s_rearm = 0;
        if (huart1.RxState == HAL_UART_STATE_READY) {
            HAL_UART_Receive_DMA(&huart1, s_dma, DMA_BUF_LEN);
            __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
        }
    }

    if (g_rc.frame_cnt != last_cnt) {
        last_cnt = g_rc.frame_cnt;
        g_rc.lost_ms = 0;
    } else if (g_rc.lost_ms < 0xFFFF) {
        g_rc.lost_ms++;
    }
}
