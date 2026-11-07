#include "remote.h"
#include "usart.h"
#include <string.h>

#define DBUS_LEN     18
#define DMA_BUF_LEN  36

Remote_t g_rc;

static uint8_t s_dma[DMA_BUF_LEN];
static uint8_t s_frame[DBUS_LEN];

void Remote_Init(void)
{
    memset(&g_rc, 0, sizeof(g_rc));
    g_rc.lost_ms = 0xFFFF;

    HAL_UART_Receive_DMA(&huart1, s_dma, DMA_BUF_LEN);

    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}

static uint16_t Get_Bits(const uint8_t *d, uint16_t bit_off, uint8_t bit_len)
{
    uint32_t byte = bit_off >> 3;
    uint32_t bit  = bit_off & 7;
    uint32_t acc  = 0;

    for (int i = 0; i < 4; i++) {
        acc |= (uint32_t)d[byte + i] << (8 * i);
    }
    acc >>= bit;

    return (uint16_t)(acc & ((1u << bit_len) - 1));
}

static void Remote_Decode(const uint8_t *d)
{
    g_rc.ch[0] = (int16_t)Get_Bits(d, 0,  11);
    g_rc.ch[1] = (int16_t)Get_Bits(d, 11, 11);
    g_rc.ch[2] = (int16_t)Get_Bits(d, 22, 11);
    g_rc.ch[3] = (int16_t)Get_Bits(d, 33, 11);

    g_rc.rx = (float)(g_rc.ch[0] - 1024) / 660.0f;
    g_rc.ry = (float)(g_rc.ch[1] - 1024) / 660.0f;
    g_rc.lx = (float)(g_rc.ch[2] - 1024) / 660.0f;
    g_rc.ly = (float)(g_rc.ch[3] - 1024) / 660.0f;

    if (g_rc.rx > -0.02f && g_rc.rx < 0.02f) g_rc.rx = 0.0f;
    if (g_rc.ry > -0.02f && g_rc.ry < 0.02f) g_rc.ry = 0.0f;
    if (g_rc.lx > -0.02f && g_rc.lx < 0.02f) g_rc.lx = 0.0f;
    if (g_rc.ly > -0.02f && g_rc.ly < 0.02f) g_rc.ly = 0.0f;

    g_rc.s[0] = (uint8_t)Get_Bits(d, 44, 2);
    g_rc.s[1] = (uint8_t)Get_Bits(d, 46, 2);

    g_rc.wheel = (float)((int16_t)Get_Bits(d, 128, 11) - 1024) / 660.0f;
    if (g_rc.wheel > -0.05f && g_rc.wheel < 0.05f) g_rc.wheel = 0.0f;

    g_rc.frame_cnt++;
    g_rc.lost_ms = 0;
}

/**
 * @brief 线路空闲（一帧收完）时被调用，在中断上下文执行
 *
 * DMA 在 Normal 模式下收满 36 字节才会自己停，而我们一帧只有 18 字节，
 * 所以每次空闲都要手动"停掉 → 重新武装"，否则下次就没 DMA 了。
 */
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

void Remote_Update(void)
{
    static uint16_t last_cnt = 0;

    if (g_rc.frame_cnt != last_cnt) {
        last_cnt = g_rc.frame_cnt;
        g_rc.lost_ms = 0;
    } else if (g_rc.lost_ms < 0xFFFF) {
        g_rc.lost_ms++;
    }
}
