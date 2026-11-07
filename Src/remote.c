#include "remote.h"
#include "usart.h"
#include <string.h>

#define DBUS_LEN     18
#define DMA_BUF_LEN  36     /* 两帧的余量：万一第一帧是半截，取后 18 字节仍是完整的 */

Remote_t g_rc;

static uint8_t s_dma[DMA_BUF_LEN];
static uint8_t s_frame[DBUS_LEN];

/* ------------------------------------------------------------------
 * 为什么不用 HAL_UARTEx_ReceiveToIdle_DMA？
 *
 * 那是 HAL 1.7.5 以后才有的 API，本工程 CubeMX 带的固件包是 HAL 1.7.4，
 * 里面根本没有这个函数（编译会报 implicit declaration）。
 *
 * 所以这里是"手工版"的等价实现，只多三行：
 *   ① HAL_UART_Receive_DMA()          启动 DMA 收数据（不关心收多少）
 *   ② __HAL_UART_ENABLE_IT(UART_IT_IDLE)  打开"线路空闲"中断
 *   ③ 在 stm32f4xx_it.c 的 USART1_IRQHandler 里判 IDLE 标志 → 调 Remote_OnIdle()
 *
 * 效果完全一样：DBUS 每帧 18 字节，帧间空闲 12ms，空闲一到就说明这帧收完了。
 * ------------------------------------------------------------------ */

void Remote_Init(void)
{
    memset(&g_rc, 0, sizeof(g_rc));
    g_rc.lost_ms = 0xFFFF;

    HAL_UART_Receive_DMA(&huart1, s_dma, DMA_BUF_LEN);

    /* 先清一次标志，再开中断，避免刚开就进一次假的空闲中断 */
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}

/** @brief 从任意起始位取一段二进制，最多 16 位 */
static uint16_t Get_Bits(const uint8_t *d, uint16_t bit_off, uint8_t bit_len)
{
    uint32_t byte = bit_off >> 3;
    uint32_t bit  = bit_off & 7;
    uint32_t acc  = 0;

    /* 一次读 4 个字节拼成 32 位数，再整体右移，就不用一位一位拼了 */
    for (int i = 0; i < 4; i++) {
        acc |= (uint32_t)d[byte + i] << (8 * i);
    }
    acc >>= bit;

    return (uint16_t)(acc & ((1u << bit_len) - 1));
}

static void Remote_Decode(const uint8_t *d)
{
    /* 1. 摇杆原始值 */
    g_rc.ch[0] = (int16_t)Get_Bits(d, 0,  11);
    g_rc.ch[1] = (int16_t)Get_Bits(d, 11, 11);
    g_rc.ch[2] = (int16_t)Get_Bits(d, 22, 11);
    g_rc.ch[3] = (int16_t)Get_Bits(d, 33, 11);

    /* 2. 归一化。364 → -1.0，1024 → 0，1684 → +1.0 */
    g_rc.rx = (float)(g_rc.ch[0] - 1024) / 660.0f;
    g_rc.ry = (float)(g_rc.ch[1] - 1024) / 660.0f;
    g_rc.lx = (float)(g_rc.ch[2] - 1024) / 660.0f;
    g_rc.ly = (float)(g_rc.ch[3] - 1024) / 660.0f;

    /* 3. 死区：摇杆回中时不会真的等于 1024，一般有 ±5 的偏移。
     *    不加死区的话，松手后车会慢慢爬。 */
    if (g_rc.rx > -0.02f && g_rc.rx < 0.02f) g_rc.rx = 0.0f;
    if (g_rc.ry > -0.02f && g_rc.ry < 0.02f) g_rc.ry = 0.0f;
    if (g_rc.lx > -0.02f && g_rc.lx < 0.02f) g_rc.lx = 0.0f;
    if (g_rc.ly > -0.02f && g_rc.ly < 0.02f) g_rc.ly = 0.0f;

    /* 4. 拨杆：11 位摇杆之后紧跟着 2+2 位 */
    g_rc.s[0] = (uint8_t)Get_Bits(d, 44, 2);
    g_rc.s[1] = (uint8_t)Get_Bits(d, 46, 2);

    /* 5. 左拨轮：在最后两个字节的低 11 位 */
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

    remain = (uint16_t)__HAL_DMA_GET_COUNTER(huart1.hdmarx);  /* 还剩几个位置没写 */
    size   = (uint16_t)(DMA_BUF_LEN - remain);                /* 这一轮实际收了多少 */

    /* 只取最后 18 字节：上电时可能正好从半帧开始，尾巴一定是完整的一帧 */
    if (size >= DBUS_LEN && size <= DMA_BUF_LEN) {
        memcpy(s_frame, &s_dma[size - DBUS_LEN], DBUS_LEN);
        Remote_Decode(s_frame);
    }

    /* 重新武装。注意 HAL_UART_AbortReceive 会顺手关掉 IDLE 中断，得再打开 */
    HAL_UART_AbortReceive(&huart1);
    HAL_UART_Receive_DMA(&huart1, s_dma, DMA_BUF_LEN);
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}

void Remote_Update(void)
{
    static uint16_t last_cnt = 0;

    if (g_rc.frame_cnt != last_cnt) {
        last_cnt = g_rc.frame_cnt;
        g_rc.lost_ms = 0;               /* 有新帧 */
    } else if (g_rc.lost_ms < 0xFFFF) {
        g_rc.lost_ms++;                 /* 没有新帧，计时往上走 */
    }
}
