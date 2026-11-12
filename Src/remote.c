#include "remote.h"
#include "usart.h"
#include <string.h>

#define DBUS_LEN     18
#define DMA_BUF_LEN  36

Remote_t g_rc;

static uint8_t s_dma[DMA_BUF_LEN];
static uint8_t s_frame[DBUS_LEN];

static volatile uint8_t s_rearm;    /* 出错后置 1，由 1ms 任务重新武装接收 */

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
        /* wheel 字段从字节 16 开始，4 字节窗会读到帧外 2 字节。
         * 越界读的是相邻静态变量，值最后会被掩码丢掉，不会出错，
         * 但没必要读 —— 出界就停。 */
        if ((uint32_t)(byte + i) >= DBUS_LEN) break;
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

/**
 * @brief UART 出错回调（HAL 里的 __weak 函数，这里覆盖它）
 *
 * ★ 这个函数不是可选的。HAL 在 DMA 接收模式下把 PE/FE/NE/ORE 一律当成
 *   "blocking error"：调 UART_EndRxTransfer() 关掉接收中断、清 DMAR、
 *   中止 DMA —— 然后【不重新打开】。原工程没有任何 ErrorCallback，
 *   所以上电后只要撞上一次帧错（接收机上电瞬间、插拔、极性边缘都算），
 *   遥控就永久收不到数据了，表现就是"车一动不动"，而且复位后可能又好了。
 *
 * 这里只置一个标志，真正的重新武装放到 1ms 任务里做（不在中断里调 HAL 的 DMA API）。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) return;

    huart->ErrorCode = HAL_UART_ERROR_NONE;

    s_rearm = 1;
}

void Remote_Update(void)
{
    static uint16_t last_cnt = 0;

    /* 上一次接收被错误中止了 → 重新武装。放在任务上下文，避开中断里调 DMA */
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
