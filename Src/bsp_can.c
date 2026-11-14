#include "bsp_can.h"
#include "can.h"

#define RX_QUEUE_SIZE 16

/* 接收帧环形队列：中断里写 head，任务里读 tail */
static CAN_RxFrame_t     s_queue[RX_QUEUE_SIZE];
static volatile uint8_t  s_head;
static volatile uint8_t  s_tail;

/* 开全通滤波 + 启动 CAN + 打开 FIFO0 接收中断 */
void BSP_CAN_Init(void)
{
    CAN_FilterTypeDef f = {0};

    f.FilterBank           = 0;
    f.FilterMode           = CAN_FILTERMODE_IDMASK;
    f.FilterScale          = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh         = 0x0000;
    f.FilterIdLow          = 0x0000;
    f.FilterMaskIdHigh     = 0x0000;
    f.FilterMaskIdLow      = 0x0000;
    f.FilterFIFOAssignment = CAN_RX_FIFO0;
    f.FilterActivation     = ENABLE;

    s_head = 0;
    s_tail = 0;

    HAL_CAN_ConfigFilter(&hcan1, &f);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/* 发一帧标准帧；邮箱满就返回 0 */
uint8_t BSP_CAN_SendStd(uint16_t std_id, const uint8_t *data, uint8_t len)
{
    CAN_TxHeaderTypeDef hdr = {0};
    uint32_t mailbox;

    hdr.StdId = std_id;
    hdr.ExtId = 0;
    hdr.IDE   = CAN_ID_STD;
    hdr.RTR   = CAN_RTR_DATA;
    hdr.DLC   = len;
    hdr.TransmitGlobalTime = DISABLE;
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
        return 0;
    }

    return (HAL_CAN_AddTxMessage(&hcan1, &hdr, (uint8_t *)data, &mailbox) == HAL_OK) ? 1 : 0;
}

/* 从队列取一帧，队列空返回 0 */
uint8_t BSP_CAN_PopFrame(CAN_RxFrame_t *out)
{
    if (s_tail == s_head) {
        return 0;
    }
    *out = s_queue[s_tail];
    s_tail = (uint8_t)((s_tail + 1) % RX_QUEUE_SIZE);
    return 1;
}

/* HAL 的接收中断回调：只收标准帧，满了就丢 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef hdr;
    uint8_t data[8];
    uint8_t next;

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hdr, data) != HAL_OK) {
        return;
    }
    if (hdr.IDE != CAN_ID_STD) {
        return;
    }

    next = (uint8_t)((s_head + 1) % RX_QUEUE_SIZE);
    if (next == s_tail) {
        return;
    }

    s_queue[s_head].id = hdr.StdId;
    for (int i = 0; i < 8; i++) {
        s_queue[s_head].data[i] = data[i];
    }
    s_head = next;
}
