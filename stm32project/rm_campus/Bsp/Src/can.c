/**
 * @file can.c
 * @brief FDCAN 经典帧收发实现。
 *
 * 本文件负责：
 *   1. 过滤器配置与外设启动。
 *   2. 标准帧发送。
 *
 * 接收由应用层的 HAL_FDCAN_RxFifo0Callback() 完成。
 */

#include "can.h"

#include <stddef.h>

/* 当前 FDCAN 句柄。一个 Can 驱动实例对应一个 FDCAN 外设。 */
static FDCAN_HandleTypeDef *s_can;

bool Can_Init(FDCAN_HandleTypeDef *hcan)
{
    /* 过滤器配置结构，先全部清零。 */
    FDCAN_FilterTypeDef filter = {0};

    if (hcan == NULL)
    {
        return false;
    }

    /* 驱动只保存一个全局句柄。再次对其他 CAN 调用 Init 会替换发送目标，
     * 当前工程只使用 FDCAN3，不支持多个独立 Can 实例并行发送。 */
    s_can = hcan;

    /*
     * 过滤器 0 使用标准 ID 范围模式，闭区间 0x201～0x204。
     * 对应 C620 ID 1～4，符合条件的反馈送入 RX FIFO0。
     * 这不是掩码模式，也不会接收总线上的全部标准 ID。
     */
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_RANGE;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0x201U;
    filter.FilterID2 = 0x204U;

    if (HAL_FDCAN_ConfigFilter(hcan, &filter) != HAL_OK)
    {
        return false;
    }

    /*
     * 全局策略拒绝未命中过滤器的标准帧、扩展帧和远程帧。
     * 只有上面范围过滤器接受的四个电机反馈进入 FIFO。
     */
    if (HAL_FDCAN_ConfigGlobalFilter(hcan,
                                     FDCAN_REJECT,
                                     FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE,
                                     FDCAN_REJECT_REMOTE) != HAL_OK)
    {
        return false;
    }

    /* 启动 FDCAN，进入正常收发状态。 */
    if (HAL_FDCAN_Start(hcan) != HAL_OK)
    {
        return false;
    }

    /*
     * 使能 RX FIFO0 新消息中断。
     * 应用层的 HAL_FDCAN_RxFifo0Callback() 由该中断触发。
     */
    if (HAL_FDCAN_ActivateNotification(hcan,
                                       FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                       0U) != HAL_OK)
    {
        return false;
    }

    return true;
}

bool Can_Send(uint32_t id, const uint8_t *data, uint8_t size)
{
    /* 发送头结构，先全部清零。 */
    FDCAN_TxHeaderTypeDef header = {0};

    if ((s_can == NULL) || (data == NULL) || (size > 8U) || (id > 0x7FFU))
    {
        return false;
    }

    /*
     * 经典 CAN 帧头配置：
     *   Identifier          = 11 位标准 ID
     *   IdType              = 标准 ID
     *   TxFrameType         = 数据帧
     *   DataLength          = 数据长度；经典帧的 DLC 与字节数一致
     *   ErrorStateIndicator = FDCAN_ESI_ACTIVE（主动状态）
     *   BitRateSwitch       = 关闭比特率切换
     *   FDFormat            = 经典 CAN
     *   TxEventFifoControl  = 关闭发送事件
     *   MessageMarker       = 0
     */
    header.Identifier = id;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    /* 当前 H7 HAL 中经典帧 DLC 0～8 与字节数相同；更换 HAL 版本时应核对
     * FDCAN_DLC_BYTES_x 的定义，不能不加判断地套用其他版本的位移写法。 */
    header.DataLength = size;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0U;

    /* 写入发送 FIFO；队列满或外设返回 HAL_ERROR 时，函数返回 false。 */
    /* HAL 会在本次调用内把数据复制到消息 RAM，因此调用方可使用局部数组。
     * 返回 true 只代表入队成功；总线 ACK、Bus-Off 和电机在线状态另行监测。 */
    return HAL_FDCAN_AddMessageToTxFifoQ(s_can,
                                         &header,
                                         (uint8_t *)data) == HAL_OK;
}
