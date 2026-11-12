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
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

/* 当前 FDCAN 句柄。一个 Can 驱动实例对应一个 FDCAN 外设。 */
static FDCAN_HandleTypeDef *s_can;
static CanRecoveryStatus s_recovery;
static volatile bool s_irq_pending, s_collect_feedback;
static volatile uint8_t s_feedback_mask;
static bool s_initialized, s_tx_allowed, s_attempted;
static uint32_t s_fault_ms, s_attempt_ms;
#define CAN_RECOVERY_RETRY_MS 100U

/* Source: USTC-ROBOWALKER damiao_mc02_bsp, commit 30ff2965,
 * dm02_test/User_File/1_Middleware/Driver/CAN/drv_can.cpp.
 * https://github.com/yssickjgd/damiao_mc02_bsp/tree/30ff2965d55afbcad500027b0e1e5031ea6f93b8
 * 保留 Stop/Start 恢复方式；ISR 只请求，由发送任务处理并核对返回值。 */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hcan, uint32_t flags)
{
    if (hcan == s_can && (flags & FDCAN_IT_BUS_OFF) != 0U) s_irq_pending = true;
}

void Can_RecoveryFeedback(uint8_t slot)
{
    if (s_collect_feedback && slot < 4U) s_feedback_mask |= (uint8_t)(1U << slot);
}

void Can_CopyRecovery(CanRecoveryStatus *out)
{
    if (out == NULL) return;
    taskENTER_CRITICAL();
    *out = s_recovery;
    out->feedback_mask = s_feedback_mask;
    taskEXIT_CRITICAL();
}

uint8_t Can_RecoveryOutputMask(void)
{
    taskENTER_CRITICAL();
    uint8_t mask = s_tx_allowed && !s_irq_pending && s_can != NULL &&
        (s_can->Instance->CCCR & FDCAN_CCCR_INIT) == 0U ? s_feedback_mask : 0U;
    taskEXIT_CRITICAL();
    return mask;
}

bool Can_ProcessRecovery(bool valid, bool bus_off, uint32_t now, uint8_t required)
{
    s_tx_allowed = false;
    if (!s_initialized) return false;
    /* 通知也会在 BO 清除时到达；以统一采集的硬件状态判断，不因退出事件再重启。 */
    taskENTER_CRITICAL();
    s_irq_pending = false;
    bool init = (s_can->Instance->CCCR & FDCAN_CCCR_INIT) != 0U;
    if (valid && bus_off &&
        (s_recovery.state == CAN_RECOVERY_READY || s_recovery.state == CAN_RECOVERY_WAIT_FEEDBACK ||
         (s_recovery.state == CAN_RECOVERY_WAIT_BUS && init))) {
        s_recovery.state = CAN_RECOVERY_REQUESTED;
        ++s_recovery.bus_off_count;
        s_fault_ms = now;
        s_collect_feedback = false;
        s_feedback_mask = 0U;
    }
    CanRecoveryState state = s_recovery.state;
    taskEXIT_CRITICAL();
    if (!valid || state == CAN_RECOVERY_FAILED) return false;

    if (state == CAN_RECOVERY_REQUESTED) {
        /* 第一次立即恢复；连续故障限制重试，WAIT_BUS/INIT=0 时不重置硬件恢复序列。 */
        if (s_attempted && (uint32_t)(now - s_attempt_ms) < CAN_RECOVERY_RETRY_MS) return false;
        s_attempt_ms = now; s_attempted = true;
        taskENTER_CRITICAL();
        ++s_recovery.attempts;
        taskEXIT_CRITICAL();

        /* Stop 中置 CCE 会重置 RX FIFO 和 TX 请求/索引；不复位整个 MCU，不改过滤器/时钟。
         * 屏蔽 CAN IRQ 的短临界区避免 ISR 在 Stop/Start 之间读取 FIFO。
         * 本项目 HAL Stop 使用有界计数等待（FDCAN_TIMEOUT_COUNT=50），不依赖 tick 延时。 */
        taskENTER_CRITICAL();
        uint32_t error = 0U;
        if (HAL_FDCAN_Stop(s_can) != HAL_OK) error = 1U;
        else if (s_can->Instance->TXBRP != 0U || HAL_FDCAN_GetRxFifoFillLevel(s_can, FDCAN_RX_FIFO0) != 0U)
            error = 2U;
        else {
            s_feedback_mask = 0U;
            s_collect_feedback = true;
            if (HAL_FDCAN_Start(s_can) != HAL_OK) error = 3U;
        }
        s_recovery.error = error;
        if (error != 0U) {
            ++s_recovery.failures;
            s_recovery.state = CAN_RECOVERY_FAILED;
            s_collect_feedback = false;
        } else s_recovery.state = CAN_RECOVERY_WAIT_BUS;
        taskEXIT_CRITICAL();
        return false; /* Start 成功不等于恢复完成，下一轮重新检查 PSR。 */
    }

    taskENTER_CRITICAL();
    if (!bus_off && !init && s_recovery.state == CAN_RECOVERY_WAIT_BUS)
        s_recovery.state = CAN_RECOVERY_WAIT_FEEDBACK;
    if (!bus_off && !init && s_recovery.state == CAN_RECOVERY_WAIT_FEEDBACK &&
        (s_feedback_mask & required) != 0U) {
        ++s_recovery.success;
        s_recovery.last_duration_ms = now - s_fault_ms;
        s_recovery.state = CAN_RECOVERY_READY;
        /* 继续收集迟到的其他轮反馈；单轮缺失不阻塞已恢复的轮。 */
    }
    s_tx_allowed = !bus_off && !init && s_recovery.state == CAN_RECOVERY_READY && !s_irq_pending;
    bool allowed = s_tx_allowed;
    taskEXIT_CRITICAL();
    return allowed;
}

bool Can_Init(FDCAN_HandleTypeDef *hcan)
{
    s_initialized = s_tx_allowed = s_attempted = s_irq_pending = s_collect_feedback = false;
    s_feedback_mask = 0x0FU;
    memset(&s_recovery, 0, sizeof(s_recovery));
    /* 过滤器配置结构，先全部清零。 */
    FDCAN_FilterTypeDef filter = {0};

    if (hcan == NULL)
    {
        return false;
    }

    /* 驱动只保存一个全局句柄。再次对其他 CAN 调用 Init 会替换发送目标，
     * 当前工程只使用 FDCAN1，不支持多个独立 Can 实例并行发送。 */
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
                                       FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_MESSAGE_LOST | FDCAN_IT_BUS_OFF,
                                       0U) != HAL_OK)
    {
        return false;
    }

    s_initialized = true;
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
    taskENTER_CRITICAL();
    bool sent = s_initialized && s_tx_allowed && !s_irq_pending &&
        (s_can->Instance->CCCR & FDCAN_CCCR_INIT) == 0U &&
        HAL_FDCAN_AddMessageToTxFifoQ(s_can, &header, (uint8_t *)data) == HAL_OK;
    taskEXIT_CRITICAL();
    return sent;
}
