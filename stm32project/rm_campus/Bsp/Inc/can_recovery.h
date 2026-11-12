#ifndef CAN_RECOVERY_H
#define CAN_RECOVERY_H
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    CAN_RECOVERY_READY = 0,
    CAN_RECOVERY_REQUESTED,
    CAN_RECOVERY_WAIT_BUS,
    CAN_RECOVERY_WAIT_FEEDBACK,
    CAN_RECOVERY_FAILED
} CanRecoveryState;
typedef struct {
    CanRecoveryState state;
    uint32_t bus_off_count, attempts, success, failures;
    uint32_t last_duration_ms;
    uint32_t error; /* 0 无，1 Stop 失败，2 队列未清空，3 Start 失败。 */
    uint8_t feedback_mask; /* 本次恢复后已收到新反馈的轮；初始无恢复事务时为 0x0F。 */
} CanRecoveryStatus;

/* ChassisTask 唯一调用者；输入来自同一轮 PSR 采样，不在恢复模块重复读取 PSR。
 * true 才可发送；活动电机中任意一台新反馈确认总线恢复，逐轮输出仍检查下面的掩码。 */
bool Can_ProcessRecovery(bool status_valid, bool bus_off, uint32_t now_ms,
                         uint8_t required_feedback_mask);
/* CAN RX ISR 在成功解析 0x201~0x204 后调用。 */
void Can_RecoveryFeedback(uint8_t slot);
uint8_t Can_RecoveryOutputMask(void);
/* 仅任务上下文调用；不清零统计。 */
void Can_CopyRecovery(CanRecoveryStatus *out);
#endif
