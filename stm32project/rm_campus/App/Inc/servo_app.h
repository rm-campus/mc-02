#ifndef SERVO_APP_H
#define SERVO_APP_H
#include "servo_config.h"
#include "servo_uart.h"
#include "zp_servo.h"
typedef enum { SERVO_OK, SERVO_QUEUED, SERVO_BUSY, SERVO_INVALID, SERVO_RANGE,
    SERVO_OFFLINE, SERVO_BAD_MODE, SERVO_NOT_READY, SERVO_DISABLED,
    SERVO_TX_ERROR, SERVO_UART_ERROR_RESULT, SERVO_TIMEOUT } ServoResult;
typedef struct { uint8_t id; const char *model; uint16_t minimum, maximum; } ServoConfig;
typedef struct {
    uint8_t id, mode, failures;
    bool online, position_seen, mode_valid, id_seen, version_valid;
    uint16_t position, target;
    uint32_t position_ms, replies, timeouts, last_ticket;
    ServoResult last_result;
    char version[ZP_VERSION_MAX];
} ServoState;
typedef struct {
    ServoState device[SERVO_COUNT];
    ServoUartStatus uart;
    uint32_t now_ms, malformed, unexpected, coalesced, cancelled;
    uint16_t queued;
    bool initialized, enabled;
} ServoSnapshot;
const ServoConfig *Servo_GetConfig(unsigned index);
const char *Servo_ResultName(ServoResult result);
ServoResult Servo_MovePulse(uint8_t id, uint16_t position, uint16_t time_ms, uint32_t *ticket);
ServoResult Servo_RequestPosition(uint8_t id, uint32_t *ticket);
ServoResult Servo_Stop(uint8_t id, uint32_t *ticket);
ServoResult Servo_SetTorque(uint8_t id, bool enabled, uint32_t *ticket);
ServoResult Servo_Pause(uint8_t id, uint32_t *ticket);
ServoResult Servo_Resume(uint8_t id, uint32_t *ticket);
void Servo_CopySnapshot(ServoSnapshot *out);
bool Servo_CopyState(uint8_t id, ServoState *out);
/* 自动控制源只允许一个持有总线运动控制权；手动查询和停止不需要持有权。 */
typedef enum { SERVO_OWNER_NONE=0, SERVO_OWNER_RC=1, SERVO_OWNER_ARM=2 } ServoControlOwner;
/* 掩码使用 SERVO_DEVICE_TABLE 的槽位，不能直接把总线 ID 当作位号。
 * Claim/Release 返回 false 时，调用方必须等待原运动及停止事务排空，不能强行抢占。 */
bool Servo_ControlClaim(ServoControlOwner owner, uint8_t slots);
bool Servo_ControlRelease(ServoControlOwner owner);
void Servo_ControlStopMask(ServoControlOwner owner, uint8_t slots);
uint8_t Servo_ControlStopPending(ServoControlOwner owner);
uint8_t Servo_ControlInhibited(ServoControlOwner owner);
void Servo_ControlClearInhibit(ServoControlOwner owner);
/* 整组先校验再提交；失败不写入任何一轴，重复 ID 视为无效输入。 */
ServoResult Servo_ControlMoveGroup(ServoControlOwner owner, const uint8_t *ids,
    const uint16_t *positions, const uint16_t *times_ms, unsigned count);
/* RC owner task only; masks use configured table slots, not protocol IDs.
 * Stop requests have reserved storage and cannot be lost to a full normal queue. */
void Servo_RcClaim(uint8_t slots);
void Servo_RcStopMask(uint8_t slots);
uint8_t Servo_RcStopPending(void);
uint8_t Servo_RcInhibited(void);
void Servo_RcClearInhibit(void);
ServoResult Servo_RcMovePulse(uint8_t id, uint16_t position, uint16_t time_ms, uint32_t *ticket);
/* Only ServoBusTask calls Init/Process. Exposed for deterministic host testing. */
bool Servo_Init(void);
void Servo_Process(uint32_t now);
void Campus_ServoTask(void *argument);
#endif
