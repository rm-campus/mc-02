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
typedef struct {
    uint32_t ticket;
    uint8_t requested_mode; /* 0 for a single read-only probe, 1..4 for a position-mode write. */
    uint8_t mode[SERVO_COUNT]; /* 0 means this batch did not receive a mode. */
    ServoResult result[SERVO_COUNT];
    bool probe;
    uint8_t probe_slot;
    uint32_t probe_first_ms; /* From UART TX completion to consuming the first RX byte. */
    uint16_t probe_rx_bytes;
    uint8_t probe_raw[ZP_FRAME_MAX];
} ServoModeReport;
const ServoConfig *Servo_GetConfig(unsigned index);
const char *Servo_ResultName(ServoResult result);
#if SERVO_JOG_TEST_ENABLE
typedef struct {
    uint32_t ticket;
    uint8_t id;
    uint16_t start, target, actual; /* P 值；0 表示尚未取得。 */
    const char *result;
} ServoJogReport;
ServoResult ServoJog_Start(uint8_t id, uint8_t direction, uint32_t ticket, uint32_t session);
bool ServoJog_TakeResult(uint32_t ticket, ServoJogReport *out);
void ServoJog_Cancel(void);
#endif
ServoResult Servo_MovePulse(uint8_t id, uint16_t position, uint16_t time_ms, uint32_t *ticket);
ServoResult Servo_RequestPosition(uint8_t id, uint32_t *ticket);
ServoResult Servo_RequestModes(uint8_t position_mode, uint32_t *ticket);
ServoResult Servo_RequestModeProbe(uint8_t id, uint32_t *ticket);
bool Servo_CopyModeReport(uint32_t ticket, ServoModeReport *out);
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
#if ROBOT_CONTROL_ENABLE
/* 撤销还在等待发送的夹爪目标，保持已经交给舵机的位置与扭矩。 */
void Servo_RobotCancelQueuedGrip(void);
#endif
ServoResult Servo_RcMovePulse(uint8_t id, uint16_t position, uint16_t time_ms, uint32_t *ticket);
/* Only ServoBusTask calls Init/Process. Exposed for deterministic host testing. */
bool Servo_Init(void);
void Servo_Process(uint32_t now);
void Campus_ServoTask(void *argument);
#endif
