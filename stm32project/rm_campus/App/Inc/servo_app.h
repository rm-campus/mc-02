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
/* Only ServoBusTask calls Init/Process. Exposed for deterministic host testing. */
bool Servo_Init(void);
void Servo_Process(uint32_t now);
void Campus_ServoTask(void *argument);
#endif
