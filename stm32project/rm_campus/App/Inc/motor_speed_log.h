#ifndef MOTOR_SPEED_LOG_H
#define MOTOR_SPEED_LOG_H
#include <stdbool.h>
#include <stdint.h>
#include "fxi6x.h"
typedef enum {
    MOTOR_LOG_BOOT, MOTOR_LOG_PREPARING, MOTOR_LOG_READY, MOTOR_LOG_RECORDING,
    MOTOR_LOG_FLUSHING, MOTOR_LOG_WAIT_READ, MOTOR_LOG_REPLAYING,
    MOTOR_LOG_ERASING, MOTOR_LOG_ERROR
} MotorLogState;
typedef struct {
    MotorLogState state;
    uint32_t records, error, stop_reason;
    bool reached, target_valid;
} MotorLogStatus;
bool MotorLog_Create(void);
void MotorLog_OnRcFrame(const Fxi6x_Data *rc);
void MotorLog_Process(uint32_t now);
bool MotorLog_RequestRead(void);
bool MotorLog_RequestClear(void);
void MotorLog_GetStatus(MotorLogStatus *out);
#endif
