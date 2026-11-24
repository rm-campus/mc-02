#ifndef SERVO_RC_H
#define SERVO_RC_H
#include "servo_config.h"
#include <stdbool.h>
#include <stdint.h>
typedef struct {
    uint32_t now_ms;
    bool config_valid, off_seen;
    uint8_t selected_mask, armed_mask, fault_mask;
    float target_p[5];
} ServoRcStatus;
#if SERVO_RC_ENABLE && SERVO_BUS_ENABLE
void ServoRc_Init(uint32_t now);
void ServoRc_Process(uint32_t now);
void ServoRc_CopyStatus(ServoRcStatus *out);
bool ServoRc_Selected(uint32_t now);
#else
static inline void ServoRc_Init(uint32_t now) { (void)now; }
static inline void ServoRc_Process(uint32_t now) { (void)now; }
static inline void ServoRc_CopyStatus(ServoRcStatus *out) { if(out)*out=(ServoRcStatus){0}; }
static inline bool ServoRc_Selected(uint32_t now) { (void)now;return false; }
#endif
#endif
