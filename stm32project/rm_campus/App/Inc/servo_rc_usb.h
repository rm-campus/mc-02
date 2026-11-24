#ifndef SERVO_RC_USB_H
#define SERVO_RC_USB_H
#include "servo_config.h"
#include <stdbool.h>
#if SERVO_RC_ENABLE && SERVO_BUS_ENABLE && SERVO_RC_TELEMETRY_ENABLE
void ServoRcUsb_Process(void);
bool ServoRcUsb_Selected(void);
bool ServoRcUsb_Busy(void);
#else
static inline void ServoRcUsb_Process(void) {}
static inline bool ServoRcUsb_Selected(void) { return false; }
static inline bool ServoRcUsb_Busy(void) { return false; }
#endif
#endif
