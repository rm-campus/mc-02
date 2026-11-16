#ifndef SERVO_USB_H
#define SERVO_USB_H
#include "servo_config.h"
#include <stdbool.h>
#include <stdint.h>
#if SERVO_USB_DEBUG_ENABLE
bool ServoUsb_Dispatch(const uint8_t *line, uint16_t length);
bool ServoUsb_Busy(void);
void ServoUsb_Process(void);
#else
static inline bool ServoUsb_Dispatch(const uint8_t *p,uint16_t n) { (void)p;(void)n;return false; }
static inline bool ServoUsb_Busy(void) { return false; }
static inline void ServoUsb_Process(void) {}
#endif
#endif
