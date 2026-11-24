#ifndef ARM_USB_H
#define ARM_USB_H
#include "arm_config.h"
#include <stdbool.h>
#if ARM_VELOCITY_IK_ENABLE
/* 只在UsbTask中调用。故障重复报告受USB完成凭据和事务所有权约束。 */
void ArmUsb_Process(void);
bool ArmUsb_Busy(void);
#else
static inline void ArmUsb_Process(void) {}
static inline bool ArmUsb_Busy(void) { return false; }
#endif
#endif
