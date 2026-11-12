#ifndef MOTOR_STATUS_USB_H
#define MOTOR_STATUS_USB_H
#include <stdbool.h>
/* 以下接口只在 UsbTask 调用。请求合并为一份快照，断线丢弃。 */
void MotorStatusUsb_Request(void);
void MotorStatusUsb_Process(void);
bool MotorStatusUsb_Busy(void);
#endif
