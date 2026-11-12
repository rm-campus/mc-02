/** @file usb_app.h
 * @brief USB 应用入口：单电机回传、motor_status 诊断，以及可选 Flash 命令。
 */
#ifndef USB_APP_H
#define USB_APP_H
#include <stdint.h>
#include "usb_serial.h"

void Campus_UsbTask(void *argument);
/* 单轮有界处理，仅由 UsbTask 调用，也供主机回归测试驱动。 */
void Campus_UsbProcess(void);
/* 收到数据后调用此函数；在 usb_app.c 中统一分发业务协议。
 * data 只在本次调用有效；length 是一段字节流，不一定是一帧。
 * 运行于 UsbTask，可以调用 UsbSerial_Send/Printf，不宜长时间阻塞。 */
void Campus_UsbOnReceive(const uint8_t *data, uint16_t length);
#endif
