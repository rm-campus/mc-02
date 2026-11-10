/** @file usb_serial_port.h
 * @brief USB 类回调接入点，供 usbd_cdc_if.c 使用，应用层不要调用。
 */
#ifndef USB_SERIAL_PORT_H
#define USB_SERIAL_PORT_H
#include <stdint.h>

void UsbSerial_OnConfigured(uint16_t packet_size);
void UsbSerial_OnDeconfigured(void);
void UsbSerial_OnReceive(const uint8_t *data, uint32_t length);
void UsbSerial_OnTransmitComplete(const uint8_t *data, uint32_t length);
#endif
