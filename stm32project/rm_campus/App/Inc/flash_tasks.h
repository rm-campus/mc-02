#ifndef FLASH_TASKS_H
#define FLASH_TASKS_H
#include <stdbool.h>
#include <stdint.h>
#include "flash_tasks_config.h"
typedef enum { FLASH_USB_NONE, FLASH_USB_TEST, FLASH_USB_LOG, FLASH_USB_STATUS } FlashUsbOwner;
void FlashTasks_Create(void);
void FlashTasks_UsbInput(const uint8_t *data, uint16_t length);
void FlashTasks_UsbSession(uint32_t session, bool ready);
bool FlashTasks_AcquireUsb(FlashUsbOwner owner);
void FlashTasks_ReleaseUsb(FlashUsbOwner owner);
void FlashTasks_Reply(const char *text);
#endif
