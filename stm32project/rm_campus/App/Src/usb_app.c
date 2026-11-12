#include "usb_app.h"
#include "cmsis_os2.h"
#include "flash_tasks.h"
#include "motor_debug_usb.h"
#include "motor_status_usb.h"

#define USB_APP_RX_CHUNK 256U

void Campus_UsbOnReceive(const uint8_t *data, uint16_t length)
{
    /* 单一接收入口：motor_status 始终有效，Flash 命令按功能开关启用。 */
    FlashTasks_UsbInput(data, length);
}

void Campus_UsbProcess(void)
{
    UsbSerial_Process();
    UsbSerialStatus usb;
    UsbSerial_GetStatus(&usb);
    FlashTasks_UsbSession(usb.session_id, usb.ready);
    uint8_t data[USB_APP_RX_CHUNK];
    uint16_t length = UsbSerial_Read(data, sizeof(data));
    if (length != 0U)
    {
        Campus_UsbOnReceive(data, length);
    }
    MotorStatusUsb_Process();
    MotorDebugUsb_Process();
    UsbSerial_Process();
}

void Campus_UsbTask(void *argument)
{
    (void)argument;
    /* MX_USB_DEVICE_Init 已由生成的 StartUsbTask 调用，不重复初始化。 */
    for (;;)
    {
        Campus_UsbProcess();
        osDelay(1U); /* 工程 tick 为 1 kHz；每轮处理量有界。 */
    }
}
