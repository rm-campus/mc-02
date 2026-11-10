#include "usb_app.h"
#include "cmsis_os2.h"

#define USB_APP_RX_CHUNK 256U

void Campus_UsbOnReceive(const uint8_t *data, uint16_t length)
{
    /* 正式协议入口：在这里消费电脑发来的字节，默认不自动回复。
     * 示例：UsbSerial_Send(reply, reply_length);
     * 示例：UsbSerial_Printf("speed=%d\r\n", speed);
     * 检查返回值，BUSY 时由业务保留数据稍后重试；不要在循环中忙等。
     * 若做跨调用组帧，使用 UsbSerial_GetStatus 的 session_id 清理旧半帧。 */
    (void)data;
    (void)length;
}

void Campus_UsbProcess(void)
{
    UsbSerial_Process();
    uint8_t data[USB_APP_RX_CHUNK];
    uint16_t length = UsbSerial_Read(data, sizeof(data));
    if (length != 0U)
    {
        Campus_UsbOnReceive(data, length);
    }
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
