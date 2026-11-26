#include "usb_app.h"
#include "cmsis_os2.h"
#include "flash_tasks.h"
#include "motor_debug_usb.h"
#include "motor_status_usb.h"
#include "servo_usb.h"
#include "servo_rc_usb.h"
#include "arm_usb.h"
#include "robot_control_config.h"
#include "robot_servo_usb.h"

#define USB_APP_RX_CHUNK 256U

void Campus_UsbOnReceive(const uint8_t *data, uint16_t length)
{
    /* 公用按行接收入口；监测固件按配置分派状态查询与定向点动。 */
    FlashTasks_UsbInput(data, length);
}

void Campus_UsbProcess(void)
{
    UsbSerial_Process();
#if ROBOT_SERVO_TELEMETRY_ENABLE
    /* 整机回传只发送监测数据，不分派 USB 控制指令。 */
    uint8_t discarded[USB_APP_RX_CHUNK];
    (void)UsbSerial_Read(discarded, sizeof(discarded));
    RobotServoUsb_Process();
#else
    UsbSerialStatus usb;
    UsbSerial_GetStatus(&usb);
    FlashTasks_UsbSession(usb.session_id, usb.ready);
    uint8_t data[USB_APP_RX_CHUNK];
    uint16_t length = UsbSerial_Read(data, sizeof(data));
    if (length != 0U)
    {
        Campus_UsbOnReceive(data, length);
    }
#if !SERVO_STATUS_ONLY
    MotorStatusUsb_Process();
#endif
    ServoUsb_Process();
#if !SERVO_STATUS_ONLY
    ArmUsb_Process();
    ServoRcUsb_Process();
    MotorDebugUsb_Process();
#endif
#endif
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
