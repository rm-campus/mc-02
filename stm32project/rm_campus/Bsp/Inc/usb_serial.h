/**
 * @file usb_serial.h
 * @brief 应用层 USB CDC 字节流接口；所有公开函数仅从任务上下文调用。
 * Send/Printf 可由多个任务调用；Read 只允许一个消费者（默认 UsbTask）。
 * 不依赖换行或字符串结束符，不保留 USB 包边界，不执行协议组帧。
 */
#ifndef USB_SERIAL_H
#define USB_SERIAL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef USB_SERIAL_RX_CAPACITY
#define USB_SERIAL_RX_CAPACITY 1024U
#endif
#ifndef USB_SERIAL_TX_CAPACITY
#define USB_SERIAL_TX_CAPACITY 1024U
#endif
#ifndef USB_SERIAL_PRINTF_CAPACITY
#define USB_SERIAL_PRINTF_CAPACITY 256U
#endif

typedef enum
{
    USB_SERIAL_OK = 0,       /* 全部数据已复制入队，不代表电脑已收到。 */
    USB_SERIAL_BUSY,         /* 空间不足，未接受任何字节，调用方可稍后重试。 */
    USB_SERIAL_NOT_READY,    /* 未配置、已断开或挂起。 */
    USB_SERIAL_INVALID_ARGUMENT,
    USB_SERIAL_TOO_LARGE,    /* 超过 TX 容量或 Printf 格式化缓冲区，不截断。 */
    USB_SERIAL_FORMAT_ERROR
} UsbSerialResult;

typedef struct
{
    bool ready;             /* 已枚举且未挂起；不代表电脑已打开串口/正在读取。 */
    bool rx_paused;         /* OUT 尚未重新开放，主机通过 NAK 等待。 */
    bool tx_in_flight;
    uint16_t rx_buffered;
    uint16_t tx_buffered;    /* 包括尚未完成的 IN 传输。 */
    uint32_t session_id;     /* USB 类初始化/撤销时递增，协议层可据此清理半帧。 */
    uint32_t rx_bytes;
    uint32_t tx_queued_bytes;
    uint32_t tx_completed_bytes; /* USB 完成回调计数，不保证电脑程序已经消费。 */
    uint32_t rx_discarded_bytes;  /* 撤销配置时丢弃的旧缓存及异常输入。 */
    uint32_t tx_discarded_bytes;  /* 撤销配置时未确认完成的字节。 */
    uint32_t rx_pause_count;
    uint32_t tx_busy_count;
    uint32_t io_error_count;
} UsbSerialStatus;

bool UsbSerial_IsReady(void);

/* 非阻塞、整段入队。data 在返回后即可修改/释放；length==0 或 NULL 为参数错误。
 * 最大一次 USB_SERIAL_TX_CAPACITY 字节；不追加任何字符。 */
UsbSerialResult UsbSerial_Send(const void *data, uint16_t length);

/* 非阻塞读取，返回实际字节数；NULL/零容量/未就绪返回 0。
 * 可跨 USB 包合并或拆分返回；缓冲区由调用方分配。 */
uint16_t UsbSerial_Read(void *buffer, uint16_t capacity);
uint16_t UsbSerial_Available(void);

/* 不自动追加换行。最多 USB_SERIAL_PRINTF_CAPACITY-1 字节，否则整条拒绝。
 * 使用当前 newlib-nano 链接选项：默认未开启 %f 浮点格式化支持。 */
#if defined(__GNUC__)
__attribute__((format(printf, 1, 2)))
#endif
UsbSerialResult UsbSerial_Printf(const char *format, ...);

/* 短临界区内复制一致性快照；累计计数跨重新枚举保留，uint32_t 自然回绕。 */
void UsbSerial_GetStatus(UsbSerialStatus *status);

/* 服务入口：仅 UsbTask 调用，每轮提交待发数据、恢复有空间的 OUT 接收。
 * 不需要应用另行初始化：StartUsbTask 内的 MX_USB_DEVICE_Init 启动设备，
 * CDC 类回调管理本模块的连接状态。 */
void UsbSerial_Process(void);

#ifdef __cplusplus
}
#endif
#endif
