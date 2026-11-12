/* 真实 BSP/应用/CDC 回调 + 模拟异步端点，覆盖队列生命周期而非串口设备。 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "usb_app.h"
#include "usbd_cdc_if.h"

USBD_HandleTypeDef hUsbDeviceHS;
static USBD_CDC_HandleTypeDef cdc;
static uint32_t irq_mask, critical_depth;
static int rx_armed, zlp_pending, fail_tx_once, fail_rx_once;
static uint8_t transmitted[32768], pattern[16384], scratch[2048];
static size_t transmitted_length;

/* 此套件验证 USB BSP；业务协议与 Flash 状态机由独立集成套件覆盖。 */
void FlashTasks_UsbInput(const uint8_t *data, uint16_t length) { (void)data; (void)length; }
void FlashTasks_UsbSession(uint32_t session, bool ready) { (void)session; (void)ready; }
void MotorStatusUsb_Process(void) {}

uint32_t __get_PRIMASK(void) { return irq_mask; }
void __disable_irq(void) { irq_mask = 1U; }
void __set_PRIMASK(uint32_t mask) { irq_mask = mask; }
void test_enter_critical(void) { ++critical_depth; }
void test_exit_critical(void) { assert(critical_depth); --critical_depth; }
int32_t osDelay(uint32_t ticks) { (void)ticks; return 0; }

uint8_t USBD_CDC_SetTxBuffer(USBD_HandleTypeDef *dev, uint8_t *data, uint32_t length)
{ (void)dev; cdc.TxBuffer = data; cdc.TxLength = length; return USBD_OK; }
uint8_t USBD_CDC_SetRxBuffer(USBD_HandleTypeDef *dev, uint8_t *data)
{ (void)dev; cdc.RxBuffer = data; return USBD_OK; }
uint8_t USBD_CDC_TransmitPacket(USBD_HandleTypeDef *dev)
{
    (void)dev;
    assert(irq_mask && !cdc.TxState);
    if (fail_tx_once) { fail_tx_once = 0; return USBD_FAIL; }
    cdc.TxState = 1U;
    return USBD_OK;
}
uint8_t USBD_CDC_ReceivePacket(USBD_HandleTypeDef *dev)
{
    (void)dev;
    assert(!rx_armed);
    if (fail_rx_once) { fail_rx_once = 0; return USBD_FAIL; }
    rx_armed = 1;
    return USBD_OK;
}

static UsbSerialStatus status(void)
{
    UsbSerialStatus value;
    UsbSerial_GetStatus(&value);
    return value;
}

static void connect_device(void)
{
    hUsbDeviceHS.dev_state = 0U;
    if (hUsbDeviceHS.pClassData != NULL) assert(USBD_Interface_fops_HS.DeInit() == USBD_OK);
    memset(&cdc, 0, sizeof(cdc));
    hUsbDeviceHS.pClassData = &cdc;
    hUsbDeviceHS.dev_speed = USBD_SPEED_FULL;
    rx_armed = zlp_pending = fail_tx_once = fail_rx_once = 0;
    transmitted_length = 0;
    assert(USBD_Interface_fops_HS.Init() == USBD_OK);
    assert(!UsbSerial_IsReady()); /* Init 回调早于配置完成。 */
    hUsbDeviceHS.dev_state = USBD_STATE_CONFIGURED;
    rx_armed = 1; /* 类驱动自己开放首次 OUT。 */
    assert(UsbSerial_IsReady());
}

static void receive(const uint8_t *data, uint32_t length)
{
    assert(rx_armed && length <= 64 && !irq_mask && !critical_depth);
    rx_armed = 0;
    memcpy(cdc.RxBuffer, data, length);
    assert(USBD_Interface_fops_HS.Receive(cdc.RxBuffer, &length) == USBD_OK);
}

static void finish_callback(void)
{
    cdc.TxState = 0;
    zlp_pending = 0;
    assert(USBD_Interface_fops_HS.TransmitCplt(cdc.TxBuffer, &cdc.TxLength, 1) == USBD_OK);
}

static void data_complete(void)
{
    assert(cdc.TxState && !zlp_pending);
    assert(transmitted_length + cdc.TxLength <= sizeof(transmitted));
    /* 完成时才读取发送缓冲区，能发现提前复用/覆盖异步缓冲区的问题。 */
    memcpy(transmitted + transmitted_length, cdc.TxBuffer, cdc.TxLength);
    transmitted_length += cdc.TxLength;
    if (cdc.TxLength != 0U && cdc.TxLength % 64U == 0U) zlp_pending = 1;
    else finish_callback();
}

static void complete(void)
{
    if (!zlp_pending) data_complete();
    if (zlp_pending) finish_callback();
}

static void drain(void)
{
    for (unsigned i = 0; i < 100 && status().tx_buffered; ++i)
    {
        UsbSerial_Process();
        if (cdc.TxState) complete();
    }
    assert(status().tx_buffered == 0);
}

static void test_rx(void)
{
    connect_device();
    for (unsigned i = 0; i < 1024; i += 64) receive(pattern + i, 64);
    assert(!rx_armed && UsbSerial_Available() == 1024 && status().rx_paused);
    assert(UsbSerial_Read(scratch, 13) == 13 && !rx_armed);
    assert(UsbSerial_Read(scratch + 13, 51) == 51 && rx_armed);
    receive(pattern + 1024, 64); /* 环形回绕，不能覆盖前面未读部分。 */
    assert(UsbSerial_Read(scratch + 64, 1024) == 1024);
    assert(memcmp(scratch, pattern, 1088) == 0 && rx_armed);
    receive(pattern, 0);
    assert(rx_armed && UsbSerial_Available() == 0);
    fail_rx_once = 1;
    receive(pattern, 7);
    assert(!rx_armed);
    UsbSerial_Process();
    assert(rx_armed && status().io_error_count > 0);
    assert(UsbSerial_Read(scratch, sizeof(scratch)) == 7);
    assert(memcmp(scratch, pattern, 7) == 0);
}

static void test_tx(void)
{
    connect_device();
    assert(UsbSerial_Send(NULL, 1) == USB_SERIAL_INVALID_ARGUMENT);
    assert(UsbSerial_Send(pattern, 0) == USB_SERIAL_INVALID_ARGUMENT);
    assert(UsbSerial_Send(pattern, 1025) == USB_SERIAL_TOO_LARGE);
    memcpy(scratch, pattern, 1024);
    assert(UsbSerial_Send(scratch, 1024) == USB_SERIAL_OK);
    memset(scratch, 0, 1024); /* 调用方可以立即复用原始数据。 */
    assert(UsbSerial_Send(pattern, 1) == USB_SERIAL_BUSY);
    UsbSerial_Process();
    assert(status().tx_in_flight && status().tx_buffered == 1024);
    data_complete();
    assert(zlp_pending && status().tx_buffered == 1024);
    assert(UsbSerial_Send(pattern, 1) == USB_SERIAL_BUSY); /* 等 ZLP 后才释放。 */
    complete();
    assert(status().tx_buffered == 0 && memcmp(transmitted, pattern, 1024) == 0);

    connect_device();
    assert(UsbSerial_Send(pattern, 900) == USB_SERIAL_OK);
    drain();
    assert(UsbSerial_Send(pattern + 900, 300) == USB_SERIAL_OK);
    UsbSerial_Process(); /* tail=900，只能先发送队列末尾 124 字节。 */
    assert(cdc.TxLength == 124);
    assert(UsbSerial_Send(pattern + 1200, 500) == USB_SERIAL_OK);
    complete();
    drain();
    assert(transmitted_length == 1700 && memcmp(transmitted, pattern, 1700) == 0);

    connect_device();
    assert(UsbSerial_Send(pattern, 63) == USB_SERIAL_OK);
    cdc.TxState = 1; /* 模拟端点暂忙，队列不得丢失。 */
    UsbSerial_Process();
    assert(!status().tx_in_flight && status().tx_buffered == 63);
    cdc.TxState = 0;
    fail_tx_once = 1;
    UsbSerial_Process();
    assert(!status().tx_in_flight && status().tx_buffered == 63);
    drain();
    assert(transmitted_length == 63 && memcmp(transmitted, pattern, 63) == 0);
}

static void test_connection_and_printf(void)
{
    connect_device();
    uint8_t coding[7], defaults[7] = {0x00, 0xC2, 0x01, 0, 0, 0, 8};
    assert(USBD_Interface_fops_HS.Control(CDC_GET_LINE_CODING, coding, 7) == USBD_OK);
    assert(memcmp(coding, defaults, 7) == 0);
    uint8_t baud9600[7] = {0x80, 0x25, 0, 0, 0, 0, 8};
    assert(USBD_Interface_fops_HS.Control(CDC_SET_LINE_CODING, baud9600, 7) == USBD_OK);
    assert(USBD_Interface_fops_HS.Control(CDC_GET_LINE_CODING, coding, 7) == USBD_OK);
    assert(memcmp(coding, baud9600, 7) == 0);
    assert(UsbSerial_Printf("value=%d\r\n", -42) == USB_SERIAL_OK);
    memset(scratch, 'X', 256); scratch[256] = 0;
    assert(UsbSerial_Printf("%s", (char *)scratch) == USB_SERIAL_TOO_LARGE);
    drain();
    assert(transmitted_length == 11 && memcmp(transmitted, "value=-42\r\n", 11) == 0);
    receive(pattern, 64);
    assert(UsbSerial_Send(pattern, 64) == USB_SERIAL_OK);
    hUsbDeviceHS.dev_state = USBD_STATE_SUSPENDED;
    assert(!UsbSerial_IsReady() && UsbSerial_Available() == 0);
    assert(UsbSerial_Send(pattern, 1) == USB_SERIAL_NOT_READY);
    assert(UsbSerial_Read(scratch, 64) == 0);
    UsbSerial_Process();
    assert(!cdc.TxState);
    hUsbDeviceHS.dev_state = USBD_STATE_CONFIGURED;
    drain();
    assert(UsbSerial_Available() == 64); /* 挂起并非撤销配置。 */
    assert(UsbSerial_Send(pattern, 64) == USB_SERIAL_OK);
    UsbSerial_Process(); /* 带着 RX 和在途 TX 重连。 */
    uint32_t session = status().session_id;
    connect_device();
    assert(status().session_id != session && UsbSerial_Available() == 0);
    UsbSerial_Process();
    assert(!cdc.TxState && status().tx_buffered == 0);
    assert(status().rx_discarded_bytes >= 64 && status().tx_discarded_bytes >= 64);
    irq_mask = 1;
    assert(UsbSerial_Send(pattern, 1) == USB_SERIAL_OK && irq_mask == 1);
    irq_mask = 0;
    drain();
}

static void test_application(void)
{
    connect_device();
    receive(pattern, 64);
    Campus_UsbProcess(); /* 空业务钩子消费接收数据，不自动发送。 */
    assert(UsbSerial_Available() == 0 && status().tx_buffered == 0 && !cdc.TxState);
    assert(transmitted_length == 0);

    assert(UsbSerial_Send(pattern, 64) == USB_SERIAL_OK);
    receive(pattern + 64, 64);
    Campus_UsbProcess(); /* 应用主动入队的发送仍由 USB 任务推进。 */
    assert(UsbSerial_Available() == 0 && status().tx_buffered == 64 && cdc.TxState);
    complete();
    Campus_UsbProcess();
    assert(status().tx_buffered == 0 && !cdc.TxState);
    assert(transmitted_length == 64 && memcmp(transmitted, pattern, 64) == 0);

    connect_device();
    Campus_UsbProcess();
    assert(!cdc.TxState && status().tx_buffered == 0 && transmitted_length == 0);
}

static void test_tx_tickets(void)
{
    connect_device();
    UsbSerialTxTicket a, b;
    assert(UsbSerial_SendTracked(pattern, 64, &a) == USB_SERIAL_OK);
    assert(UsbSerial_GetTxState(&a) == USB_TX_PENDING);
    UsbSerial_Process(); data_complete();
    assert(zlp_pending && UsbSerial_GetTxState(&a) == USB_TX_PENDING);
    complete(); assert(UsbSerial_GetTxState(&a) == USB_TX_COMPLETE);
    assert(UsbSerial_SendTracked(pattern, 64, &b) == USB_SERIAL_OK);
    connect_device();
    assert(UsbSerial_GetTxState(&b) == USB_TX_DISCONNECTED);
    assert(UsbSerial_SendTracked(pattern, 7, &a) == USB_SERIAL_OK);
    assert(UsbSerial_GetTxState(&a) == USB_TX_PENDING); drain();
    assert(UsbSerial_GetTxState(&a) == USB_TX_COMPLETE); /* 旧会话丢弃字节不影响新凭据。 */
    hUsbDeviceHS.dev_state = USBD_STATE_SUSPENDED;
    assert(UsbSerial_GetTxState(&a) == USB_TX_DISCONNECTED);
}

int main(void)
{
    for (size_t i = 0; i < sizeof(pattern); ++i) pattern[i] = (uint8_t)(i * 37U + i / 64U);
    assert(!UsbSerial_IsReady());
    assert(UsbSerial_Send(pattern, 1) == USB_SERIAL_NOT_READY);
    assert(CDC_Transmit_HS(pattern, 1) == USBD_FAIL);
    test_rx();
    test_tx();
    test_connection_and_printf();
    test_application();
    test_tx_tickets();
    assert(!irq_mask && !critical_depth);
    puts("PASS: USB queues, wrap, backpressure, TX ownership/ZLP, retries, session reset, printf, application RX/TX.");
    return 0;
}
