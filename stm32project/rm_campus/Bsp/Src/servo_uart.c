#include "servo_uart.h"
#include "servo_config.h"
#include "zp_servo.h"
#include "usart.h"
#include "cmsis_os2.h"
#include <string.h>

static uint8_t s_rx_byte, s_ring[SERVO_RX_RING_SIZE];
static uint8_t s_tx[ZP_FRAME_MAX];
static volatile uint16_t s_head, s_tail;
static volatile uint32_t s_events;
static volatile ServoUartStatus s_status;
static osThreadId_t s_task;
static uint32_t lock(void) { uint32_t p=__get_PRIMASK(); __disable_irq(); return p; }
static void notify(void) { if(s_task) (void)osThreadFlagsSet(s_task,SERVO_TASK_FLAG); }
static void error(void) { ++s_status.errors; s_status.ready=false; s_events|=SERVO_UART_ERROR; notify(); }

/* These UART7 calls only update registers/state: no DMA and no polling wait. */
static bool receive(void)
{
    return HAL_HalfDuplex_EnableReceiver(&huart7)==HAL_OK &&
           HAL_UART_Receive_IT(&huart7,&s_rx_byte,1U)==HAL_OK;
}

bool ServoUart_Recover(void)
{
    uint32_t p=lock();
    s_status.ready=false; s_status.transmitting=false;
    (void)HAL_UART_Abort(&huart7); /* UART7 only, no DMA; restore HAL states after errors/timeouts. */
    __HAL_UART_CLEAR_FLAG(&huart7,UART_CLEAR_PEF|UART_CLEAR_FEF|UART_CLEAR_NEF|UART_CLEAR_OREF);
    __HAL_UART_SEND_REQ(&huart7,UART_RXDATA_FLUSH_REQUEST);
    s_head=s_tail=0; s_events=0;
    s_status.ready=receive();
    bool ok=s_status.ready;
    if(!ok)++s_status.errors;
    __set_PRIMASK(p);
    return ok;
}
bool ServoUart_Init(void)
{
    s_task=osThreadGetId();
    return ServoUart_Recover();
}

bool ServoUart_Send(const char *data, uint16_t length)
{
    if(!data || !length || length>sizeof(s_tx))return false;
    uint32_t p=lock();
    if(!s_status.ready || s_status.transmitting) { __set_PRIMASK(p); return false; }
    s_status.transmitting=true;
    bool ok=HAL_UART_AbortReceive(&huart7)==HAL_OK;
    s_head=s_tail=0; s_events=0;
    __HAL_UART_SEND_REQ(&huart7,UART_RXDATA_FLUSH_REQUEST);
    memcpy(s_tx,data,length);
    ok=ok && HAL_HalfDuplex_EnableTransmitter(&huart7)==HAL_OK &&
       HAL_UART_Transmit_IT(&huart7,s_tx,length)==HAL_OK;
    if(ok)++s_status.tx_frames;
    else { s_status.transmitting=false; error(); }
    __set_PRIMASK(p);
    return ok;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart!=&huart7 || !s_status.transmitting)return;
    /* HAL invokes this on USART TC, not merely TXE: final stop bit has left the pin. */
    s_status.transmitting=false;
    s_status.tx_done_ms=HAL_GetTick();
    if(!receive())error();
    else s_events|=SERVO_UART_TX_DONE;
    notify();
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart!=&huart7 || !s_status.ready || s_status.transmitting)return;
    s_status.last_rx_ms=HAL_GetTick(); ++s_status.rx_bytes;
    uint16_t next=(uint16_t)((s_head+1U)%SERVO_RX_RING_SIZE);
    if(next==s_tail) { ++s_status.overflows; error(); return; }
    s_ring[s_head]=s_rx_byte; __DMB(); s_head=next;
    if(HAL_UART_Receive_IT(&huart7,&s_rx_byte,1U)!=HAL_OK)error();
    notify();
}
/* fxi6x.c owns the one global HAL error callback; unrelated UARTs are forwarded here. */
void Bsp_UnhandledUartError(UART_HandleTypeDef *huart)
{
    if(huart==&huart7)error();
}

uint16_t ServoUart_Read(uint8_t *out, uint16_t capacity)
{
    if(!out || !capacity)return 0;
    uint32_t p=lock(); uint16_t n=0;
    while(n<capacity && s_tail!=s_head) { out[n++]=s_ring[s_tail]; s_tail=(uint16_t)((s_tail+1U)%SERVO_RX_RING_SIZE); }
    __set_PRIMASK(p); return n;
}
uint32_t ServoUart_TakeEvents(void)
{
    uint32_t p=lock(), events=s_events; s_events=0; __set_PRIMASK(p); return events;
}
void ServoUart_CopyStatus(ServoUartStatus *out)
{
    if(out) { uint32_t p=lock(); *out=s_status; __set_PRIMASK(p); }
}
