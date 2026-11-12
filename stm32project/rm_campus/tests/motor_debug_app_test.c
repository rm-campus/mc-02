/* Run the real chassis task, M3508 parser and CAN packer against simulated IO. */
#include "campus_app.h"
#include "motor_debug_config.h"
#include "can.h"
#include "task.h"
#include "usb_serial.h"
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>

FDCAN_HandleTypeDef hfdcan3;
static uint32_t now;
static unsigned scenario, pending, nonzero, depth;
static jmp_buf finished;
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *, uint32_t);

uint32_t HAL_GetTick(void) { return now; }
void test_enter(void) { ++depth; }
void test_exit(void) { assert(depth); --depth; }
bool Can_Init(FDCAN_HandleTypeDef *handle) { return handle==&hfdcan3; }
bool Can_Send(uint32_t id, const uint8_t *data, uint8_t length)
{
    assert(id==0x200 && length==8 && !depth);
    for(unsigned slot=0;slot<4;++slot) {
        int16_t raw=(int16_t)(((uint16_t)data[slot*2]<<8)|data[slot*2+1]);
        if(!MOTOR_DEBUG_ENABLE || slot!=MOTOR_DEBUG_MOTOR_ID-1U) assert(raw==0);
        if(scenario==5 || ((scenario>=1 && scenario<=4) && now>=901)) assert(raw==0);
        if(raw!=0) ++nonzero;
    }
    return !(scenario==2 && now==900);
}
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(FDCAN_HandleTypeDef *h,
                                             FDCAN_ProtocolStatusTypeDef *p)
{
    assert(h==&hfdcan3); p->BusOff=scenario==4 && now>=900; return HAL_OK;
}
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *h,uint32_t fifo)
{ (void)h; (void)fifo; return pending; }
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *h, uint32_t fifo,
                                        FDCAN_RxHeaderTypeDef *header, uint8_t *data)
{
    (void)h; (void)fifo;
    *header=(FDCAN_RxHeaderTypeDef){.Identifier=0x200+MOTOR_DEBUG_MOTOR_ID,
        .IdType=FDCAN_STANDARD_ID,.RxFrameType=FDCAN_DATA_FRAME,
        .FDFormat=FDCAN_CLASSIC_CAN,.DataLength=FDCAN_DLC_BYTES_8};
    memset(data,0,8); data[6]=25; pending=0; return HAL_OK;
}
bool Fxi6x_Init(void) { return true; }
bool Fxi6x_Update(void) { return true; }
void Fxi6x_CopyData(Fxi6x_Data *rc)
{
    *rc=(Fxi6x_Data){.received=true,.valid=!(scenario==1 && now>=900 && now<910),
        .last_valid_ms=now,.switch_a=(now<5 && scenario!=5)?-1:1,
        .rocker_r_v=now>400?660:0};
}
TickType_t xTaskGetTickCount(void) { return now; }
void vTaskDelayUntil(TickType_t *wake,TickType_t delta)
{
    *wake+=delta; now=*wake;
    if(now>=1200) longjmp(finished,1);
    if(!(scenario==3 && now>=850)) {
        pending=1; HAL_FDCAN_RxFifo0Callback(&hfdcan3,0);
    }
}
int32_t osDelay(uint32_t ticks) { (void)ticks; return 0; }
uint32_t osThreadFlagsWait(uint32_t a,uint32_t b,uint32_t c)
{ (void)a; (void)b; (void)c; return 0; }
bool Led_Init(void) { return true; }
bool Led_SetRGB(uint8_t r,uint8_t g,uint8_t b) { (void)r;(void)g;(void)b;return true; }
UsbSerialResult UsbSerial_Printf(const char *format,...) { (void)format;return USB_SERIAL_OK; }

int main(void)
{
    for(scenario=0;scenario<=5;++scenario) {
        now=0; pending=0; nonzero=0; depth=0;
        Campus_Init();
        if(setjmp(finished)==0) Campus_ChassisTask(NULL);
        assert(!depth);
        CampusMotorDebugSnapshot s; Campus_CopyMotorDebug(&s);
        if(!MOTOR_DEBUG_ENABLE) {
            assert(nonzero==0 && s.control.state==MOTOR_DEBUG_DISABLED);
        } else if(scenario==5) {
            assert(nonzero==0 && s.control.state!=MOTOR_DEBUG_RUNNING);
        } else {
            assert(nonzero>0);
            if(scenario==0) assert(s.control.state==MOTOR_DEBUG_RUNNING);
            else assert(s.control.command_raw==0 && s.control.state!=MOTOR_DEBUG_RUNNING);
        }
    }
    puts("PASS: real chassis task and CAN frames, selected motor only, default off, RC/feedback/CAN faults and no auto-restart.");
    return 0;
}
