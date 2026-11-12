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

FDCAN_HandleTypeDef hfdcan1;
static uint32_t now;
static unsigned scenario, pending, nonzero, depth;
static unsigned injected, protocol_reads;
static jmp_buf finished;
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *, uint32_t);

uint32_t HAL_GetTick(void) { return now; }
void test_enter(void) { ++depth; }
void test_exit(void) { assert(depth); --depth; }
bool Can_Init(FDCAN_HandleTypeDef *handle) { return handle==&hfdcan1; }
/* 此套件验证控制策略；真实 Stop/Start 与恢复发送门由 can_recovery 套件覆盖。 */
bool Can_ProcessRecovery(bool valid,bool bus_off,uint32_t tick,uint8_t mask)
{ (void)valid; (void)bus_off; assert(mask==(1U<<(MOTOR_DEBUG_MOTOR_ID-1U))); return !(scenario==6 && tick>=900); }
void Can_RecoveryFeedback(uint8_t slot) { assert(slot==MOTOR_DEBUG_MOTOR_ID-1U); }
uint8_t Can_RecoveryOutputMask(void) { return 0x0FU; }
void Can_CopyRecovery(CanRecoveryStatus *out)
{ *out=(CanRecoveryStatus){.state=(scenario==4 && now>=900)?CAN_RECOVERY_WAIT_BUS:CAN_RECOVERY_READY}; }
bool Can_Send(uint32_t id, const uint8_t *data, uint8_t length)
{
    assert(id==0x200 && length==8 && !depth);
    assert(!(scenario==6 && now>=900)); /* Recovery pause does not enter the TX path. */
    for(unsigned slot=0;slot<4;++slot) {
        int16_t raw=(int16_t)(((uint16_t)data[slot*2]<<8)|data[slot*2+1]);
        if(!MOTOR_DEBUG_ENABLE || slot!=MOTOR_DEBUG_MOTOR_ID-1U) assert(raw==0);
        bool stopping_fault=scenario==1 || scenario==3 ||
            (MOTOR_DEBUG_CAN_FAULT_ENABLE && (scenario==2 || scenario==4));
        if(scenario==5 || (stopping_fault && now>=901)) assert(raw==0);
        if(raw!=0) ++nonzero;
    }
    return !(scenario==2 && now==900);
}
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(FDCAN_HandleTypeDef *h,
                                             FDCAN_ProtocolStatusTypeDef *p)
{
    assert(h==&hfdcan1); ++protocol_reads;
    p->BusOff=scenario==4 && now>=900;
    p->LastErrorCode=now==800 ? 3U : (scenario==4 && now>=900) ? 5U : 7U;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_GetErrorCounters(FDCAN_HandleTypeDef *h,FDCAN_ErrorCountersTypeDef *e)
{ assert(h==&hfdcan1); *e=(FDCAN_ErrorCountersTypeDef){.TxErrorCnt=4,.RxErrorCnt=2}; return HAL_OK; }
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *h,uint32_t fifo)
{ (void)h; (void)fifo; return pending; }
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *h, uint32_t fifo,
                                        FDCAN_RxHeaderTypeDef *header, uint8_t *data)
{
    (void)h; (void)fifo;
    *header=(FDCAN_RxHeaderTypeDef){.Identifier=0x200+MOTOR_DEBUG_MOTOR_ID,
        .IdType=FDCAN_STANDARD_ID,.RxFrameType=FDCAN_DATA_FRAME,
        .FDFormat=FDCAN_CLASSIC_CAN,.DataLength=FDCAN_DLC_BYTES_8};
    memset(data,0,8); data[6]=25; pending=0;
    if(injected==1) header->DataLength=7;
    if(injected==2) data[0]=0x20;
    if(injected==3) return 1;
    if(injected==4) header->Identifier=0x205;
    return HAL_OK;
}
bool Fxi6x_Init(void) { return true; }
bool Fxi6x_Update(void) { return true; }
void Fxi6x_CopyData(Fxi6x_Data *rc)
{
    *rc=(Fxi6x_Data){.received=true,.valid=!(scenario==1 && now>=900 && now<910),
        .last_valid_ms=now,.switch_a=(now<5 && scenario!=5)?1:-1,
        .rocker_l_v=now>400?660:0, .rocker_r_v=-660};
}
TickType_t xTaskGetTickCount(void) { return now; }
void vTaskDelayUntil(TickType_t *wake,TickType_t delta)
{
    *wake+=delta; now=*wake;
    if(now>=1200) longjmp(finished,1);
    if(!(scenario==3 && now>=800)) {
        pending=1; HAL_FDCAN_RxFifo0Callback(&hfdcan1,0);
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
    for(scenario=0;scenario<=6;++scenario) {
        now=0; pending=0; nonzero=0; depth=0;
        Campus_Init();
        if(setjmp(finished)==0) Campus_ChassisTask(NULL);
        assert(!depth);
        CampusMotorDebugSnapshot s; Campus_CopyMotorDebug(&s);
        CampusMotorStatusSnapshot report;
        unsigned reads=protocol_reads;
        Campus_CopyMotorStatus(&report);
        assert(protocol_reads==reads); /* Query never re-reads PSR. */
        assert(report.can.last_error==3 && report.can.last_error_ms==800);
        assert(report.can.status_valid && report.can.counters_valid);
        assert(report.can.tec==4 && report.can.rec==2);
        assert(report.can.valid_frames[MOTOR_DEBUG_MOTOR_ID-1U]>0);
        assert(report.can.bus_off==(scenario==4));
        assert(report.status.can_tx_failures==(scenario==2 ? 1U : 0U));
        if(!MOTOR_DEBUG_ENABLE) {
            assert(nonzero==0 && s.control.state==MOTOR_DEBUG_DISABLED);
        } else if(scenario==5) {
            assert(nonzero==0 && s.control.state!=MOTOR_DEBUG_RUNNING);
        } else {
            assert(nonzero>0);
            if(scenario==0 || scenario==6 || (!MOTOR_DEBUG_CAN_FAULT_ENABLE && (scenario==2 || scenario==4))) {
                assert(s.control.state==MOTOR_DEBUG_RUNNING && s.control.command_raw>0);
                assert(s.control.reason==MOTOR_DEBUG_OK);
            }
            else assert(s.control.command_raw==0 && s.control.state!=MOTOR_DEBUG_RUNNING);
        }
    }
    Campus_Init(); now=2000; injected=0; pending=1;
    HAL_FDCAN_RxFifo0Callback(&hfdcan1,0);
    for(injected=1;injected<=4;++injected) {
        ++now; pending=1; HAL_FDCAN_RxFifo0Callback(&hfdcan1,FDCAN_IT_RX_FIFO0_MESSAGE_LOST);
    }
    CampusMotorStatusSnapshot report; Campus_CopyMotorStatus(&report);
    assert(report.can.valid_frames[MOTOR_DEBUG_MOTOR_ID-1U]==1);
    assert(report.can.bad_encoder[MOTOR_DEBUG_MOTOR_ID-1U]==1);
    assert(report.can.bad_format==2 && report.can.read_fail==1 && report.can.fifo_lost_events==4);
    assert(report.motors.last_feedback_ms[MOTOR_DEBUG_MOTOR_ID-1U]==2000);
    printf("PASS: real chassis/CAN, debug=%d CAN_FAULT=%d, diagnostics retained and RC/feedback faults stop.\n",
           MOTOR_DEBUG_ENABLE, MOTOR_DEBUG_CAN_FAULT_ENABLE);
    return 0;
}
