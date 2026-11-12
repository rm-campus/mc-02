#include "can.h"
#include "motor_debug.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static FDCAN_GlobalTypeDef regs;
static FDCAN_HandleTypeDef h={.Instance=&regs}, other={.Instance=&regs};
static unsigned depth, stops, starts, notifications, tx, fifo, failure;
static bool stop_dirty, rx_dirty, filter_fail, physical_bus_off;
static uint8_t data[8];
void enter(void) { ++depth; }
void leave(void) { assert(depth); --depth; }
int HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *handle,FDCAN_FilterTypeDef *f)
{ assert(handle==&h && f->FilterID1==0x201 && f->FilterID2==0x204); return filter_fail ? HAL_ERROR : HAL_OK; }
int HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *handle,uint32_t a,uint32_t b,uint32_t c,uint32_t d)
{ (void)handle; assert(a==FDCAN_REJECT && b==a && c==FDCAN_REJECT_REMOTE && d==c); return HAL_OK; }
int HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *handle,uint32_t flags,uint32_t indexes)
{ assert(handle==&h && indexes==0); notifications=flags; return HAL_OK; }
int HAL_FDCAN_Start(FDCAN_HandleTypeDef *handle)
{
    assert(handle==&h && h.State==0); ++starts;
    if(failure==3) return HAL_ERROR;
    h.State=1; regs.CCCR=0; return HAL_OK; /* Does NOT complete bus recovery. */
}
int HAL_FDCAN_Stop(FDCAN_HandleTypeDef *handle)
{
    assert(handle==&h && h.State==1 && depth); ++stops;
    if(failure==1) return HAL_ERROR;
    regs.CCCR=FDCAN_CCCR_INIT; h.State=0;
    if(!stop_dirty) regs.TXBRP=0;
    if(!rx_dirty) fifo=0;
    return HAL_OK;
}
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *handle,uint32_t location)
{ assert(handle==&h && location==0); return fifo; }
int HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *handle,FDCAN_TxHeaderTypeDef *head,uint8_t *bytes)
{
    assert(handle==&h && head->Identifier==0x200 && head->DataLength==8);
    assert(depth && !physical_bus_off && !(regs.CCCR&FDCAN_CCCR_INIT));
    assert(bytes[0]==data[0]); ++tx; regs.TXBRP=1; return HAL_OK;
}
static CanRecoveryStatus status(void) { CanRecoveryStatus s; Can_CopyRecovery(&s); return s; }
static bool poll(uint32_t now) { return Can_ProcessRecovery(true,physical_bus_off,now,1U); }
static void bus_off(void)
{ physical_bus_off=true; regs.CCCR|=FDCAN_CCCR_INIT; HAL_FDCAN_ErrorStatusCallback(&h,FDCAN_IT_BUS_OFF); }
static void bus_online(void)
{ physical_bus_off=false; regs.CCCR=0; HAL_FDCAN_ErrorStatusCallback(&h,FDCAN_IT_BUS_OFF); }
static void boot(void)
{
    depth=stops=starts=tx=fifo=failure=0; stop_dirty=rx_dirty=filter_fail=physical_bus_off=false;
    regs=(FDCAN_GlobalTypeDef){0}; h.State=0;
    assert(Can_Init(&h)); assert(notifications&FDCAN_IT_BUS_OFF); assert(poll(0));
}
int main(void)
{
    boot(); assert(Can_Send(0x200,data,8));
    HAL_FDCAN_ErrorStatusCallback(&other,FDCAN_IT_BUS_OFF); assert(Can_Send(0x200,data,8));
    bus_off(); unsigned before=tx;
    assert(!Can_Send(0x200,data,8)); assert(stops==0); /* ISR only signals. */
    fifo=4; assert(!poll(1));
    assert(stops==1 && starts==2 && regs.TXBRP==0 && fifo==0);
    assert(status().state==CAN_RECOVERY_WAIT_BUS && status().success==0);
    assert(!poll(2) && !Can_Send(0x200,data,8) && tx==before);
    bus_online(); assert(!poll(3)); assert(status().state==CAN_RECOVERY_WAIT_FEEDBACK);
    Can_RecoveryFeedback(1); assert(!poll(4)); /* Another motor cannot confirm ID1 recovery. */
    Can_RecoveryFeedback(0); assert(poll(5));
    assert(status().success==1 && status().last_duration_ms==4);
    assert(Can_Send(0x200,data,8));
    assert(!Can_ProcessRecovery(false,false,6,1)); assert(!Can_Send(0x200,data,8));
    assert(poll(7));

    /* Fault repeats quickly: wait until 100ms since last Stop/Start, no tight restart loop. */
    bus_off(); assert(!poll(8)); assert(stops==1);
    assert(!poll(100) && stops==1); assert(!poll(101) && stops==2);
    for(unsigned t=102;t<400;++t) assert(!poll(t));
    assert(stops==2); /* INIT=0 hardware recovery in progress: do not interrupt it. */
    bus_online(); Can_RecoveryFeedback(0); assert(poll(400));
    assert(status().bus_off_count==2 && status().success==2);

    for(unsigned kind=1;kind<=4;++kind) {
        boot(); regs.TXBRP=3; fifo=2; bus_off();
        failure=kind==1 ? 1 : kind==3 ? 3 : 0;
        stop_dirty=kind==2; rx_dirty=kind==4;
        assert(!poll(1));
        assert(status().state==CAN_RECOVERY_FAILED && status().failures==1);
        assert(status().error==(kind==4 ? 2 : kind));
        assert(!poll(1000) && !Can_Send(0x200,data,8));
    }
    boot(); bus_off(); assert(!poll(UINT32_MAX-50U));
    bus_online(); Can_RecoveryFeedback(0); assert(poll(UINT32_MAX-48U));
    bus_off(); assert(!poll(10)); assert(stops==1);
    assert(!poll(49) && stops==2); /* Millisecond wrap-safe retry deadline. */

    boot(); bus_off(); assert(!Can_ProcessRecovery(true,true,1,15));
    bus_online(); Can_RecoveryFeedback(0);
    assert(Can_ProcessRecovery(true,false,3,15));
    assert(Can_RecoveryOutputMask()==1); /* One missing wheel cannot block the others. */
    Can_RecoveryFeedback(2); assert(Can_RecoveryOutputMask()==5);
    Can_RecoveryFeedback(1); assert(Can_RecoveryOutputMask()==7);
    bus_off(); assert(Can_RecoveryOutputMask()==0);

    /* Controller remains RUNNING during a short recovery when CAN_FAULT is disabled,
     * but the 100ms feedback timeout still latches MOTOR_OFFLINE. */
    MotorDebug d; MotorDebug_Init(&d); MotorDebugConfig c=MotorDebug_DefaultConfig();
    MotorDebugInput in={.rc_valid=true,.rc_received=true,.motor_received=true,.can_ok=true,
        .enable_switch=1,.period_ms=1,.temperature=25};
    for(unsigned n=0;n<350;++n) {
        in.now_ms=n; in.rc_ms=in.feedback_ms=n; in.enable_switch=n<2?1:-1;
        MotorDebug_Step(&d,&c,&in);
    }
    assert(d.state==MOTOR_DEBUG_RUNNING);
    in.can_ok=false;
    for(unsigned n=350;n<=449;++n) {
        in.now_ms=in.rc_ms=n; MotorDebug_Step(&d,&c,&in);
        if(n<449) assert(d.state==MOTOR_DEBUG_RUNNING);
    }
    assert(d.reason==MOTOR_DEBUG_MOTOR_OFFLINE);
    in.can_ok=true; in.now_ms=in.rc_ms=in.feedback_ms=450;
    MotorDebug_Step(&d,&c,&in); assert(d.state==MOTOR_DEBUG_WAIT_OFF);
    puts("PASS: deferred Stop/Start, old FIFO reset, fresh selected feedback, retry/race gates, HAL failures, wrap and unchanged motor restart policy.");
    return 0;
}
