/* Real four-wheel task, feedback parser and 0x200 packer, simulated scheduler/CAN. */
#include "campus_app.h"
#include "motor_debug_config.h"
#include "can.h"
#include "task.h"
#include <assert.h>
#include <math.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
FDCAN_HandleTypeDef hfdcan1;
static uint32_t now;
static unsigned scenario, rx_slot, pending, depth, tx_count;
static jmp_buf done;
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *,uint32_t);
uint32_t HAL_GetTick(void) { return now; }
void test_enter(void) { ++depth; }
void test_exit(void) { assert(depth); --depth; }
bool Can_Init(FDCAN_HandleTypeDef *h) { return h==&hfdcan1; }
bool Can_ProcessRecovery(bool valid,bool off,uint32_t tick,uint8_t mask)
{ assert(valid && !off && tick==now && mask==15); return true; }
uint8_t Can_RecoveryOutputMask(void) { return scenario==5 && now>=900 && now<1150 ? 5U : 15U; }
void Can_RecoveryFeedback(uint8_t slot) { assert(slot==rx_slot); }
void Can_CopyRecovery(CanRecoveryStatus *out) { *out=(CanRecoveryStatus){.feedback_mask=Can_RecoveryOutputMask()}; }
bool Can_Send(uint32_t id,const uint8_t *data,uint8_t size)
{
    assert(id==0x200 && size==8 && !depth); ++tx_count;
    int16_t raw[4];
    for(unsigned i=0;i<4;++i) {
        raw[i]=(int16_t)(((uint16_t)data[2*i]<<8)|data[2*i+1]);
        if(!MOTOR_DEBUG_ENABLE || (now>=1200 && now<1505)) assert(raw[i]==0);
        if(MOTOR_DEBUG_ENABLE && now>=950 && now<1200) {
            bool stopped=(scenario==1 && i==0) || (scenario==2 && i==1) ||
                (scenario==3 && i==2) || (scenario==4 && i==3) || scenario==6 ||
                (scenario==5 && now<1150 && (i==1 || i==3));
            if(stopped) assert(raw[i]==0); else assert(raw[i]>0);
        }
    }
    if(MOTOR_DEBUG_ENABLE && scenario==0 && now==1000)
        assert(raw[0]<raw[1] && raw[1]<raw[2] && raw[2]<raw[3]);
    return true;
}
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(FDCAN_HandleTypeDef *h,FDCAN_ProtocolStatusTypeDef *p)
{ assert(h==&hfdcan1); *p=(FDCAN_ProtocolStatusTypeDef){0}; return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_GetErrorCounters(FDCAN_HandleTypeDef *h,FDCAN_ErrorCountersTypeDef *e)
{ assert(h==&hfdcan1); *e=(FDCAN_ErrorCountersTypeDef){0}; return HAL_OK; }
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *h,uint32_t f)
{ (void)h; (void)f; return pending; }
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *h,uint32_t f,FDCAN_RxHeaderTypeDef *head,uint8_t *data)
{
    (void)h; (void)f;
    *head=(FDCAN_RxHeaderTypeDef){.Identifier=0x201+rx_slot,.DataLength=8};
    memset(data,0,8); data[6]=25;
    if(scenario==2 && rx_slot==1 && now>=900 && now<1150) data[6]=MOTOR_DEBUG_MAX_TEMPERATURE;
    if(scenario==3 && rx_slot==2 && now>=900 && now<1150) { data[2]=12000>>8; data[3]=12000&255; }
    pending=0; return HAL_OK;
}
bool Fxi6x_Init(void) { return true; }
bool Fxi6x_Update(void) { return true; }
void Fxi6x_CopyData(Fxi6x_Data *rc)
{
    *rc=(Fxi6x_Data){.received=true,.valid=!(scenario==6 && now>=900 && now<910),.last_valid_ms=now,
        .switch_a=(now<5 || (now>=1200 && now<1205))?1:-1,
        .rocker_l_v=((now>400 && now<1200) || now>=1510)?330:0,.rocker_r_v=-660};
    if (scenario==8 && now<1200) {
        rc->switch_b = now>=900 ? FXI6X_HIGH : FXI6X_LOW;
        rc->rocker_l_v = 0;
        rc->rocker_l_h = now<350 ? 41 : 0; /* CH4 单独偏移也会阻止启用。 */
        if (now>=700) {
            rc->rocker_l_h = 350;
            if (now>=750) rc->rocker_l_v = 350;
            if (now>=800) rc->rocker_l_h = -40;
            if (now>=850 && now<900) rc->rocker_l_h = -660;
        }
    }
}
TickType_t xTaskGetTickCount(void) { return now; }
void vTaskDelayUntil(TickType_t *wake,TickType_t delta)
{
    *wake+=delta; now=*wake;
    if (scenario==8 && MOTOR_DEBUG_ENABLE && now<=901) {
        CampusMotorDebugSnapshot d; Campus_CopyMotorDebug(&d);
        assert(d.mecanum_mode==(now<=900));
        if (now==600) assert(!d.running_mask);
        if (now>=701) {
            const float half=MOTOR_DEBUG_MAX_SPEED*0.5f;
            float plus=half, minus=-half;
            if (now>750) { plus=2*half; minus=0; }
            if (now>800) { plus=minus=half; }
            if (now>850) { plus=-2*half/3; minus=2*half; }
            if (now>900) { plus=minus=MOTOR_DEBUG_MAX_SPEED*350/660; }
            assert(d.running_mask==15);
            for (unsigned i=0;i<4;++i)
                assert(fabsf(d.wheel[i].control.target-((i==0 || i==3)?plus:minus))<0.001f);
        }
    }
    if(now==949 || now==948 || now==1199) {
        CampusMotorDebugSnapshot d; Campus_CopyMotorDebug(&d);
        if(MOTOR_DEBUG_ENABLE && scenario==1) {
            /* Snapshot is from preceding iteration: age 99 accepted, age 100 stops. */
            if(now==949) assert(d.wheel[0].control.state==MOTOR_DEBUG_RUNNING);
            if(now==1199) assert(d.wheel[0].control.state!=MOTOR_DEBUG_RUNNING);
        }
    }
    if(now>=(scenario==7?61000U:2000U)) longjmp(done,1);
    for(rx_slot=0;rx_slot<4;++rx_slot) {
        if((scenario==1 && rx_slot==0 && now>=850 && now<1150) || (scenario==4 && rx_slot==3)) continue;
        pending=1; HAL_FDCAN_RxFifo0Callback(&hfdcan1,0);
    }
}
int32_t osDelay(uint32_t n) { (void)n; return 0; }
uint32_t osThreadFlagsWait(uint32_t a,uint32_t b,uint32_t c) { (void)a;(void)b;(void)c;return 0; }
bool Led_Init(void) { return true; }
bool Led_SetRGB(uint8_t a,uint8_t b,uint8_t c) { (void)a;(void)b;(void)c;return true; }
int main(void)
{
    for(scenario=0;scenario<9;++scenario) {
        now=pending=tx_count=depth=0; Campus_Init();
        if(setjmp(done)==0) Campus_ChassisTask(NULL);
        CampusMotorDebugSnapshot d; Campus_CopyMotorDebug(&d);
        assert(tx_count==now && d.active_mask==15);
        assert(d.running_mask==(MOTOR_DEBUG_ENABLE?(scenario==4?7:15):0));
        assert(d.wheel[0].control.integral_a==0); /* Ki1=0, other wheels use independent integrators. */
        if(MOTOR_DEBUG_ENABLE) assert(d.wheel[1].control.integral_a>0);
    }
    puts("PASS: four independent gains/states, local faults, missing wheel, 100ms boundary, rearm, recovery output mask, 61s operation and one CAN frame/tick.");
    return 0;
}
