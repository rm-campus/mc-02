/* Real UART/queue/controller/USB paths, with the deterministic bus fixture. */
#define main legacy_servo_main
#include "servo_test.c"
#undef main
#include "servo_rc.h"
#include "servo_rc_usb.h"
#include <math.h>
static Fxi6x_Data receiver={.received=true,.valid=true,.switch_c=1};
static bool freeze_rc;
void Fxi6x_CopyData(Fxi6x_Data *out)
{ if(!freeze_rc)receiver.last_valid_ms=tick;*out=receiver; }
static ServoRcStatus control(void)
{ ServoRcStatus s;ServoRc_CopyStatus(&s);return s; }
static void center(void)
{receiver.rocker_l_v=receiver.rocker_l_h=receiver.rocker_r_h=0;}
static void rearm(int group)
{ center();receiver.switch_c=1;run(250);receiver.switch_c=(int8_t)group;run(400); }
static unsigned rows(unsigned columns)
{
    unsigned count=0,commas=0;
    for(char *p=output;*p;++p) {
        if(*p==',')++commas;
        if(*p=='\n') { if(commas+1==columns)++count;commas=0; }
    }
    return count;
}
static void clear_output(void) {output_len=0;output[0]=0;}
int main(void)
{
    receiver.switch_c=0;assert(Servo_Init());run(800);
    assert(!moves); /* Power-up in selected group is never an implicit arm. */
    if(!SERVO_BUS_ENABLE) { assert(!tx_count);puts("PASS: RC with bus disabled.");return 0; }
    if(!SERVO_RC_ENABLE) {
        assert(Servo_MovePulse(1,1510,100,NULL)==SERVO_QUEUED);run(100);assert(positions[0]==1510);
        puts("PASS: RC disabled, manual servo API retained.");return 0;
    }
    assert(control().config_valid && !control().off_seen && !control().armed_mask);
    receiver.switch_c=1;run(50);receiver.switch_c=0;receiver.rocker_r_h=61;run(500);
    assert(!control().armed_mask);center();run(299);assert(!control().armed_mask);run(10);
    assert(control().armed_mask==7);
    assert(Servo_MovePulse(1,2000,100,NULL)==SERVO_BUSY);
    const float full_travel=SERVO_MAX_SPEED_DEG_S*1000.0f/270.0f;
    const float half_stick_travel=full_travel*0.45f;
    receiver.rocker_l_v=660;receiver.rocker_l_h=-660;receiver.rocker_r_h=330;run(500);
    assert(fabsf(positions[0]-(1500+full_travel))<=7.0f);
    assert(fabsf(positions[1]-(1500-full_travel))<=7.0f);
    assert(fabsf(positions[2]-(1500+half_stick_travel))<=5.0f);
    assert(positions[3]==1500 && positions[4]==1500);
    center();run(80);uint16_t held=positions[0];run(300);assert(positions[0]==held);
    /* Switching with a held stick stops the old group and waits for the new axes. */
    receiver.switch_c=-1;receiver.rocker_l_v=660;run(500);assert(!control().armed_mask);
    assert(positions[0]==held && positions[3]==1500);center();run(400);assert(control().armed_mask==24);
    receiver.rocker_l_v=660;receiver.rocker_l_h=-660;receiver.rocker_r_h=660;run(500);
    assert(fabsf(positions[3]-(1500+full_travel))<=7.0f);
    assert(fabsf(positions[4]-(1500-full_travel))<=7.0f && positions[0]==held);
    /* Manual halt must latch, including removal of pending RC moves. */
    assert(Servo_Pause(4,NULL)==SERVO_QUEUED);run(80);held=positions[3];run(400);
    assert(positions[3]==held && (control().fault_mask&8) && (control().armed_mask&16));
    assert(Servo_Resume(4,NULL)==SERVO_BUSY);
    rearm(-1);assert(control().armed_mask==24);
    /* Mid-stream loss affects one servo, reconnect cannot replay an old target. */
    receiver.rocker_l_v=660;receiver.rocker_l_h=660;reply_mask&=~8U;run(700);
    assert(!(control().armed_mask&8) && (control().fault_mask&8) && (control().armed_mask&16));
    reply_mask|=8U;run(1500);assert(!(control().armed_mask&8));
    rearm(0);assert(control().armed_mask==7);
    /* A timed-out motion must discard any newer same-axis target already queued. */
    receiver.rocker_l_v=660;drop_move_tc=true;run(100);drop_move_tc=false;run(200);
    assert((control().fault_mask&1) && !(control().armed_mask&1));
    held=positions[0];run(300);assert(positions[0]==held);
    rearm(0);assert(control().armed_mask==7);
    receiver.rocker_l_v=receiver.rocker_l_h=receiver.rocker_r_h=660;run(100);
    assert(Servo_Stop(2,NULL)==SERVO_QUEUED);
    assert(Servo_SetTorque(3,false,NULL)==SERVO_QUEUED);run(100);
    uint16_t second=positions[1],third=positions[2];run(300);
    assert(positions[1]==second && positions[2]==third && (control().armed_mask&1) && (control().fault_mask&6)==6);
    /* Reserved stop slots remain available even when the normal request queue is full. */
    unsigned queued_queries=0;
    while(Servo_RequestPosition(1,NULL)==SERVO_QUEUED)++queued_queries;
    assert(queued_queries>0);receiver.switch_c=1;run(100);held=positions[0];run(400);
    assert(!control().armed_mask && !Servo_RcStopPending() && positions[0]==held);
    rearm(0);assert(control().armed_mask==7);
    receiver.rocker_l_v=660;freeze_rc=true;run(101);assert(!control().armed_mask && !control().off_seen);
    freeze_rc=false;run(400);assert(!control().armed_mask);rearm(0);assert(control().armed_mask==7);
    /* Maximum travel clamp and fractional accumulation for very small deflections. */
    receiver.rocker_l_v=660;run(9500);assert(positions[0]==2500);
    receiver.rocker_l_v=-660;run(18500);assert(positions[0]==500);
    receiver.rocker_l_v=61;run(4000);assert(positions[0]>=501 && positions[0]<=502);
    center();run(100);
    for(unsigned id=1;id<=5;++id)assert(state((uint8_t)id).online);
    /* Late control ticks stop, with no large catch-up jump. */
    tick+=100;run(1);assert(!control().armed_mask && !control().off_seen);
    rearm(0);assert(control().armed_mask==7);

    usb.ready=true;clear_output();run(1000);
#if SERVO_RC_TELEMETRY_ENABLE
    assert(rows(5)==49 && rows(9)==0 && rows(12)==0);
    /* Known feedback differs from last requested targets; two protocols' spans. */
    receiver.switch_c=1;run(200);positions[0]=1500;positions[1]=1000;positions[2]=2000;positions[3]=500;positions[4]=2500;
    modes[1]=3;assert(Servo_Init());receiver.switch_c=0;clear_output();run(800);
    assert(strstr(output,"135.000,45.000,202.500,0.000,270.000\n"));
    reply_mask&=~16U;clear_output();run(500);assert(strstr(output,"135.000,45.000,202.500,0.000,-1.000\n"));
    reply_mask|=16U;clear_output();input("servo_status\n");run(150);
    const char *begin=strstr(output,"SERVO_STATUS BEGIN"),*end=strstr(output,"SERVO_STATUS END");
    assert(begin&&end&&end>begin&&!memchr(begin,',',(size_t)(end-begin))&&strstr(begin,"RC enabled=1"));
    assert(rows(5)>0 && rows(9)==0);
    /* USB backpressure, disconnection and ownership drain on mode exit. */
    clear_output();usb_hold=true;run(100);assert(usb_len && rows(5)==0);
    receiver.switch_c=1;run(50);assert(!rows(9));usb_hold=false;run(30);assert(rows(9)>0);
    receiver.switch_c=-1;clear_output();run(100);assert(rows(5)>0);
    usb.ready=false;run(5);usb_len=0;++usb.session_id;usb.ready=true;clear_output();run(100);
    assert(rows(5)>0 && rows(9)==0);
#else
    assert(rows(5)==0 && rows(9)>0); /* Disabling servo telemetry preserves motor telemetry. */
#endif
    usb.ready=false;run(5);center();receiver.switch_c=1;tick=UINT32_MAX-150U;assert(Servo_Init());run(600);
    receiver.switch_c=-1;run(400);assert(control().armed_mask==24);
    assert(!depth && mask==0);
    puts("PASS: RC groups/axes, startup/centering, incremental hold/limits, manual arbitration, per-axis loss/rearm, feedback fairness, five-angle CSV/owners/session and tick wrap.");
    return 0;
}
