#include "servo_rc.h"
#if SERVO_RC_ENABLE && SERVO_BUS_ENABLE
#include "servo_app.h"
#include "fxi6x.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <string.h>

typedef struct { int direction; float speed_deg_s; } AxisConfig;
static const AxisConfig axes[5]={SERVO_RC_AXIS_TABLE};
static ServoRcStatus state, published;
static uint8_t slots[5], modes[5];
static uint16_t last_p[5];
static uint32_t last_control, center_ms;
static bool centering;

static bool rc_valid(const Fxi6x_Data *rc,uint32_t now)
{
    return rc->received && rc->valid && (uint32_t)(now-rc->last_valid_ms)<SERVO_RC_TIMEOUT_MS &&
        rc->switch_c>=-1 && rc->switch_c<=1 &&
        rc->rocker_l_v>=-660 && rc->rocker_l_v<=660 &&
        rc->rocker_l_h>=-660 && rc->rocker_l_h<=660 &&
        rc->rocker_r_h>=-660 && rc->rocker_r_h<=660;
}
bool ServoRc_Selected(uint32_t now)
{
    Fxi6x_Data rc;Fxi6x_CopyData(&rc);
    return rc_valid(&rc,now) && rc.switch_c!=1;
}
static uint8_t slot_mask(uint8_t ids)
{
    uint8_t mask=0;
    for(unsigned i=0;i<5;++i)if((ids&(1U<<i)) && slots[i]<SERVO_COUNT)mask|=(uint8_t)(1U<<slots[i]);
    return mask;
}
static void publish(uint32_t now)
{
    state.now_ms=now;
    taskENTER_CRITICAL();published=state;taskEXIT_CRITICAL();
}
void ServoRc_CopyStatus(ServoRcStatus *out)
{ if(out) { taskENTER_CRITICAL();*out=published;taskEXIT_CRITICAL(); } }
void ServoRc_Init(uint32_t now)
{
    memset(&state,0,sizeof(state));memset(modes,0,sizeof(modes));memset(last_p,0,sizeof(last_p));
    last_control=now;centering=false;
    state.config_valid=SERVO_COUNT==5U && SERVO_RC_PERIOD_MS>0U && SERVO_RC_PERIOD_MS<=SERVO_RC_MAX_PERIOD_MS &&
        SERVO_RC_DEADBAND>=0 && SERVO_RC_DEADBAND<660 && SERVO_RC_CENTER_HOLD_MS>=300U;
    for(unsigned id=0;id<5;++id) {
        slots[id]=SERVO_COUNT;
        for(unsigned j=0;j<SERVO_COUNT;++j)if(Servo_GetConfig(j)->id==id+1U)slots[id]=(uint8_t)j;
        if(slots[id]>=SERVO_COUNT || (axes[id].direction!=1 && axes[id].direction!=-1) ||
            !isfinite(axes[id].speed_deg_s) || axes[id].speed_deg_s<=0.0f)state.config_valid=false;
    }
    publish(now);
}
static float fraction(int value)
{
    if(value>SERVO_RC_DEADBAND)return (float)(value-SERVO_RC_DEADBAND)/(660-SERVO_RC_DEADBAND);
    if(value<-SERVO_RC_DEADBAND)return (float)(value+SERVO_RC_DEADBAND)/(660-SERVO_RC_DEADBAND);
    return 0.0f;
}
static bool fresh(const ServoState *s,uint32_t now)
{
    return s->online && s->position_seen && (uint32_t)(now-s->position_ms)<SERVO_POSITION_STALE_MS &&
        s->mode_valid && s->mode>=1U && s->mode<=4U;
}
static void fault(unsigned id)
{
    uint8_t bit=(uint8_t)(1U<<id);
    state.armed_mask &= (uint8_t)~bit;state.fault_mask |= bit;
    Servo_RcStopMask(slot_mask(bit));
}
void ServoRc_Process(uint32_t now)
{
    Fxi6x_Data rc;Fxi6x_CopyData(&rc);
    bool valid=rc_valid(&rc,now);
    uint8_t selected=valid && rc.switch_c!=1 ? (rc.switch_c==0 ? 7U : 24U) : 0U;
    /* Claiming rejects conflicting USB motion even while waiting for first OFF/centering. */
    Servo_RcClaim(slot_mask(selected));
    if(selected!=state.selected_mask) {
        uint8_t stop=state.selected_mask;
        if(state.off_seen)stop|=selected;
        if(state.off_seen || state.armed_mask)Servo_RcStopMask(slot_mask(stop));
        state.selected_mask=selected;state.armed_mask=0;centering=false;last_control=now;
    }
    if(!valid || !state.config_valid) {
        Servo_RcStopMask(slot_mask(state.armed_mask));
        state.armed_mask=0;state.off_seen=false;centering=false;last_control=now;publish(now);return;
    }
    if(rc.switch_c==1) {
        state.off_seen=true;state.armed_mask=state.fault_mask=0;
        Servo_RcClearInhibit();centering=false;last_control=now;publish(now);return;
    }
    if(!state.off_seen) { publish(now);return; }
    const int input[5]={rc.rocker_l_v,rc.rocker_l_h,rc.rocker_r_h,rc.rocker_l_v,rc.rocker_l_h};
    bool centered=true;
    for(unsigned i=0;i<5;++i)if((selected&(1U<<i)) && fraction(input[i])!=0.0f)centered=false;
    if(!centered)centering=false;
    else if(!centering) { centering=true;center_ms=now; }
    uint8_t inhibited=Servo_RcInhibited();
    for(unsigned i=0;i<5;++i)if(inhibited&(1U<<slots[i])) {
        state.fault_mask|=(uint8_t)(1U<<i);state.armed_mask&=(uint8_t)~(1U<<i);
    }
    /* Switching groups drains their stop transactions before the new group can arm. */
    bool ready=centering && (uint32_t)(now-center_ms)>=SERVO_RC_CENTER_HOLD_MS && !Servo_RcStopPending();
    uint32_t elapsed=now-last_control;
    bool update=elapsed>=SERVO_RC_PERIOD_MS;
    if(update && elapsed>SERVO_RC_MAX_PERIOD_MS) {
        Servo_RcStopMask(slot_mask(state.armed_mask));state.armed_mask=0;state.off_seen=false;
        centering=false;last_control=now;publish(now);return;
    }
    for(unsigned i=0;i<5;++i) {
        uint8_t bit=(uint8_t)(1U<<i);
        if(!(selected&bit) || (state.fault_mask&bit))continue;
        ServoState s;bool seen=Servo_CopyState((uint8_t)(i+1U),&s);
        const ServoConfig *c=Servo_GetConfig(slots[i]);
        bool usable=seen && fresh(&s,now) && s.position>=c->minimum && s.position<=c->maximum;
        if(!(state.armed_mask&bit)) {
            if(ready && usable) {
                state.target_p[i]=(float)s.position;last_p[i]=s.position;modes[i]=s.mode;state.armed_mask|=bit;
            }
            continue;
        }
        if(!usable || s.mode!=modes[i]) { fault(i);continue; }
        if(!update)continue;
        float span=s.mode<=2U ? 270.0f : 180.0f;
        float p=state.target_p[i]+fraction(input[i])*(float)axes[i].direction*axes[i].speed_deg_s*
            ((float)elapsed*0.001f)*(2000.0f/span);
        if(p<c->minimum)p=(float)c->minimum;
        if(p>c->maximum)p=(float)c->maximum;
        uint16_t rounded=(uint16_t)lroundf(p);
        if(rounded!=last_p[i]) {
            if(Servo_RcMovePulse((uint8_t)(i+1U),rounded,SERVO_RC_PERIOD_MS,NULL)!=SERVO_QUEUED) { fault(i);continue; }
            last_p[i]=rounded;
        }
        state.target_p[i]=p; /* Preserve fractional increments; center holds the last target. */
    }
    if(update)last_control=now;
    publish(now);
}
#endif
