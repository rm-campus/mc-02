#include "servo_app.h"
#include "robot_servo.h"
#include "servo_rc.h"
#include "arm_velocity.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include <string.h>

static const ServoConfig configs[SERVO_COUNT] = {SERVO_DEVICE_TABLE};
typedef struct { uint8_t slot; ZpOperation op; uint16_t position, time_ms; uint32_t ticket; ServoControlOwner owner; } Request;
typedef enum { IDLE, TX, REPLY } Phase;
static Request queue[SERVO_QUEUE_SIZE], current;
static uint16_t queued;
static uint32_t sequence;
static ServoSnapshot work, published;
static uint32_t next_poll[SERVO_COUNT], next_move[SERVO_COUNT], next_slot, deadline, guard, request_started;
static uint8_t boot_phase[SERVO_COUNT], poll_slot;
static Phase phase;
static bool auto_request, hardware_ready;
static char line[ZP_FRAME_MAX];
static uint16_t line_length;
static osThreadId_t thread;
static ServoControlOwner control_owner;
static uint8_t control_claimed;
/* 停止和故障按源保留，释放控制权不能撤销已经要求的停止。 */
static uint8_t control_stops[3], control_inhibited[3], control_manual_hold[3], control_stop_failed[3];
/* One USB batch at a time. ServoBusTask owns all UART traffic and publishes the result. */
static ServoModeReport mode_work, mode_done;
static uint8_t mode_slot, mode_end_slot, mode_attempt;
static uint32_t mode_due, mode_deadline, probe_tx_done_ms;
static bool mode_current, mode_verify, mode_yield_poll;
#define MODE_SETTLE_MS 100U
#define MODE_READ_ATTEMPTS 2U
#define MODE_BATCH_TIMEOUT_MS 4000U

static bool due(uint32_t now, uint32_t when) { return (int32_t)(now-when)>=0; }
static int slot_for(uint8_t id)
{
    for(unsigned i=0;i<SERVO_COUNT;++i)if(configs[i].id==id)return (int)i;
    return -1;
}
const ServoConfig *Servo_GetConfig(unsigned i) { return i<SERVO_COUNT ? &configs[i] : NULL; }
const char *Servo_ResultName(ServoResult r)
{
    static const char *names[]={"OK","QUEUED","BUSY","INVALID","RANGE","OFFLINE","BAD_MODE",
        "NOT_READY","DISABLED","TX_ERROR","UART_ERROR","TIMEOUT"};
    return (unsigned)r<sizeof(names)/sizeof(names[0]) ? names[r] : "UNKNOWN";
}
void Servo_CopySnapshot(ServoSnapshot *out)
{
    if(out) { taskENTER_CRITICAL(); *out=published; taskEXIT_CRITICAL(); }
}
bool Servo_CopyState(uint8_t id, ServoState *out)
{
    int i=slot_for(id); if(i<0 || !out)return false;
    taskENTER_CRITICAL(); *out=published.device[i]; taskEXIT_CRITICAL(); return true;
}
static void publish(uint32_t now)
{
    work.now_ms=now; ServoUart_CopyStatus(&work.uart);
    taskENTER_CRITICAL(); work.queued=queued; published=work; taskEXIT_CRITICAL();
}
static void remove_at(unsigned i)
{
    for(unsigned j=i+1;j<queued;++j)queue[j-1]=queue[j];
    --queued;
}
static void cancel_moves(unsigned slot)
{
    taskENTER_CRITICAL();
    for(unsigned i=0;i<queued;) {
        if(queue[i].slot==slot && (queue[i].op==ZP_MOVE || queue[i].op==ZP_RESUME || queue[i].op==ZP_TORQUE_ON)) {
            remove_at(i); ++work.cancelled;
        } else ++i;
    }
    taskEXIT_CRITICAL();
}
static bool owner_valid(ServoControlOwner owner)
{ return owner==SERVO_OWNER_RC || owner==SERVO_OWNER_ARM; }
static bool is_motion(ZpOperation op)
{ return op==ZP_MOVE || op==ZP_RESUME || op==ZP_TORQUE_ON; }
static bool is_halt(ZpOperation op)
{ return op==ZP_STOP || op==ZP_PAUSE || op==ZP_TORQUE_OFF; }
static bool motion_requires_reset(void)
{
#if ARM_VELOCITY_IK_OUTPUT_ENABLE
    /* 硬件IK超限后，即使重新初始化驱动或释放owner，也不能经手动接口绕过。
     * 影子计算不输出动作，因此不限制原有真实舵机手动控制。 */
    return Arm_ResetRequired();
#else
    return false;
#endif
}
static uint8_t valid_slots(void) { return (uint8_t)((1U<<SERVO_COUNT)-1U); }
/* 由调用者持有短临界区，返回本控制源尚未完成的停止/手动暂停事务。 */
static uint8_t pending_stops(ServoControlOwner owner)
{
    uint8_t mask=control_stops[owner];
    for(unsigned i=0;i<queued;++i)
        if(is_halt(queue[i].op) && (control_manual_hold[owner]&(1U<<queue[i].slot)))mask|=(uint8_t)(1U<<queue[i].slot);
    if(phase!=IDLE && is_halt(current.op) && (control_manual_hold[owner]&(1U<<current.slot)))mask|=(uint8_t)(1U<<current.slot);
    return mask;
}
static bool motion_pending(ServoControlOwner owner)
{
    for(unsigned i=0;i<queued;++i)if(queue[i].owner==owner && is_motion(queue[i].op))return true;
    return phase!=IDLE && current.owner==owner && is_motion(current.op);
}
bool Servo_ControlClaim(ServoControlOwner owner,uint8_t slots)
{
    if(!owner_valid(owner) || (slots&~valid_slots()))return false;
    if(!slots)return Servo_ControlRelease(owner);
    if(!SERVO_BUS_ENABLE)return false;
    taskENTER_CRITICAL();
    bool ready=published.initialized && !mode_work.ticket &&
        (control_owner==SERVO_OWNER_NONE || control_owner==owner);
    /* ARM 首次接管必须等所有旧动作/停止结束，查询事务可以自然完成。 */
    if(ready && control_owner==SERVO_OWNER_NONE && owner==SERVO_OWNER_ARM) {
        ready=!control_stops[SERVO_OWNER_RC] && !control_stops[SERVO_OWNER_ARM];
        for(unsigned i=0;i<queued && ready;++i)if(is_motion(queue[i].op)||is_halt(queue[i].op))ready=false;
        if(phase!=IDLE && (is_motion(current.op)||is_halt(current.op)))ready=false;
    }
    /* ARM 的关节集合在一次接管期间固定，防止遗留队列变成无主动作。 */
    if(ready && control_owner==SERVO_OWNER_ARM && slots!=control_claimed)ready=false;
    if(ready) { control_owner=owner;control_claimed=slots; }
    taskEXIT_CRITICAL();return ready;
}
bool Servo_ControlRelease(ServoControlOwner owner)
{
    if(!owner_valid(owner))return false;
    taskENTER_CRITICAL();
    bool released=control_owner==SERVO_OWNER_NONE && !motion_pending(owner) && !pending_stops(owner);
    if(control_owner==owner && !motion_pending(owner) && !pending_stops(owner)) {
        control_owner=SERVO_OWNER_NONE;control_claimed=0;released=true;
    }
    taskEXIT_CRITICAL();return released;
}
void Servo_ControlStopMask(ServoControlOwner owner,uint8_t slots)
{
    if(!owner_valid(owner))return;
    taskENTER_CRITICAL();
    if(control_owner==SERVO_OWNER_NONE || control_owner==owner) {
        slots&=valid_slots();
        for(unsigned i=0;i<SERVO_COUNT;++i)if(slots&(1U<<i))cancel_moves(i);
        /* 手动暂停/释力的轴由其手动命令终止，不在其后追加可能改变保持行为的 PDST。 */
        control_stops[owner]|=slots&(uint8_t)~(control_manual_hold[owner]|control_stop_failed[owner]);
    }
    taskEXIT_CRITICAL();
}
uint8_t Servo_ControlStopPending(ServoControlOwner owner)
{
    if(!owner_valid(owner))return 0;
    taskENTER_CRITICAL();uint8_t v=pending_stops(owner);taskEXIT_CRITICAL();return v;
}
uint8_t Servo_ControlInhibited(ServoControlOwner owner)
{
    if(!owner_valid(owner))return 0;
    taskENTER_CRITICAL();uint8_t v=control_inhibited[owner];taskEXIT_CRITICAL();return v;
}
void Servo_ControlClearInhibit(ServoControlOwner owner)
{
    if(!owner_valid(owner))return;
    taskENTER_CRITICAL();
    if((control_owner==SERVO_OWNER_NONE || control_owner==owner) &&
        (owner==SERVO_OWNER_RC || (!pending_stops(owner) && !motion_pending(owner)))) {
        control_inhibited[owner]=0;control_manual_hold[owner]=0;control_stop_failed[owner]=0;
    }
    taskEXIT_CRITICAL();
}
/* ARM 任一受控关节异常时，由驱动立即撤销整组，不能等待应用层下一次 20 ms 更新。 */
static void arm_failure(unsigned failed_slot)
{
    if(control_owner!=SERVO_OWNER_ARM || !(control_claimed&(1U<<failed_slot)))return;
    control_inhibited[SERVO_OWNER_ARM]|=(uint8_t)(1U<<failed_slot);
    for(unsigned i=0;i<SERVO_COUNT;++i)if(control_claimed&(1U<<i))cancel_moves(i);
    control_stops[SERVO_OWNER_ARM]|=control_claimed&
        (uint8_t)~(control_manual_hold[SERVO_OWNER_ARM]|control_stop_failed[SERVO_OWNER_ARM]);
}
static ServoResult validate_motion(ServoControlOwner owner,unsigned slot,uint16_t p,uint16_t ms,uint32_t now)
{
    if(motion_requires_reset() || mode_work.ticket)return SERVO_BUSY;
    if(!published.initialized)return SERVO_NOT_READY;
    if(p<configs[slot].minimum || p>configs[slot].maximum || ms>9999U)return SERVO_RANGE;
    if(owner==SERVO_OWNER_NONE) {
        if(control_owner!=SERVO_OWNER_NONE)return SERVO_BUSY;
    } else if(!owner_valid(owner) || control_owner!=owner || !(control_claimed&(1U<<slot)) ||
        ((control_stops[owner]|control_inhibited[owner])&(1U<<slot)) ||
        (owner==SERVO_OWNER_ARM && control_inhibited[owner]))return SERVO_BUSY;
    const ServoState *s=&published.device[slot];
    if(!s->online || (uint32_t)(now-s->position_ms)>=SERVO_POSITION_STALE_MS)return SERVO_OFFLINE;
    if(!s->mode_valid || s->mode<1U || s->mode>4U)return SERVO_BAD_MODE;
    return SERVO_QUEUED;
}
static int replace_index(unsigned slot,ServoControlOwner owner)
{
    for(unsigned i=0;i<queued;++i)if(queue[i].slot==slot && queue[i].op==ZP_MOVE && queue[i].owner==owner)return (int)i;
    return -1;
}
static uint32_t next_ticket(void) { if(++sequence==0U)++sequence;return sequence; }
static ServoResult enqueue(uint8_t id,ZpOperation op,uint16_t p,uint16_t ms,uint32_t *ticket,ServoControlOwner owner)
{
    if(ticket)*ticket=0;
    if(!SERVO_BUS_ENABLE)return SERVO_DISABLED;
    int slot=slot_for(id);if(slot<0)return SERVO_INVALID;
    taskENTER_CRITICAL();
    ServoState *s=&published.device[slot];
    ServoResult result=SERVO_QUEUED;
    if(is_motion(op) && (motion_requires_reset() || mode_work.ticket))result=SERVO_BUSY;
    else if(op==ZP_MOVE)result=validate_motion(owner,(unsigned)slot,p,ms,HAL_GetTick());
    else if(!published.initialized)result=SERVO_NOT_READY;
    else if(is_motion(op) && control_owner!=SERVO_OWNER_NONE)result=SERVO_BUSY;
    else if(is_motion(op) && (!s->online || (uint32_t)(HAL_GetTick()-s->position_ms)>=SERVO_POSITION_STALE_MS))result=SERVO_OFFLINE;
    if(result!=SERVO_QUEUED) { taskEXIT_CRITICAL();return result; }
    if(is_halt(op)) {
        cancel_moves((unsigned)slot);
        if(control_owner==SERVO_OWNER_ARM)arm_failure((unsigned)slot);
    }
    int replace=op==ZP_MOVE ? replace_index((unsigned)slot,owner) : -1;
    if(replace<0 && queued==SERVO_QUEUE_SIZE) { taskEXIT_CRITICAL();return SERVO_BUSY; }
    if(owner==SERVO_OWNER_NONE && control_owner!=SERVO_OWNER_NONE && is_halt(op)) {
        ServoControlOwner automatic=control_owner;
        control_inhibited[automatic]|=(uint8_t)(1U<<slot);
        control_manual_hold[automatic]|=(uint8_t)(1U<<slot);
        control_stops[automatic]&=(uint8_t)~(1U<<slot);
    }
    Request r={(uint8_t)slot,op,p,ms,next_ticket(),owner};
    if(replace>=0) { queue[replace]=r;++work.coalesced; }
    else queue[queued++]=r;
    if(ticket)*ticket=r.ticket;
    taskEXIT_CRITICAL();
    if(thread)(void)osThreadFlagsSet(thread,SERVO_TASK_FLAG);
    return SERVO_QUEUED;
}
ServoResult Servo_ControlMoveGroup(ServoControlOwner owner,const uint8_t *ids,
    const uint16_t *positions,const uint16_t *times_ms,unsigned count)
{
    if(!SERVO_BUS_ENABLE)return SERVO_DISABLED;
    if(!owner_valid(owner) || !ids || !positions || !times_ms || !count || count>SERVO_COUNT)return SERVO_INVALID;
    uint8_t slots[SERVO_COUNT],seen=0;int replacements[SERVO_COUNT];unsigned needed=0;
    taskENTER_CRITICAL();uint32_t now=HAL_GetTick();
    ServoResult result=SERVO_QUEUED;
    for(unsigned i=0;i<count;++i) {
        int slot=slot_for(ids[i]);
        if(slot<0 || (seen&(1U<<slot))) { result=SERVO_INVALID;break; }
        slots[i]=(uint8_t)slot;seen|=(uint8_t)(1U<<slot);
        result=validate_motion(owner,(unsigned)slot,positions[i],times_ms[i],now);
        if(result!=SERVO_QUEUED)break;
        replacements[i]=replace_index((unsigned)slot,owner);
        if(replacements[i]<0)++needed;
    }
    if(result==SERVO_QUEUED && queued+needed>SERVO_QUEUE_SIZE)result=SERVO_BUSY;
    /* 在同一短临界区完成全量校验和提交，任何失败均不写入部分关节。 */
    if(result==SERVO_QUEUED)for(unsigned i=0;i<count;++i) {
        Request r={slots[i],ZP_MOVE,positions[i],times_ms[i],next_ticket(),owner};
        if(replacements[i]>=0) { queue[replacements[i]]=r;++work.coalesced; }
        else queue[queued++]=r;
    }
    taskEXIT_CRITICAL();
    if(result==SERVO_QUEUED && thread)(void)osThreadFlagsSet(thread,SERVO_TASK_FLAG);
    return result;
}
/* 兼容现有 CH9 控制接口，其状态位仍仅属于 RC 控制源。 */
void Servo_RcClaim(uint8_t slots) { (void)Servo_ControlClaim(SERVO_OWNER_RC,slots); }
#if ROBOT_CONTROL_ENABLE
void Servo_RobotCancelQueuedGrip(void)
{
    taskENTER_CRITICAL();
    for(unsigned i=0;i<SERVO_COUNT;++i)if(configs[i].id==5U)cancel_moves(i);
    taskEXIT_CRITICAL();
}
#endif
void Servo_RcStopMask(uint8_t slots) { Servo_ControlStopMask(SERVO_OWNER_RC,slots); }
uint8_t Servo_RcStopPending(void) { return Servo_ControlStopPending(SERVO_OWNER_RC); }
uint8_t Servo_RcInhibited(void) { return Servo_ControlInhibited(SERVO_OWNER_RC); }
void Servo_RcClearInhibit(void) { Servo_ControlClearInhibit(SERVO_OWNER_RC); }
ServoResult Servo_MovePulse(uint8_t id,uint16_t p,uint16_t ms,uint32_t *t) { return enqueue(id,ZP_MOVE,p,ms,t,SERVO_OWNER_NONE); }
ServoResult Servo_RequestPosition(uint8_t id,uint32_t *t) { return enqueue(id,ZP_POSITION,0,0,t,SERVO_OWNER_NONE); }
static ServoResult request_modes(uint8_t position_mode, int probe_slot, uint32_t *ticket)
{
    if(ticket)*ticket=0U;
    if(!SERVO_BUS_ENABLE)return SERVO_DISABLED;
    if(!ticket)return SERVO_INVALID;
    taskENTER_CRITICAL();
    ServoResult result=SERVO_QUEUED;
    if(!published.initialized)result=SERVO_NOT_READY;
    else if(mode_work.ticket)result=SERVO_BUSY;
    else {
        if(control_owner!=SERVO_OWNER_NONE || control_stops[SERVO_OWNER_RC] ||
            control_stops[SERVO_OWNER_ARM] ||
            (phase!=IDLE && (is_motion(current.op) || is_halt(current.op)))) result=SERVO_BUSY;
        for(unsigned i=0;i<queued && result==SERVO_QUEUED;++i)
            if(is_motion(queue[i].op) || is_halt(queue[i].op))result=SERVO_BUSY;
    }
    if(result==SERVO_QUEUED) {
        mode_slot=probe_slot<0 ? 0U : (uint8_t)probe_slot;
        mode_end_slot=probe_slot<0 ? SERVO_COUNT : (uint8_t)(mode_slot+1U);
        mode_work=(ServoModeReport){.ticket=next_ticket(),.requested_mode=position_mode,
                                    .probe=probe_slot>=0,.probe_slot=mode_slot};
        mode_attempt=0U;
        mode_due=HAL_GetTick();
        mode_deadline=mode_due+MODE_BATCH_TIMEOUT_MS;
        mode_verify=position_mode==0U;
        mode_yield_poll=false;
        *ticket=mode_work.ticket;
    }
    taskEXIT_CRITICAL();
    if(result==SERVO_QUEUED && thread)(void)osThreadFlagsSet(thread,SERVO_TASK_FLAG);
    return result;
}
ServoResult Servo_RequestModes(uint8_t position_mode, uint32_t *ticket)
{
    if(position_mode<1U || position_mode>4U)return SERVO_INVALID;
    return request_modes(position_mode,-1,ticket);
}
ServoResult Servo_RequestModeProbe(uint8_t id, uint32_t *ticket)
{
    int slot=slot_for(id);
    if(slot<0)return SERVO_INVALID;
    return request_modes(0U,slot,ticket);
}
bool Servo_CopyModeReport(uint32_t ticket, ServoModeReport *out)
{
    if(!ticket || !out)return false;
    taskENTER_CRITICAL();
    bool ready=mode_done.ticket==ticket;
    if(ready)*out=mode_done;
    taskEXIT_CRITICAL();
    return ready;
}
ServoResult Servo_Stop(uint8_t id,uint32_t *t) { return enqueue(id,ZP_STOP,0,0,t,SERVO_OWNER_NONE); }
ServoResult Servo_SetTorque(uint8_t id,bool on,uint32_t *t) { return enqueue(id,on?ZP_TORQUE_ON:ZP_TORQUE_OFF,0,0,t,SERVO_OWNER_NONE); }
ServoResult Servo_Pause(uint8_t id,uint32_t *t) { return enqueue(id,ZP_PAUSE,0,0,t,SERVO_OWNER_NONE); }
ServoResult Servo_Resume(uint8_t id,uint32_t *t) { return enqueue(id,ZP_RESUME,0,0,t,SERVO_OWNER_NONE); }
ServoResult Servo_RcMovePulse(uint8_t id,uint16_t p,uint16_t ms,uint32_t *t) { return enqueue(id,ZP_MOVE,p,ms,t,SERVO_OWNER_RC); }

bool Servo_Init(void)
{
    thread=osThreadGetId();
    memset(&work,0,sizeof(work)); work.enabled=SERVO_BUS_ENABLE!=0;
    uint32_t now=HAL_GetTick();
    taskENTER_CRITICAL();queued=0;sequence=0;control_owner=SERVO_OWNER_NONE;control_claimed=0;
    memset(control_stops,0,sizeof(control_stops));memset(control_inhibited,0,sizeof(control_inhibited));
    memset(control_manual_hold,0,sizeof(control_manual_hold));memset(control_stop_failed,0,sizeof(control_stop_failed));taskEXIT_CRITICAL();
    memset(&mode_work,0,sizeof(mode_work));memset(&mode_done,0,sizeof(mode_done));
    mode_current=false;
    ServoRc_Init(now);
    Arm_Init(now);
    phase=IDLE; line_length=0; poll_slot=0; next_slot=guard=now;
    for(unsigned i=0;i<SERVO_COUNT;++i) {
        work.device[i].id=configs[i].id;
        next_poll[i]=next_move[i]=now; boot_phase[i]=0;
        if(configs[i].id==255U || configs[i].minimum<500U || configs[i].maximum>2500U || configs[i].minimum>configs[i].maximum) {
            publish(now); return false;
        }
        for(unsigned j=0;j<i;++j)if(configs[i].id==configs[j].id) { publish(now); return false; }
    }
    work.initialized=SERVO_BUS_ENABLE!=0;
    hardware_ready=SERVO_BUS_ENABLE ? ServoUart_Init() : false;
    publish(now); return !SERVO_BUS_ENABLE || hardware_ready;
}

static void publish_modes(void)
{
    taskENTER_CRITICAL();
    mode_done=mode_work;
    mode_work.ticket=0;
    mode_work.requested_mode=0;
    taskEXIT_CRITICAL();
}
static void finish_mode(ServoResult result, uint32_t now, const ZpReply *reply)
{
    if(current.op==ZP_SET_MODE && phase==REPLY) {
        /* Verify even without a write ACK, since the device may have saved the mode. */
        mode_verify=true;
        mode_due=now+MODE_SETTLE_MS;
        return;
    }
    if(current.op==ZP_MODE && !mode_work.probe && result!=SERVO_OK && mode_attempt<MODE_READ_ATTEMPTS) {
        mode_due=now+SERVO_POLL_PERIOD_MS;
        return;
    }
    if(current.op==ZP_MODE && result==SERVO_OK && reply) {
        mode_work.mode[mode_slot]=(uint8_t)reply->value;
        if(mode_work.requested_mode && reply->value!=mode_work.requested_mode)result=SERVO_BAD_MODE;
    }
    mode_work.result[mode_slot++]=result;
    mode_attempt=0;
    mode_verify=mode_work.requested_mode==0U;
    mode_due=now;
    if(mode_slot==mode_end_slot)publish_modes();
}

static void finish(ServoResult result,uint32_t now,const ZpReply *reply)
{
    ServoState *s=&work.device[current.slot];
    if(current.ticket) { s->last_ticket=current.ticket; s->last_result=result; }
    if(owner_valid(current.owner) && (result!=SERVO_OK || current.op==ZP_STOP)) {
        taskENTER_CRITICAL();
        if(current.op==ZP_STOP)control_stops[current.owner]&=(uint8_t)~(1U<<current.slot);
        if(result!=SERVO_OK) {
            control_inhibited[current.owner]|=(uint8_t)(1U<<current.slot);
            if(current.op==ZP_STOP)control_stop_failed[current.owner]|=(uint8_t)(1U<<current.slot);
            cancel_moves(current.slot);
            if(current.owner==SERVO_OWNER_RC && current.op==ZP_MOVE)
                control_stops[current.owner]|=(uint8_t)(1U<<current.slot);
        }
        taskEXIT_CRITICAL();
    }
    if(result!=SERVO_OK && !mode_current) {
        taskENTER_CRITICAL();arm_failure(current.slot);taskEXIT_CRITICAL();
    }
    if(result==SERVO_OK) {
        if(current.op==ZP_MOVE)s->target=current.position;
        if(reply) {
            ++s->replies;
            switch(reply->type) {
            case ZP_REPLY_ID: s->id_seen=true; break;
            case ZP_REPLY_VERSION: memcpy(s->version,reply->version,sizeof(s->version)); s->version_valid=true; break;
            case ZP_REPLY_MODE:
                if(current.op==ZP_MODE) { s->mode=(uint8_t)reply->value; s->mode_valid=true; }
                break;
            case ZP_REPLY_POSITION:
                /* Recheck metadata after an offline device reappears. No stale moves are replayed. */
                if(s->position_seen && !s->online)boot_phase[current.slot]=0;
                s->position=reply->value; s->position_ms=now; s->position_seen=true; s->online=true; s->failures=0;
                break;
            default: break;
            }
        }
    } else {
        if(result==SERVO_TIMEOUT)++s->timeouts;
        if(current.op==ZP_POSITION && s->failures<255U)++s->failures;
    }
    if(auto_request && current.op!=ZP_POSITION && boot_phase[current.slot]<3U)++boot_phase[current.slot];
    /* Repeated movement commands must not postpone feedback polling indefinitely. */
    if(!mode_current && current.op>=ZP_POSITION && current.op<=ZP_MODE)
        next_poll[current.slot]=s->failures>=SERVO_FAIL_LIMIT ? now+SERVO_OFFLINE_RETRY_MS : request_started+SERVO_POLL_PERIOD_MS;
    if(mode_current) { finish_mode(result,now,reply); mode_current=false; }
    if(result!=SERVO_OK) {
        hardware_ready=ServoUart_Recover();
        guard=now+SERVO_RECOVERY_QUIET_MS;
    } else guard=now+SERVO_BUS_GAP_MS;
    phase=IDLE; line_length=0;
}

static void input_byte(uint8_t c,uint32_t now)
{
    if(phase==REPLY && mode_current && mode_work.probe) {
        const uint16_t index=mode_work.probe_rx_bytes;
        if(index==0U)mode_work.probe_first_ms=(uint32_t)(now-probe_tx_done_ms);
        if(index<sizeof(mode_work.probe_raw))mode_work.probe_raw[index]=c;
        if(index<UINT16_MAX)++mode_work.probe_rx_bytes;
    }
    if(c=='#')line_length=0;
    else if(!line_length)return;
    if(line_length>=sizeof(line)) { ++work.malformed; line_length=0; return; }
    line[line_length++]=(char)c;
    if(c=='!') {
        ZpReply r;
        if(!Zp_Parse(line,line_length,&r))++work.malformed;
        else if(phase!=REPLY || !Zp_Matches(current.op,configs[current.slot].id,&r))++work.unexpected;
        else finish(SERVO_OK,now,&r);
        line_length=0;
    }
}

static bool take_poll(uint32_t now)
{
    if(!due(now,next_slot))return false;
    for(unsigned n=0;n<SERVO_COUNT;++n) {
        unsigned i=poll_slot; poll_slot=(uint8_t)((poll_slot+1U)%SERVO_COUNT);
        if(!due(now,next_poll[i]))continue;
        next_slot=now+SERVO_POLL_SLOT_MS;
        const ZpOperation ops[]={ZP_ID,ZP_VERSION,ZP_MODE,ZP_POSITION};
        ZpOperation op=ops[boot_phase[i]];
        if(work.device[i].position_seen && (uint32_t)(now-work.device[i].position_ms)>=SERVO_POSITION_STALE_MS/2U)
            op=ZP_POSITION;
        current=(Request){.slot=(uint8_t)i,.op=op}; auto_request=true; return true;
    }
    return false;
}
static bool take_mode_request(uint32_t now)
{
    if(!mode_work.ticket || !due(now,mode_due))return false;
    current=(Request){.slot=mode_slot,.op=mode_verify ? ZP_MODE : ZP_SET_MODE,
                      .position=mode_work.requested_mode};
    if(mode_verify)++mode_attempt;
    else {
        /* Changing the mode changes the interpretation of position P. */
        work.device[mode_slot].mode_valid=false;
        work.device[mode_slot].mode=0U;
    }
    auto_request=false;
    mode_current=mode_yield_poll=true;
    return true;
}
static bool take_request(uint32_t now)
{
    taskENTER_CRITICAL();
    int take=-1;
    for(unsigned i=0;i<queued;++i)
        if(queue[i].op==ZP_STOP || queue[i].op==ZP_PAUSE || queue[i].op==ZP_TORQUE_OFF) { take=(int)i; break; }
    if(take>=0) { current=queue[take];remove_at((unsigned)take);auto_request=false;taskEXIT_CRITICAL();return true; }
    for(unsigned owner=SERVO_OWNER_RC;owner<=SERVO_OWNER_ARM;++owner)
        for(unsigned i=0;i<SERVO_COUNT;++i)if(control_stops[owner]&(1U<<i)) {
            current=(Request){.slot=(uint8_t)i,.op=ZP_STOP,.owner=(ServoControlOwner)owner};auto_request=false;
            taskEXIT_CRITICAL();return true;
        }
    taskEXIT_CRITICAL();
    /* Reserve due query slots before ordinary motion: continuous joystick commands cannot starve feedback. */
    if(mode_yield_poll && take_poll(now)) { mode_yield_poll=false; return true; }
    if(take_mode_request(now))return true;
    if(take_poll(now)) { mode_yield_poll=false; return true; }
    taskENTER_CRITICAL();
    for(unsigned i=0;i<queued;++i)if(queue[i].op!=ZP_MOVE || due(now,next_move[queue[i].slot])) { take=(int)i;break; }
    if(take>=0) { current=queue[take];remove_at((unsigned)take);auto_request=false; }
    taskEXIT_CRITICAL();
    return take>=0;
}

void Servo_Process(uint32_t now)
{
    Arm_Process(now); /* 速度IK独立接收应用层指令；默认关闭。 */
    if(!work.initialized || !SERVO_BUS_ENABLE)return;
#if ROBOT_CONTROL_ENABLE
    RobotServo_Process(now);
#else
    ServoRc_Process(now);
#endif
    for(unsigned i=0;i<SERVO_COUNT;++i) {
        ServoState *s=&work.device[i];
        if(s->online && (s->failures>=SERVO_FAIL_LIMIT || (uint32_t)(now-s->position_ms)>=SERVO_POSITION_STALE_MS)) {
            s->online=false; s->mode_valid=false; boot_phase[i]=3;
            cancel_moves(i);
            taskENTER_CRITICAL();arm_failure(i);taskEXIT_CRITICAL();
        }
    }
    uint32_t events=ServoUart_TakeEvents();
    if(events&SERVO_UART_ERROR) {
        if(phase!=IDLE)finish(SERVO_UART_ERROR_RESULT,now,NULL);
        else {
            taskENTER_CRITICAL();
            if(control_owner==SERVO_OWNER_ARM)for(unsigned i=0;i<SERVO_COUNT;++i)
                if(control_claimed&(1U<<i)) { arm_failure(i);break; }
            taskEXIT_CRITICAL();
            hardware_ready=ServoUart_Recover();guard=now+SERVO_RECOVERY_QUIET_MS;line_length=0;
        }
    }
    if(phase==TX && (events&SERVO_UART_TX_DONE)) {
        if(!Zp_ExpectsReply(current.op))finish(SERVO_OK,now,NULL);
        else {
            ServoUartStatus p;
            ServoUart_CopyStatus(&p);
            phase=REPLY;
            uint32_t timeout=SERVO_REPLY_TIMEOUT_MS;
            if(current.op==ZP_MODE) {
                timeout=SERVO_MODE_REPLY_TIMEOUT_MS;
                if(mode_current && mode_work.probe) {
                    probe_tx_done_ms=p.tx_done_ms;
                    timeout=SERVO_MODE_PROBE_TIMEOUT_MS;
                }
            }
            deadline=p.tx_done_ms+timeout;
        }
    }
    uint8_t data[64];
    for(unsigned batch=0;batch<4;++batch) {
        uint16_t n=ServoUart_Read(data,sizeof(data));
        for(unsigned i=0;i<n;++i)input_byte(data[i],now);
        if(n<sizeof(data))break;
    }
    if(phase!=IDLE && due(now,deadline))finish(SERVO_TIMEOUT,now,NULL);
    if(phase==IDLE && mode_work.ticket && due(now,mode_deadline)) {
        /* A UART that never recovers must still produce a finite status report. */
        while(mode_slot<mode_end_slot)mode_work.result[mode_slot++]=SERVO_TIMEOUT;
        publish_modes();
    }
    if(phase==IDLE && due(now,guard)) {
        ServoUartStatus p; ServoUart_CopyStatus(&p);
        if(!hardware_ready || !p.ready) {
            hardware_ready=ServoUart_Recover(); guard=now+SERVO_RECOVERY_QUIET_MS;
        } else if((uint32_t)(now-p.last_rx_ms)>=SERVO_BUS_GAP_MS && take_request(now)) {
            request_started=now;
            ServoState *s=&work.device[current.slot];
            bool motion=current.op==ZP_MOVE || current.op==ZP_RESUME || current.op==ZP_TORQUE_ON;
            if(motion && motion_requires_reset())finish(SERVO_BUSY,now,NULL);
            else if(owner_valid(current.owner) && current.op==ZP_MOVE &&
                (control_owner!=current.owner || !(control_claimed&(1U<<current.slot)) ||
                 ((control_inhibited[current.owner]|control_stops[current.owner])&(1U<<current.slot)) ||
                 (current.owner==SERVO_OWNER_ARM && control_inhibited[current.owner])))finish(SERVO_BUSY,now,NULL);
            else if(motion && (!s->online || (uint32_t)(now-s->position_ms)>=SERVO_POSITION_STALE_MS))finish(SERVO_OFFLINE,now,NULL);
            else if(current.op==ZP_MOVE && (!s->mode_valid || s->mode<1U || s->mode>4U))finish(SERVO_BAD_MODE,now,NULL);
            else {
                char frame[ZP_FRAME_MAX];
                size_t n=Zp_Build(frame,sizeof(frame),current.op,s->id,current.position,current.time_ms);
                line_length=0;
#if ROBOT_CONTROL_ENABLE
                taskENTER_CRITICAL();
                bool obsolete = current.op==ZP_MOVE && current.owner==SERVO_OWNER_RC &&
                    !RobotServo_CanTransmit(s->id, HAL_GetTick());
                bool sent = !obsolete && n && ServoUart_Send(frame,(uint16_t)n);
                taskEXIT_CRITICAL();
                /* 切档撤销排队动作是正常操作，不能触发驱动错误路径的 PDST。 */
                if(obsolete) { publish(now); return; }
#else
                bool sent = n && ServoUart_Send(frame,(uint16_t)n);
#endif
                if(!sent)finish(SERVO_TX_ERROR,now,NULL);
                else {
                    phase=TX; deadline=now+SERVO_TX_TIMEOUT_MS;
                    if(current.op==ZP_MOVE)next_move[current.slot]=now+SERVO_MOVE_INTERVAL_MS;
                    if(current.ticket) { s->last_ticket=current.ticket; s->last_result=SERVO_QUEUED; }
                }
            }
        }
    }
    publish(now);
}

void Campus_ServoTask(void *argument)
{
    (void)argument; (void)Servo_Init();
    for(;;) {
        Servo_Process(HAL_GetTick());
        (void)osThreadFlagsWait(SERVO_TASK_FLAG,osFlagsWaitAny, SERVO_BUS_ENABLE ? 1U : 1000U);
    }
}
