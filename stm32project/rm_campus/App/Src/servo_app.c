#include "servo_app.h"
#include "robot_servo.h"
#include "servo_rc.h"
#include "arm_velocity.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include <string.h>
#if SERVO_FASHIONSTAR_GRIPPER
#include "fs_servo.h"
#include "robot_control.h"
/* 实测众灵 ASCII 帧会使此 U25-M 漏掉后续数条请求。只用定向读角度重新同步。 */
enum { FS_QUERY_REPLY_MS = 5, FS_QUERY_ATTEMPTS = 16, FS_BUS_GAP_MS = 6 };
static bool fs_aligned, fs_repeat;
static unsigned fs_attempts;
#endif
#if SERVO_JOG_TEST_ENABLE
#include "usb_serial.h"
#include <math.h>
#include <stdlib.h>
#endif

static const ServoConfig configs[SERVO_COUNT] = {SERVO_DEVICE_TABLE};
typedef struct { uint8_t slot; ZpOperation op; uint16_t position, time_ms; uint32_t ticket; ServoControlOwner owner; } Request;
typedef enum { IDLE, TX, REPLY } Phase;
static Request queue[SERVO_QUEUE_SIZE], current;
static uint16_t queued;
static uint32_t sequence;
static ServoSnapshot work, published;
static uint32_t next_poll[SERVO_COUNT], next_move[SERVO_COUNT], next_slot, deadline, guard, request_started;
static uint8_t boot_phase[SERVO_COUNT], poll_slot;
static uint32_t last_metadata_retry;
static unsigned metadata_cursor;
static bool metadata_retry_current, refresh_after_retry;
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
static bool bus_slot_active(unsigned slot)
{
    /* 保留示波器槽位；正式遥控的腕俯仰 001 不参与任何 UART7 事务。 */
    return !ROBOT_CONTROL_ENABLE || configs[slot].id != 1U;
}
static bool fashionstar_slot(unsigned slot)
{
    return SERVO_FASHIONSTAR_GRIPPER && configs[slot].id == 5U;
}
static int slot_for(uint8_t id)
{
    for(unsigned i=0;i<SERVO_COUNT;++i)if(configs[i].id==id)return (int)i;
    return -1;
}
const ServoConfig *Servo_GetConfig(unsigned i) { return i<SERVO_COUNT ? &configs[i] : NULL; }
#if SERVO_JOG_TEST_ENABLE
typedef enum { JOG_IDLE, JOG_CHECK_LINK, JOG_MODE, JOG_BASE, JOG_MOVE, JOG_VERIFY, JOG_COMPLETE } JogPhase;
static struct {
    JogPhase phase;
    uint8_t slot, direction, mode;
    uint32_t session, expires, verify_after;
    ServoJogReport report;
} jog;
/* 已取出的 UART 事务即使跨 USB 断线结束，也不能推进新会话的点动。 */
static uint32_t jog_generation, wire_generation;

ServoResult ServoJog_Start(uint8_t id,uint8_t direction,uint32_t ticket,uint32_t session)
{
    int slot=slot_for(id);
    if(slot<0 || !ticket || (direction!=1U && direction!=2U))return SERVO_INVALID;
    taskENTER_CRITICAL();
    const ServoState *s=&published.device[slot];
    uint32_t now=HAL_GetTick();
    ServoResult result=SERVO_QUEUED;
    if(jog.phase!=JOG_IDLE)result=SERVO_BUSY;
    else if(!published.initialized || !s->position_seen)result=SERVO_OFFLINE;
    else {
        if(++jog_generation==0U)++jog_generation;
        jog.slot=(uint8_t)slot;
        jog.direction=direction;
        jog.session=session;
        jog.mode=s->mode_valid?s->mode:0;
        jog.report=(ServoJogReport){.id=id,.ticket=ticket};
        if(!s->online || now-s->position_ms>=SERVO_POSITION_STALE_MS)jog.phase=JOG_CHECK_LINK;
        else jog.phase=jog.mode ? JOG_BASE : JOG_MODE;
        jog.expires=now+SERVO_MODE_PROBE_TIMEOUT_MS+SERVO_JOG_SETTLE_TIMEOUT_MS;
    }
    taskEXIT_CRITICAL();
    return result;
}

bool ServoJog_TakeResult(uint32_t ticket,ServoJogReport *out)
{
    taskENTER_CRITICAL();
    bool done=out && jog.phase==JOG_COMPLETE && ticket==jog.report.ticket;
    if(done) { *out=jog.report; jog.phase=JOG_IDLE; }
    taskEXIT_CRITICAL();
    return done;
}

void ServoJog_Cancel(void)
{
    taskENTER_CRITICAL();
    jog.phase=JOG_IDLE;
    taskEXIT_CRITICAL();
}

/* 以下逻辑仅在 ServoBusTask 中运行；USB 任务通过上面三个短临界区交接。 */
static void jog_done(const char *result)
{
    jog.report.result=result;
    jog.phase=JOG_COMPLETE;
}

static bool jog_session_ready(void)
{
    UsbSerialStatus usb;
    UsbSerial_GetStatus(&usb);
    return usb.ready && usb.session_id==jog.session;
}

static void jog_tick(uint32_t now)
{
    if(jog.phase==JOG_IDLE || jog.phase==JOG_COMPLETE)return;
    if(!jog_session_ready()) { jog_done("DISCONNECTED"); return; }
    if(due(now,jog.expires)) {
        const char *reason="TIMEOUT";
        if(jog.phase==JOG_MODE)reason="MODE_UNKNOWN";
        if(jog.phase==JOG_VERIFY && jog.report.actual)reason="NOT_REACHED";
        jog_done(reason);
    }
}

static bool take_jog_request(void)
{
    wire_generation=0;
    ZpOperation op;
    switch(jog.phase) {
    case JOG_CHECK_LINK: op=ZP_POSITION; break;
    case JOG_MODE: op=ZP_MODE; break;
    case JOG_BASE:
        if(jog.mode<1U || jog.mode>4U) { jog_done("BAD_MODE"); return false; }
        op=ZP_POSITION;
        break;
    case JOG_MOVE: op=ZP_MOVE; break;
    default: return false;
    }
    current=(Request){.slot=jog.slot,.op=op,.position=jog.report.target,.time_ms=SERVO_JOG_MOVE_MS,
                      .ticket=op==ZP_MOVE?jog.report.ticket:0U};
    wire_generation=jog_generation;
    auto_request=false;
    return true;
}

static void finish_jog_request(ServoResult result,uint32_t now,const ZpReply *reply)
{
    if(jog.phase==JOG_IDLE || jog.phase==JOG_COMPLETE || current.slot!=jog.slot)return;
    const unsigned span=jog.mode<=2U?270U:180U;
    /* 只有运动帧发送完成、计划运动时间届满后新发起的位置查询才可确认到位。 */
    if(jog.phase==JOG_VERIFY && auto_request && current.op==ZP_POSITION &&
       result==SERVO_OK && reply && due(request_started,jog.verify_after)) {
        jog.report.actual=reply->value;
        unsigned error=(unsigned)abs((int)jog.report.actual-(int)jog.report.target);
        unsigned step=(unsigned)abs((int)jog.report.start-(int)jog.report.target);
        if(error<step && error*span<=2000.0f*SERVO_JOG_TOLERANCE_DEG)
            jog_done("DONE");
        return;
    }
    if(!wire_generation || wire_generation!=jog_generation)return;
    if(result!=SERVO_OK) {
        const char *why=Servo_ResultName(result);
        if(result==SERVO_TIMEOUT && phase==TX)why="TX_TIMEOUT";
        else if(result==SERVO_TIMEOUT && jog.phase==JOG_MODE)why="MODE_UNKNOWN";
        else if(result==SERVO_TIMEOUT && jog.phase==JOG_CHECK_LINK)why="OFFLINE";
        jog_done(why);
        return;
    }
    switch(jog.phase) {
    case JOG_CHECK_LINK:
        /* 单线轮询/长模式应答可能使缓存过期，收到位置后重新确认模式。 */
        jog.mode=work.device[jog.slot].mode_valid?work.device[jog.slot].mode:0U;
        jog.phase=jog.mode ? JOG_BASE : JOG_MODE;
        break;
    case JOG_MODE:
        jog.mode=(uint8_t)reply->value;
        jog.phase=JOG_BASE;
        break;
    case JOG_BASE: {
        jog.report.start=reply->value;
        const float degrees=(float)SERVO_JOG_STEP_DEG;
        if(!isfinite(degrees) || degrees<=0.0f || degrees>(float)span) { jog_done("BAD_CONFIG"); break; }
        int step=(int)lroundf(degrees*2000.0f/(float)span);
        if(step==0) { jog_done("BAD_CONFIG"); break; }
        bool increase=(jog.direction==1U)==((jog.mode&1U)!=0U);
        int target=(int)jog.report.start+(increase?step:-step);
        const ServoConfig *c=&configs[jog.slot];
        if(jog.report.start<c->minimum || jog.report.start>c->maximum ||
           target<c->minimum || target>c->maximum) { jog_done("LIMIT"); break; }
        jog.report.target=(uint16_t)target;
        jog.phase=JOG_MOVE;
        break;
    }
    case JOG_MOVE:
        jog.verify_after=now+SERVO_JOG_MOVE_MS;
        jog.expires=jog.verify_after+SERVO_JOG_SETTLE_TIMEOUT_MS;
        jog.phase=JOG_VERIFY;
        break;
    default: break;
    }
}
#endif
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
static uint8_t valid_slots(void)
{
    uint8_t mask=0;
    for(unsigned i=0;i<SERVO_COUNT;++i)if(bus_slot_active(i))mask|=(uint8_t)(1U<<i);
    return mask;
}
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
    if(SERVO_STATUS_ONLY)return false;
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
    if(SERVO_STATUS_ONLY || !owner_valid(owner))return;
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
static bool position_mode_required(ServoControlOwner owner)
{
    return !ROBOT_CONTROL_ENABLE || owner != SERVO_OWNER_RC;
}
static bool motion_position_ready(const ServoState *s, ServoControlOwner owner, uint32_t now)
{
    /* 整机遥控使用最后一次有效位置；调试和 IK 仍要求在线的实时位置。 */
    if(ROBOT_CONTROL_ENABLE && owner==SERVO_OWNER_RC)return s->position_seen;
    return s->online && (uint32_t)(now-s->position_ms)<SERVO_POSITION_STALE_MS;
}
static ServoResult validate_motion(ServoControlOwner owner,unsigned slot,uint16_t p,uint16_t ms,uint32_t now)
{
    if(!bus_slot_active(slot))return SERVO_DISABLED;
    if(motion_requires_reset() || mode_work.ticket)return SERVO_BUSY;
    if(!published.initialized)return SERVO_NOT_READY;
    if(p<configs[slot].minimum || p>configs[slot].maximum || ms>9999U)return SERVO_RANGE;
    if(owner==SERVO_OWNER_NONE) {
        if(control_owner!=SERVO_OWNER_NONE)return SERVO_BUSY;
    } else if(!owner_valid(owner) || control_owner!=owner || !(control_claimed&(1U<<slot)) ||
        ((control_stops[owner]|control_inhibited[owner])&(1U<<slot)) ||
        (owner==SERVO_OWNER_ARM && control_inhibited[owner]))return SERVO_BUSY;
    const ServoState *s=&published.device[slot];
    if(!motion_position_ready(s,owner,now))return SERVO_OFFLINE;
    if(position_mode_required(owner) && (!s->mode_valid || s->mode<1U || s->mode>4U))return SERVO_BAD_MODE;
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
    if(SERVO_STATUS_ONLY || !SERVO_BUS_ENABLE)return SERVO_DISABLED;
    int slot=slot_for(id);if(slot<0)return SERVO_INVALID;
    if(!bus_slot_active((unsigned)slot))return SERVO_DISABLED;
    taskENTER_CRITICAL();
    ServoState *s=&published.device[slot];
    ServoResult result=SERVO_QUEUED;
    if(is_motion(op) && (motion_requires_reset() || mode_work.ticket))result=SERVO_BUSY;
    else if(op==ZP_MOVE)result=validate_motion(owner,(unsigned)slot,p,ms,HAL_GetTick());
    else if(!published.initialized)result=SERVO_NOT_READY;
    else if(is_motion(op) && control_owner!=SERVO_OWNER_NONE)result=SERVO_BUSY;
    else if(is_motion(op) && !motion_position_ready(s,owner,HAL_GetTick()))result=SERVO_OFFLINE;
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
    if(SERVO_STATUS_ONLY || !SERVO_BUS_ENABLE)return SERVO_DISABLED;
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
    if(SERVO_STATUS_ONLY || !SERVO_BUS_ENABLE)return SERVO_DISABLED;
    if(probe_slot>=0 && !bus_slot_active((unsigned)probe_slot))return SERVO_DISABLED;
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
#if SERVO_JOG_TEST_ENABLE
    ServoJog_Cancel();
    wire_generation=0;
#endif
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
#if SERVO_FASHIONSTAR_GRIPPER
    fs_aligned=fs_repeat=false; fs_attempts=0U;
#endif
    last_metadata_retry=now; metadata_cursor=0; metadata_retry_current=false; refresh_after_retry=false;
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
    const bool robot_rc = ROBOT_CONTROL_ENABLE && current.owner == SERVO_OWNER_RC;
    if(current.ticket) { s->last_ticket=current.ticket; s->last_result=result; }
    if(owner_valid(current.owner) && (result!=SERVO_OK || current.op==ZP_STOP)) {
        taskENTER_CRITICAL();
        if(current.op==ZP_STOP)control_stops[current.owner]&=(uint8_t)~(1U<<current.slot);
        if(result!=SERVO_OK) {
            if(!robot_rc) {
                control_inhibited[current.owner]|=(uint8_t)(1U<<current.slot);
                if(current.op==ZP_STOP)control_stop_failed[current.owner]|=(uint8_t)(1U<<current.slot);
            }
            cancel_moves(current.slot);
            if(current.owner==SERVO_OWNER_RC && current.op==ZP_MOVE)
                control_stops[current.owner]|=(uint8_t)(1U<<current.slot);
        }
        taskEXIT_CRITICAL();
        if(robot_rc && result!=SERVO_OK) {
            /* 动作或停止帧自身发送失败时，重新读取位置再接管。 */
            s->online=false;
            s->position_seen=false;
            s->mode_valid=false;
            s->target=0;
            boot_phase[current.slot]=3;
            next_poll[current.slot]=now;
        }
    }
    /* 可选后台查询缺应答只计超时；真正的通信错误和位置故障仍走原控制保护。 */
    bool missing_metadata=metadata_retry_current && phase==REPLY && result==SERVO_TIMEOUT;
    if(result!=SERVO_OK && !mode_current && !missing_metadata) {
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
        if(auto_request && current.op==ZP_VERSION)s->version_valid=false;
    }
    if(auto_request && current.op!=ZP_POSITION && boot_phase[current.slot]<3U)++boot_phase[current.slot];
    /* Repeated movement commands must not postpone feedback polling indefinitely. */
    if(!mode_current && !metadata_retry_current && current.op>=ZP_POSITION && current.op<=ZP_MODE)
        next_poll[current.slot]=s->failures>=SERVO_FAIL_LIMIT ? now+SERVO_OFFLINE_RETRY_MS : request_started+SERVO_POLL_PERIOD_MS;
#if SERVO_JOG_TEST_ENABLE
    finish_jog_request(result,now,reply);
#endif
    if(mode_current) { finish_mode(result,now,reply); mode_current=false; }
    if(result!=SERVO_OK) {
        hardware_ready=ServoUart_Recover();
        guard=now+SERVO_RECOVERY_QUIET_MS;
    } else guard=now+(SERVO_FASHIONSTAR_GRIPPER ? 6U : SERVO_BUS_GAP_MS);
#if SERVO_FASHIONSTAR_GRIPPER
    if (fashionstar_slot(current.slot) && current.op==ZP_POSITION) {
        /* 已经收到过位置时，切换协议后尝试找回回包即可继续遥控。 */
        fs_aligned=result==SERVO_OK || (result==SERVO_TIMEOUT && phase==REPLY && s->position_seen);
        fs_repeat=false;
        fs_attempts=0U;
    }
#endif
    if(metadata_retry_current)refresh_after_retry=true;
    phase=IDLE; line_length=0; metadata_retry_current=false;
}

static void input_byte(uint8_t c,uint32_t now)
{
#if SERVO_FASHIONSTAR_GRIPPER
    if (phase==REPLY && fashionstar_slot(current.slot)) {
        if (line_length==0U && c!=0x05) return;
        if (line_length==1U && c!=0x1C) {
            line_length=c==0x05;
            return;
        }
        line[line_length++]=(char)c;
        if (line_length==FS_SERVO_ANGLE_REPLY_SIZE) {
            ServoState *state=&work.device[current.slot];
            ZpReply position={.type=ZP_REPLY_POSITION, .id=state->id};
            if (FsServo_ParsePosition((const uint8_t *)line,state->id,&position.value)) {
                state->id_seen=true;
                finish(SERVO_OK,now,&position);
            } else {
                ++work.malformed;
                line_length=0U;
            }
        }
        return;
    }
#endif
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

static bool take_metadata_retry(uint32_t now,uint32_t oldest_age)
{
    if(now-last_metadata_retry<SERVO_METADATA_RETRY_MS || mode_work.ticket || control_owner==SERVO_OWNER_ARM)
        return false;
#if SERVO_JOG_TEST_ENABLE
    if(jog.phase!=JOG_IDLE && jog.phase!=JOG_COMPLETE)return false;
#endif
    /* 跨 ID 和字段轮换；一个模式始终不回复时，其他模式和版本仍有机会。 */
    const unsigned fields=2U*SERVO_COUNT;
    for(unsigned n=0;n<fields;++n) {
        unsigned field=(metadata_cursor+n)%fields;
        unsigned slot=field%SERVO_COUNT;
        const ServoState *s=&work.device[slot];
        if(!s->online || boot_phase[slot]!=3U || fashionstar_slot(slot))continue;
        bool mode=field<SERVO_COUNT;
        if(mode?s->mode_valid:s->version_valid)continue;
        /* 单线一次只能等一个应答，先给在线位置预留超时恢复和下一个轮询槽。 */
        uint32_t timeout=mode?SERVO_MODE_REPLY_TIMEOUT_MS:SERVO_REPLY_TIMEOUT_MS;
        if(oldest_age+timeout+SERVO_RECOVERY_QUIET_MS+SERVO_POLL_SLOT_MS>=SERVO_POSITION_STALE_MS)continue;
        current=(Request){.slot=(uint8_t)slot,.op=mode?ZP_MODE:ZP_VERSION};
        auto_request=true; metadata_retry_current=true;
        metadata_cursor=(field+1U)%fields;
        last_metadata_retry=now;
        return true;
    }
    return false;
}

static bool take_poll(uint32_t now,bool positions_only)
{
    if(!due(now,next_slot))return false;
#if FS_GRIPPER_BENCH_ISOLATION
    const RobotMode robot_mode=RobotControl_Get(now).mode;
    if (robot_mode==ROBOT_WAIT || robot_mode==ROBOT_WRIST) {
        const int grip=slot_for(5U);
        if (grip<0 || !due(now,next_poll[grip])) return false;
        current=(Request){.slot=(uint8_t)grip,.op=ZP_POSITION};
        auto_request=true;next_slot=now+SERVO_POLL_SLOT_MS;
        return true;
    }
#endif
    uint32_t oldest_age=0;
    unsigned oldest=SERVO_COUNT;
    for(unsigned i=0;i<SERVO_COUNT;++i) {
        if(FS_GRIPPER_BENCH_ISOLATION && fashionstar_slot(i))continue;
        const ServoState *s=&work.device[i];
        uint32_t age=now-s->position_ms;
        if(s->online && age>oldest_age) { oldest=i; oldest_age=age; }
    }
    /* 混接时先给健康轴位置回读留出一轮最慢的 ZP 查询时间，防止离线轴轮询挤掉夹爪反馈。 */
    const uint32_t refresh_age = SERVO_POSITION_STALE_MS/2U;
    if((refresh_after_retry || SERVO_FASHIONSTAR_GRIPPER) && oldest<SERVO_COUNT && oldest_age>=refresh_age &&
       !work.device[oldest].failures && due(now,next_poll[oldest])) {
        current=(Request){.slot=(uint8_t)oldest,.op=ZP_POSITION};
        auto_request=true; next_slot=now+SERVO_POLL_SLOT_MS;
        return true;
    }
    refresh_after_retry=false;
    if(!positions_only && take_metadata_retry(now,oldest_age))return true;
    for(unsigned n=0;n<SERVO_COUNT;++n) {
        unsigned i=poll_slot; poll_slot=(uint8_t)((poll_slot+1U)%SERVO_COUNT);
        if(!bus_slot_active(i))continue;
        if(!due(now,next_poll[i]))continue;
        if(FS_GRIPPER_BENCH_ISOLATION && fashionstar_slot(i))continue;
        if(positions_only && !work.device[i].online &&
           !(ROBOT_CONTROL_ENABLE && control_owner==SERVO_OWNER_RC &&
             (control_claimed&(1U<<i)) && work.device[i].position_seen))continue;
        next_slot=now+SERVO_POLL_SLOT_MS;
        const ZpOperation ops[]={ZP_ID,ZP_VERSION,ZP_MODE,ZP_POSITION};
        ZpOperation op=ops[boot_phase[i]];
        if(fashionstar_slot(i) || (SERVO_FASHIONSTAR_GRIPPER && !work.device[i].online) || positions_only ||
           (work.device[i].position_seen &&
           (uint32_t)(now-work.device[i].position_ms)>=SERVO_POSITION_STALE_MS/2U))
            op=ZP_POSITION;
        current=(Request){.slot=(uint8_t)i,.op=op}; auto_request=true; return true;
    }
    return false;
}
static bool take_mode_request(uint32_t now)
{
    if(!mode_work.ticket || !due(now,mode_due))return false;
    while(mode_slot<mode_end_slot && !bus_slot_active(mode_slot))
        mode_work.result[mode_slot++]=SERVO_DISABLED;
    if(mode_slot==mode_end_slot) { publish_modes(); return false; }
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
static bool take_request(uint32_t now,bool positions_only)
{
#if SERVO_FASHIONSTAR_GRIPPER
    if (fs_repeat) { fs_repeat=false; return true; }
#endif
#if SERVO_JOG_TEST_ENABLE
    /* 点动的少数显式事务与正常轮询共用 UART；等运动到位期间继续轮询五台。 */
    return take_jog_request() || take_poll(now,false);
#else
    if(SERVO_STATUS_ONLY)return take_poll(now,false);
#endif
    taskENTER_CRITICAL();
    int take=-1;
    for(unsigned i=0;i<queued;++i)
        if(queue[i].op==ZP_STOP || queue[i].op==ZP_PAUSE || queue[i].op==ZP_TORQUE_OFF) { take=(int)i; break; }
    if(take>=0) { current=queue[take];remove_at((unsigned)take);auto_request=false;taskEXIT_CRITICAL();return true; }
    for(unsigned owner=SERVO_OWNER_RC;owner<=SERVO_OWNER_ARM;++owner)
        for(unsigned i=0;i<SERVO_COUNT;++i)if(control_stops[owner]&(1U<<i)) {
#if SERVO_FASHIONSTAR_GRIPPER
            if (fashionstar_slot(i) && !fs_aligned && work.device[i].online && !work.device[i].failures) {
                current=(Request){.slot=(uint8_t)i,.op=ZP_POSITION};auto_request=true;
                taskEXIT_CRITICAL();return true;
            }
#endif
            current=(Request){.slot=(uint8_t)i,.op=ZP_STOP,.owner=(ServoControlOwner)owner};auto_request=false;
            taskEXIT_CRITICAL();return true;
        }
#if SERVO_FASHIONSTAR_GRIPPER
    /* 保持二进制会话：回读有效后先送同轴最新目标，再让众灵请求占用总线。 */
    if (fs_aligned) for (unsigned i=0;i<queued;++i) {
        if (queue[i].op!=ZP_MOVE || !fashionstar_slot(queue[i].slot) || !due(now,next_move[queue[i].slot]))
            continue;
        current=queue[i];remove_at(i);auto_request=false;
        taskEXIT_CRITICAL();return true;
    }
#endif
    taskEXIT_CRITICAL();
    /* 在线位置轮询维持优先级。推杆期间延后可能阻塞总线的离线重试和模式/版本查询。 */
    if(mode_yield_poll && take_poll(now,positions_only)) { mode_yield_poll=false; return true; }
    if(take_mode_request(now))return true;
    if(take_poll(now,positions_only)) { mode_yield_poll=false; return true; }
    taskENTER_CRITICAL();
    for(unsigned i=0;i<queued;++i)if(queue[i].op!=ZP_MOVE || due(now,next_move[queue[i].slot])) { take=(int)i;break; }
    if(take>=0) {
        current=queue[take];
#if SERVO_FASHIONSTAR_GRIPPER
        if (current.op==ZP_MOVE && fashionstar_slot(current.slot) && !fs_aligned) {
            /* 等回包时保留队列中的目标，继续允许摇杆合并或切档撤销。 */
            current=(Request){.slot=current.slot,.op=ZP_POSITION};auto_request=true;
            taskEXIT_CRITICAL();return true;
        }
#endif
        remove_at((unsigned)take);auto_request=false;
    }
    taskEXIT_CRITICAL();
    return take>=0;
}

void Servo_Process(uint32_t now)
{
    Arm_Process(now); /* 速度IK独立接收应用层指令；默认关闭。 */
    if(!work.initialized || !SERVO_BUS_ENABLE)return;
#if SERVO_JOG_TEST_ENABLE
    jog_tick(now);
#endif
#if ROBOT_CONTROL_ENABLE
    const bool positions_only=RobotServo_Process(now);
#else
    const bool positions_only=false;
    ServoRc_Process(now);
#endif
    for(unsigned i=0;i<SERVO_COUNT;++i) {
        ServoState *s=&work.device[i];
        if(s->online && (s->failures>=SERVO_FAIL_LIMIT || (uint32_t)(now-s->position_ms)>=SERVO_POSITION_STALE_MS)) {
            s->online=false; s->mode_valid=false; boot_phase[i]=3;
            if(!ROBOT_CONTROL_ENABLE || control_owner!=SERVO_OWNER_RC || !(control_claimed&(1U<<i)))
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
        const bool expects_reply = fashionstar_slot(current.slot) ? current.op==ZP_POSITION :
                                    Zp_ExpectsReply(current.op);
        if(!expects_reply)finish(SERVO_OK,now,NULL);
        else {
            ServoUartStatus p;
            ServoUart_CopyStatus(&p);
            phase=REPLY;
            uint32_t timeout=SERVO_REPLY_TIMEOUT_MS;
#if SERVO_FASHIONSTAR_GRIPPER
            if (fashionstar_slot(current.slot)) { timeout=FS_QUERY_REPLY_MS; ++fs_attempts; }
#endif
            if(current.op==ZP_MODE) {
                timeout=SERVO_MODE_REPLY_TIMEOUT_MS;
                if(mode_current && mode_work.probe) {
                    probe_tx_done_ms=p.tx_done_ms;
                    timeout=SERVO_MODE_PROBE_TIMEOUT_MS;
                }
#if SERVO_JOG_TEST_ENABLE
                if(wire_generation)timeout=SERVO_MODE_PROBE_TIMEOUT_MS;
#endif
            }
            deadline=p.tx_done_ms+timeout;
        }
    }
    uint8_t data[64];
    /* TC 可能刚好发生在 TakeEvents 和读环之间。在任务确认 TC 前保留整个快回包。 */
    for(unsigned batch=0;phase!=TX && batch<4;++batch) {
        uint16_t n=ServoUart_Read(data,sizeof(data));
        for(unsigned i=0;i<n;++i)input_byte(data[i],now);
        if(n<sizeof(data))break;
    }
    if(phase!=IDLE && due(now,deadline)) {
        if(phase==TX) {
            /* 到期边界再看中断侧状态：已发生的 TC/错误留给下一轮取事件和读环。 */
            ServoUartStatus uart;
            ServoUart_CopyStatus(&uart);
            if(uart.ready && uart.transmitting)finish(SERVO_TIMEOUT,now,NULL);
        } else {
#if SERVO_FASHIONSTAR_GRIPPER
            if (current.op==ZP_POSITION && fashionstar_slot(current.slot) &&
                !fs_aligned && fs_attempts<FS_QUERY_ATTEMPTS) {
                phase=IDLE;line_length=0U;fs_repeat=true;guard=request_started+FS_BUS_GAP_MS;
            } else
#endif
            finish(SERVO_TIMEOUT,now,NULL);
        }
    }
    if(phase==IDLE && mode_work.ticket && due(now,mode_deadline)) {
        /* A UART that never recovers must still produce a finite status report. */
        while(mode_slot<mode_end_slot)mode_work.result[mode_slot++]=SERVO_TIMEOUT;
        publish_modes();
    }
    if(phase==IDLE && due(now,guard)) {
        ServoUartStatus p; ServoUart_CopyStatus(&p);
        if(!hardware_ready || !p.ready) {
            hardware_ready=ServoUart_Recover(); guard=now+SERVO_RECOVERY_QUIET_MS;
        } else if((uint32_t)(now-p.last_rx_ms)>=SERVO_BUS_GAP_MS && take_request(now,positions_only)) {
            request_started=now;
            ServoState *s=&work.device[current.slot];
            bool motion=current.op==ZP_MOVE || current.op==ZP_RESUME || current.op==ZP_TORQUE_ON;
            if(!bus_slot_active(current.slot))finish(SERVO_DISABLED,now,NULL);
            else if(motion && motion_requires_reset())finish(SERVO_BUSY,now,NULL);
            else if(owner_valid(current.owner) && current.op==ZP_MOVE &&
                (control_owner!=current.owner || !(control_claimed&(1U<<current.slot)) ||
                 ((control_inhibited[current.owner]|control_stops[current.owner])&(1U<<current.slot)) ||
                 (current.owner==SERVO_OWNER_ARM && control_inhibited[current.owner])))finish(SERVO_BUSY,now,NULL);
            else if(motion && !motion_position_ready(s,current.owner,now))finish(SERVO_OFFLINE,now,NULL);
            else if(current.op==ZP_MOVE && position_mode_required(current.owner) &&
                    (!s->mode_valid || s->mode<1U || s->mode>4U))finish(SERVO_BAD_MODE,now,NULL);
#if SERVO_JOG_TEST_ENABLE
            else if(current.op==ZP_MOVE && s->mode!=jog.mode)finish(SERVO_BAD_MODE,now,NULL);
#endif
            else {
                char frame[ZP_FRAME_MAX];
                size_t n;
#if SERVO_FASHIONSTAR_GRIPPER
                if (fashionstar_slot(current.slot)) {
                    uint8_t *wire=(uint8_t *)frame;
                    switch (current.op) {
                    case ZP_POSITION: n=FsServo_BuildQuery(wire,sizeof(frame),s->id); break;
                    case ZP_MOVE: n=FsServo_BuildMove(wire,sizeof(frame),s->id,current.position,
                                                      current.time_ms,FS_GRIPPER_POWER_MW); break;
                    case ZP_STOP: n=FsServo_BuildHold(wire,sizeof(frame),s->id,FS_GRIPPER_POWER_MW); break;
                    default: n=0U; break;
                    }
                } else
#endif
                n=Zp_Build(frame,sizeof(frame),current.op,s->id,current.position,current.time_ms);
                line_length=0;
#if ROBOT_CONTROL_ENABLE
                taskENTER_CRITICAL();
                bool obsolete = current.op==ZP_MOVE && current.owner==SERVO_OWNER_RC &&
                    !RobotServo_CanTransmit(s->id, HAL_GetTick());
                bool sent = !obsolete && n && ServoUart_Send(frame,(uint16_t)n);
                taskEXIT_CRITICAL();
                /* 切档撤销排队动作是正常操作，不能触发驱动错误路径的 PDST。 */
                if(obsolete) { publish(now); return; }
#elif SERVO_JOG_TEST_ENABLE
                /* USB 撤销配置/重连后，尚未发出的旧会话运动不能交给 UART。 */
                bool sent;
                if(wire_generation) {
                    taskENTER_CRITICAL();
                    bool valid=wire_generation==jog_generation && jog.phase!=JOG_IDLE &&
                              jog.phase!=JOG_COMPLETE && jog_session_ready();
                    sent=valid && n && ServoUart_Send(frame,(uint16_t)n);
                    taskEXIT_CRITICAL();
                    if(!valid) {
                        if(wire_generation==jog_generation && jog.phase!=JOG_IDLE && jog.phase!=JOG_COMPLETE)
                            jog_done("DISCONNECTED");
                        publish(now);
                        return;
                    }
                } else sent=n && ServoUart_Send(frame,(uint16_t)n);
#else
                bool sent = n && ServoUart_Send(frame,(uint16_t)n);
#endif
                if(!sent)finish(SERVO_TX_ERROR,now,NULL);
                else {
#if SERVO_FASHIONSTAR_GRIPPER
                    if (!fashionstar_slot(current.slot)) fs_aligned=false;
#endif
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
