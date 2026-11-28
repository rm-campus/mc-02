#include "robot_control.h"
#if ROBOT_CONTROL_ENABLE
#include "robot_servo.h"
#include "servo_app.h"
#include <math.h>
#if SERVO_FASHIONSTAR_GRIPPER
#include "fs_servo.h"
#endif

/* 此模块只在 ServoBusTask 使用，轴表按总线 ID 排序；掩码始终按设备槽位。 */
typedef struct { int direction; float speed; } Axis;
static const Axis axes[5] = {SERVO_RC_AXIS_TABLE};
static float position[5];
static uint16_t queued_position[5];
static uint8_t initialized, previous_slots;
static uint32_t generation, last_update, last_ready_count;
static uint8_t priority_slots, arm_centered;
static uint32_t priority_ticket[SERVO_COUNT];
static int priority_direction[SERVO_COUNT], grip_stick;
static uint16_t priority_grip_pulse;
static bool grip_settled, grip_stick_changed;

static bool selected_id(RobotMode mode, unsigned id)
{
    const unsigned middle_axis=SERVO_FASHIONSTAR_GRIPPER ? 2U : 1U;
    return (mode == ROBOT_ARM && (id == middle_axis || id == 3 || id == 4)) ||
           (mode == ROBOT_WRIST && (id == 5 || (!SERVO_FASHIONSTAR_GRIPPER && id == 2)));
}

static int axis_stick(const RobotControlState *control, unsigned id)
{
    switch (id) {
    case 2: return control->rc.rocker_l_h;
    case 3: return control->rc.rocker_l_v;
    case 4: return control->rc.rocker_r_h;
    default: return control->rc.rocker_r_v;
    }
}

static float gripper_goal(int stick)
{
    const float fraction=(stick+660)/1320.0f;
#if SERVO_FASHIONSTAR_GRIPPER
    const int angle=(int)lroundf(FS_GRIPPER_OPEN_DDEG+fraction*FS_GRIPPER_TRAVEL_DDEG);
    return FsServo_PulseFromAngle(angle);
#else
    return ROBOT_GRIPPER_OPEN_P+fraction*((float)ROBOT_GRIPPER_CLOSED_P-ROBOT_GRIPPER_OPEN_P);
#endif
}

bool RobotServo_PriorityMove(unsigned slot, uint32_t ticket, uint32_t now)
{
    if (!(priority_slots & (1U << slot)) || !ticket || ticket != priority_ticket[slot]) return false;
    const unsigned id = Servo_GetConfig(slot)->id;
    const RobotControlState latest = RobotControl_Get(now);
    if (!latest.ready || latest.generation != generation || !selected_id(latest.mode, id)) return false;
    const int stick = axis_stick(&latest, id);
    if (id==5) {
        const long goal=lroundf(gripper_goal(stick));
        return priority_direction[slot]>0 ? goal>=priority_grip_pulse : goal<=priority_grip_pulse;
    }
    return stick*priority_direction[slot]>0;
}

void RobotServo_MoveSent(unsigned slot, uint32_t ticket)
{
    if ((priority_slots & (1U << slot)) && ticket == priority_ticket[slot]) priority_slots = 0;
}

bool RobotServo_CanTransmit(uint8_t id, uint32_t now)
{
    RobotControlState latest = RobotControl_Get(now);
    return latest.ready && latest.generation == generation && selected_id(latest.mode, id);
}

static bool usable(const ServoState *s, const ServoConfig *c)
{
    return s->position_seen && s->position >= c->minimum && s->position <= c->maximum;
}

bool RobotServo_Process(uint32_t now)
{
    RobotControlState control = RobotControl_Get(now);
    uint8_t selected = 0, grip_slot = 0, arm_sticks = 0;
    for (unsigned i=0; i<SERVO_COUNT; ++i) {
        const unsigned id = Servo_GetConfig(i)->id;
        if (id==5) grip_slot = 1U<<i;
        if (selected_id(control.mode,id)) {
            selected |= 1U<<i;
            if (id!=5 && axis_stick(&control,id)) arm_sticks |= 1U<<i;
        }
    }
    if (generation != control.generation || previous_slots != selected || last_ready_count != control.ready_count) {
        const bool interrupted = !control.ready || last_ready_count != control.ready_count;
        Servo_RobotCancelQueuedGrip();
        Servo_RcStopMask(interrupted ? previous_slots : previous_slots & (uint8_t)~grip_slot);
        initialized=0; last_update=now;
        priority_slots=arm_centered=0; grip_settled=false; grip_stick_changed=true;
        generation=control.generation; previous_slots=selected; last_ready_count=control.ready_count;
        Servo_RcClearInhibit();
    }
    if (!selected) {
        (void)Servo_ControlRelease(SERVO_OWNER_RC);
        return false;
    }
    if (!Servo_ControlClaim(SERVO_OWNER_RC,selected)) {
        priority_slots=arm_centered=0; grip_settled=false; grip_stick_changed=true;
        return false;
    }
    const uint8_t stops_pending=Servo_RcStopPending();
    if (stops_pending) {
        priority_slots=arm_centered=0; grip_settled=false; grip_stick_changed=true;
    } else {
        const uint8_t fresh=arm_centered&arm_sticks;
        priority_slots &= (uint8_t)(arm_sticks|grip_slot);
        priority_slots |= fresh;
        for (unsigned slot=0; fresh && slot<SERVO_COUNT; ++slot)
            if (fresh&(1U<<slot)) priority_ticket[slot]=0;
        arm_centered=selected&(uint8_t)~(arm_sticks|grip_slot);
    }
    uint32_t elapsed=now-last_update;
    bool update=elapsed >= SERVO_RC_PERIOD_MS;
    if (update) last_update=now;
    /* 任务延迟时只推进一个有限步长，避免追补期间未发送的动作。 */
    float dt=(elapsed>SERVO_RC_MAX_PERIOD_MS ? SERVO_RC_PERIOD_MS : elapsed)*0.001f;
    bool motion_requested = false;
    for (unsigned slot=0; slot<SERVO_COUNT; ++slot) {
        const ServoConfig *c=Servo_GetConfig(slot);
        const unsigned id=c->id;
        uint8_t bit=(uint8_t)(1U<<slot);
        if (!(selected&bit) || id==0 || id>5) continue;
        ServoState s;
        if (!Servo_CopyState(id,&s) || !usable(&s,c) || (Servo_RcInhibited()&bit)) {
            priority_slots&=(uint8_t)~bit;
            if (id==5) { grip_settled=false; grip_stick_changed=true; }
            if (initialized&bit) {
                if (!(stops_pending&bit)) Servo_RcStopMask(bit);
                initialized&=(uint8_t)~bit;
            }
            continue;
        }
        if (stops_pending) continue;
        float *target=&position[id-1];
        if (!(initialized&bit)) {
            *target=s.position;
            queued_position[id-1]=(uint16_t)*target;
            initialized |= bit;
        }
        /* 模式只用于步长换算。未读到 180° 位置模式时按 270° 换算。 */
        const float span=SERVO_FASHIONSTAR_GRIPPER && id==5 ? 360.0f :
            (s.mode_valid && (s.mode==3U || s.mode==4U) ? 180.0f : 270.0f);
        const float scale=id==5 ? 1.0f : ROBOT_JOINT_SPEED_SCALE;
        const float speed=fminf(SERVO_MAX_SPEED_DEG_S,fmaxf(0.0f,axes[id-1].speed*scale));
        const float step=speed*dt*(2000.0f/span);
        const int input=axis_stick(&control,id);
        float goal=0.0f;
        if (id==5) {
#if !SERVO_FASHIONSTAR_GRIPPER
            if (ROBOT_GRIPPER_CLOSED_P==0U) continue;
#endif
            goal=gripper_goal(input);
            /* 非回中夹爪从任意已到达的目标再次拨动时，只优先首条新目标。 */
            if (input!=grip_stick) {
                if (grip_settled) {
                    priority_slots|=bit;
                    priority_ticket[slot]=0;
                }
                grip_stick=input;
                grip_settled=false;
                grip_stick_changed=true;
            }
            motion_requested |= fabsf(goal-*target)>=1.0f;
            if (!update) continue;
            *target+=fminf(step,fmaxf(-step,goal-*target));
        } else {
            const int stick=input*axes[id-1].direction;
            motion_requested |= (stick > ROBOT_SERVO_DEFER_POLL_STICK && *target < c->maximum) ||
                                (stick < -ROBOT_SERVO_DEFER_POLL_STICK && *target > c->minimum);
            if (!update) continue;
            *target+=(float)stick*(1.0f/660.0f)*step;
            if (id==3 || id==4) {
                /* 直接限制累加值，避免受阻时攒下目标，反拨杆时也能立即退出限幅。 */
                const float lead=ROBOT_SHOULDER_WRIST_MAX_LEAD_P;
                *target=fminf(s.position+lead,fmaxf(s.position-lead,*target));
            }
        }
        *target=fminf(c->maximum,fmaxf(c->minimum,*target));
        uint16_t pulse=(uint16_t)lroundf(*target);
        if (id==5) {
            const float limited_goal=fminf(c->maximum,fmaxf(c->minimum,goal));
            grip_settled=!grip_stick_changed && lroundf(limited_goal)==pulse;
            grip_stick_changed=false;
        }
        uint32_t ticket;
        if (pulse!=queued_position[id-1] && Servo_RcMovePulse(id,pulse,SERVO_RC_PERIOD_MS,&ticket)==SERVO_QUEUED) {
            if (priority_slots&bit) {
                priority_ticket[slot]=ticket;
                if (id==5) {
                    priority_direction[slot]=pulse>queued_position[id-1] ? 1 : -1;
                    priority_grip_pulse=pulse;
                } else priority_direction[slot]=input;
            }
            queued_position[id-1]=pulse;
        }
    }
    return motion_requested;
}
#endif
