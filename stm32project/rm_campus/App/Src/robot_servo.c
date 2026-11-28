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

static bool selected_id(RobotMode mode, unsigned id)
{
    return (mode == ROBOT_ARM && (id == 2 || id == 3 || id == 4)) ||
           (mode == ROBOT_WRIST && id == 5);
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
    uint8_t selected = 0, grip_slot = 0;
    for (unsigned i=0; i<SERVO_COUNT; ++i) {
        const unsigned id = Servo_GetConfig(i)->id;
        if (id==5) grip_slot = 1U<<i;
        if (selected_id(control.mode,id)) selected |= 1U<<i;
    }
    if (generation != control.generation || previous_slots != selected || last_ready_count != control.ready_count) {
        const bool interrupted = !control.ready || last_ready_count != control.ready_count;
        Servo_RobotCancelQueuedGrip();
        Servo_RcStopMask(interrupted ? previous_slots : previous_slots & (uint8_t)~grip_slot);
        initialized=0; last_update=now;
        generation=control.generation; previous_slots=selected; last_ready_count=control.ready_count;
        Servo_RcClearInhibit();
    }
    if (!selected) {
        (void)Servo_ControlRelease(SERVO_OWNER_RC);
        return false;
    }
    if (!Servo_ControlClaim(SERVO_OWNER_RC,selected)) return false;
    const uint8_t stops_pending=Servo_RcStopPending();
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
        const float speed = axes[id-1].speed * (id==5 ? 1.0f : ROBOT_JOINT_SPEED_SCALE);
        const float step=speed*dt*(2000.0f/span);
        if (id==5) {
            const float fraction=(control.rc.rocker_r_v+660)/1320.0f;
#if SERVO_FASHIONSTAR_GRIPPER
            const int angle=(int)lroundf(FS_GRIPPER_OPEN_DDEG+fraction*FS_GRIPPER_TRAVEL_DDEG);
            const float goal=FsServo_PulseFromAngle(angle);
#else
            if (ROBOT_GRIPPER_CLOSED_P==0U) continue;
            const float goal=ROBOT_GRIPPER_OPEN_P+fraction*((float)ROBOT_GRIPPER_CLOSED_P-ROBOT_GRIPPER_OPEN_P);
#endif
            motion_requested |= fabsf(goal-*target)>=1.0f;
            if (!update) continue;
            *target+=fminf(step,fmaxf(-step,goal-*target));
        } else {
            int stick = control.rc.rocker_l_h;
            if (id==3) stick = control.rc.rocker_l_v;
            if (id==4) stick = control.rc.rocker_r_h;
            stick *= axes[id-1].direction;
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
        if (pulse!=queued_position[id-1] && Servo_RcMovePulse(id,pulse,SERVO_RC_PERIOD_MS,NULL)==SERVO_QUEUED)
            queued_position[id-1]=pulse;
    }
    return motion_requested;
}
#endif
