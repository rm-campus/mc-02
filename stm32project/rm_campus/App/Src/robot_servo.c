#include "robot_control.h"
#if ROBOT_CONTROL_ENABLE
#include "robot_servo.h"
#include "servo_app.h"
#include <math.h>
#include <stdlib.h>

/* 此模块只在 ServoBusTask 使用，轴表按总线 ID 排序；掩码始终按设备槽位。 */
typedef struct { int direction; float speed; } Axis;
static const Axis axes[5] = {SERVO_RC_AXIS_TABLE};
static float position[5];
static uint16_t queued_position[5];
static uint8_t initialized, previous_slots;
static uint32_t generation, last_update, last_ready_count;
static uint16_t boot_p;
static bool picked_up, have_previous_stick;
static int previous_stick;

static bool selected_id(RobotMode mode, unsigned id)
{
    return (mode == ROBOT_ARM && (id == 2 || id == 3)) ||
           (mode == ROBOT_WRIST && (id == 1 || id == 4 || id == 5));
}

bool RobotServo_CanTransmit(uint8_t id, uint32_t now)
{
    RobotControlState latest = RobotControl_Get(now);
    return latest.ready && latest.generation == generation && selected_id(latest.mode, id);
}

static bool usable(const ServoState *s, const ServoConfig *c, uint32_t now)
{
    return s->position_seen && s->online && now - s->position_ms < SERVO_POSITION_STALE_MS &&
           s->mode_valid && s->mode >= 1 && s->mode <= 4 &&
           s->position >= c->minimum && s->position <= c->maximum;
}

/* 两个等效接管点分别在 +threshold 和 -threshold。跨零时即使没采到 0 也可接管。 */
static bool crosses(int old, int current, float threshold)
{
    int low = old < current ? old : current;
    int high = old > current ? old : current;
    return (low <= threshold && threshold <= high) || (low <= -threshold && -threshold <= high);
}

void RobotServo_Process(uint32_t now)
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
        initialized=0; picked_up=false; have_previous_stick=false; last_update=now;
        generation=control.generation; previous_slots=selected; last_ready_count=control.ready_count;
        Servo_RcClearInhibit();
    }
    if (!boot_p) {
        ServoState g;
        if (Servo_CopyState(5,&g) && g.online && g.position_seen && now-g.position_ms<SERVO_POSITION_STALE_MS &&
            g.position>=500 && g.position<=2500) boot_p=g.position;
    }
    if (!selected) {
        (void)Servo_ControlRelease(SERVO_OWNER_RC);
        return;
    }
    if (Servo_RcStopPending() || !Servo_ControlClaim(SERVO_OWNER_RC,selected)) return;
    uint32_t elapsed=now-last_update;
    bool update=elapsed >= SERVO_RC_PERIOD_MS;
    if (update) last_update=now;
    /* 任务延迟时只推进一个有限步长，避免追补期间未发送的动作。 */
    float dt=(elapsed>SERVO_RC_MAX_PERIOD_MS ? SERVO_RC_PERIOD_MS : elapsed)*0.001f;
    for (unsigned slot=0; slot<SERVO_COUNT; ++slot) {
        const ServoConfig *c=Servo_GetConfig(slot);
        const unsigned id=c->id;
        uint8_t bit=(uint8_t)(1U<<slot);
        if (!(selected&bit) || id==0 || id>5) continue;
        ServoState s;
        if (!Servo_CopyState(id,&s) || !usable(&s,c,now) || (Servo_RcInhibited()&bit)) {
            if (initialized&bit) {
                Servo_RcStopMask(bit); initialized&=(uint8_t)~bit;
                if (id==5) { picked_up=false; have_previous_stick=false; }
            }
            continue;
        }
        float *target=&position[id-1];
        if (!(initialized&bit)) {
            *target=s.position;
            if (id==5 && s.target>=c->minimum && s.target<=c->maximum) *target=s.target;
            queued_position[id-1]=(uint16_t)*target;
            initialized |= bit;
        }
        const float span=s.mode<=2 ? 270.0f : 180.0f;
        const float step=axes[id-1].speed*dt*(2000.0f/span);
        if (id==5) {
            if (!boot_p || ROBOT_GRIPPER_CLOSED_P<c->minimum || ROBOT_GRIPPER_CLOSED_P>c->maximum ||
                boot_p==ROBOT_GRIPPER_CLOSED_P) continue;
            int stick=control.rc.rocker_r_v;
            if (!picked_up) {
                float stroke=(float)ROBOT_GRIPPER_CLOSED_P-boot_p;
                float held=(s.target>=c->minimum && s.target<=c->maximum)?s.target:boot_p;
                float threshold=fminf(660.0f,fmaxf(0.0f,(held-boot_p)*660.0f/stroke));
                picked_up=crosses(have_previous_stick?previous_stick:stick,stick,threshold);
                previous_stick=stick; have_previous_stick=true;
                if (!picked_up) continue;
            }
            if (!update) continue;
            float goal=boot_p+((float)ROBOT_GRIPPER_CLOSED_P-boot_p)*abs(stick)/660.0f;
            *target+=fminf(step,fmaxf(-step,goal-*target));
        } else {
            if (!update) continue;
            int stick=(id==1 || id==3) ? control.rc.rocker_l_v : control.rc.rocker_l_h;
            *target+=(float)stick*(1.0f/660.0f)*axes[id-1].direction*step;
        }
        *target=fminf(c->maximum,fmaxf(c->minimum,*target));
        uint16_t pulse=(uint16_t)lroundf(*target);
        if (pulse!=queued_position[id-1] && Servo_RcMovePulse(id,pulse,SERVO_RC_PERIOD_MS,NULL)==SERVO_QUEUED)
            queued_position[id-1]=pulse;
    }
}
#endif
