#include "robot_control.h"
#if ROBOT_CONTROL_ENABLE
#include "FreeRTOS.h"
#include "task.h"

static RobotControlState state;
static RobotMode candidate;
static uint32_t candidate_since, first_frame_ms, last_frame_ms;
static unsigned good_frames;

static RobotMode switch_mode(int value)
{
    switch (value) {
    case ROBOT_SWITCH_UP: return ROBOT_DRIVE;
    case ROBOT_SWITCH_MIDDLE: return ROBOT_ARM;
    case ROBOT_SWITCH_DOWN: return ROBOT_WRIST;
    default: return ROBOT_WAIT;
    }
}

/* 调用者持短临界区。退出或变档都留下代数，下游可发现两次任务运行之间发生的切换。 */
static void select_mode(RobotMode next)
{
    if (state.mode != next) {
        state.mode = next;
        ++state.generation;
    }
}

static void lose_link(void)
{
    if (state.ready && state.mode == ROBOT_WAIT) ++state.generation;
    select_mode(ROBOT_WAIT);
    state.ready = false;
    state.rc.valid = false;
    candidate = ROBOT_WAIT;
    good_frames = 0;
}

static void check_age(uint32_t now)
{
    /* 调用者可能刚在高优先级 RcTask 抢占前取过 now，新帧会比它晚 1 tick。 */
    if (good_frames && (int32_t)(now - last_frame_ms) >= (int32_t)ROBOT_LINK_TIMEOUT_MS)
        lose_link();
}

void Fxi6x_OnStreamBreak(void)
{
    taskENTER_CRITICAL();
    lose_link();
    taskEXIT_CRITICAL();
}

void Fxi6x_OnFrame(const Fxi6x_Data *frame)
{
    const RobotMode next = switch_mode(frame->switch_c);
    if (!frame->received || !frame->valid || frame->failsafe || frame->frame_lost || next == ROBOT_WAIT) {
        Fxi6x_OnStreamBreak();
        return;
    }
    const uint32_t now = frame->last_valid_ms;
    taskENTER_CRITICAL();
    check_age(now);
    if (!good_frames || frame->frame_count != state.rc.frame_count) {
        if (!good_frames) first_frame_ms = now;
        if (good_frames < ROBOT_LINK_MIN_FRAMES) ++good_frames;
        state.rc = *frame;
        last_frame_ms = now;
        if (next != candidate) {
            candidate = next;
            candidate_since = now;
            select_mode(ROBOT_WAIT); /* 立即撤销旧档；新档只在真实新帧上确认。 */
        }
        if (!state.ready && good_frames >= ROBOT_LINK_MIN_FRAMES &&
            now - first_frame_ms >= ROBOT_LINK_MIN_MS) {
            state.ready = true;
            ++state.ready_count;
        }
        if (state.ready && now - candidate_since >= ROBOT_SWITCH_STABLE_MS)
            select_mode(next);
    }
    taskEXIT_CRITICAL();
}

RobotControlState RobotControl_Get(uint32_t now)
{
    taskENTER_CRITICAL();
    check_age(now);
    RobotControlState snapshot = state;
    taskEXIT_CRITICAL();
    return snapshot;
}
#endif
