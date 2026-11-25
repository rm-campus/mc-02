#include "robot_control.h"
#include <assert.h>
#include <stdio.h>

static Fxi6x_Data frame = { .received = true, .valid = true };
static RobotControlState send(uint32_t ms, int value)
{
    frame.switch_c = (int8_t)value;
    frame.last_valid_ms = ms;
    ++frame.frame_count;
    Fxi6x_OnFrame(&frame);
    return RobotControl_Get(ms);
}

int main(void)
{
    assert(!send(0, ROBOT_SWITCH_UP).ready);
    for (unsigned i = 0; i < 80; ++i) {
        Fxi6x_OnFrame(&frame); /* 一帧被重复读取不能延长在线时间或许可启动 */
        assert(!RobotControl_Get(i).ready);
    }
    assert(!send(40, ROBOT_SWITCH_UP).ready);
    RobotControlState s = send(51, ROBOT_SWITCH_UP);
    assert(s.ready && s.mode == ROBOT_DRIVE && s.ready_count == 1);
    uint32_t generation = s.generation;
    s = send(55, ROBOT_SWITCH_MIDDLE);
    assert(s.ready && s.mode == ROBOT_WAIT && s.generation != generation);
    assert(send(65, ROBOT_SWITCH_DOWN).mode == ROBOT_WAIT);
    assert(send(94, ROBOT_SWITCH_DOWN).mode == ROBOT_WAIT);
    assert(send(95, ROBOT_SWITCH_DOWN).mode == ROBOT_WRIST);
    assert(RobotControl_Get(94).ready); /* 控制任务取时后被新帧抢占 */
    assert(RobotControl_Get(194).ready);
    assert(!RobotControl_Get(195).ready);
    assert(!send(201, ROBOT_SWITCH_MIDDLE).ready);
    assert(!send(220, ROBOT_SWITCH_MIDDLE).ready);
    s = send(251, ROBOT_SWITCH_MIDDLE);
    assert(s.mode == ROBOT_ARM && s.ready_count == 2);
    frame.failsafe = true;
    assert(!send(255, ROBOT_SWITCH_UP).ready);
    frame.failsafe = false;
    assert(!send(260, ROBOT_SWITCH_UP).ready);
    assert(!send(285, ROBOT_SWITCH_UP).ready);
    frame.frame_lost = true;
    assert(!send(310, ROBOT_SWITCH_UP).ready);
    frame.frame_lost = false;
    assert(!send(325, ROBOT_SWITCH_UP).ready);
    Fxi6x_OnStreamBreak();
    assert(!send(UINT32_MAX-40U, ROBOT_SWITCH_DOWN).ready);
    assert(!send(UINT32_MAX-10U, ROBOT_SWITCH_DOWN).ready);
    s = send(10, ROBOT_SWITCH_DOWN);
    assert(s.mode == ROBOT_WRIST && s.ready_count == 3);
    /* 新帧先于各控制任务发现超时，也必须重新累计连续帧。 */
    assert(!send(110, ROBOT_SWITCH_DOWN).ready);
    puts("robot link tests passed");
    return 0;
}
