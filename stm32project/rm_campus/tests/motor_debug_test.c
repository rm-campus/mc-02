#include "motor_debug.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>

static MotorDebugConfig c;
static MotorDebugInput in;
static MotorDebug d;

static void step(uint32_t ms)
{
    in.now_ms += ms;
    in.rc_ms = in.feedback_ms = in.now_ms;
    in.period_ms = ms;
    MotorDebug_Step(&d, &c, &in);
    assert(fabsf(d.output_a) <= c.current_limit);
    assert(fabsf(d.integral_a) <= c.integral_limit);
}

static void reset(void)
{
    c = MotorDebug_DefaultConfig();
    c.enabled = true; c.kp = 0.5f; c.ki = 0.4f;
    in = (MotorDebugInput){.rc_valid=true, .rc_received=true, .motor_received=true,
        .can_ok=true, .enable_switch=1, .temperature=25};
    MotorDebug_Init(&d);
}

static void arm(void)
{
    in.enable_switch = -1; in.stick = 0; in.speed = 0.0f;
    step(1);
    in.enable_switch = 0; step(1); /* 三档开关经过中档。 */
    in.enable_switch = 1;
    for (unsigned n=0; n<=c.arm_hold_ms; ++n) step(1);
    assert(d.state == MOTOR_DEBUG_RUNNING);
}

static void zero_and_rearm_required(MotorDebugReason reason)
{
    assert(d.state == MOTOR_DEBUG_WAIT_OFF && d.reason == reason);
    assert(d.command_raw == 0 && d.reference == 0.0f && d.integral_a == 0.0f);
    in.rc_valid=true; in.rc_received=true; in.motor_received=true; in.can_ok=true;
    in.temperature=25; in.speed=0; in.measured_current=0;
    for (unsigned n=0;n<400;++n) step(1);
    assert(d.state != MOTOR_DEBUG_RUNNING && d.command_raw == 0);
    arm();
}

static void safety(void)
{
    reset();
    for (unsigned n=0;n<500;++n) step(1);
    assert(d.state == MOTOR_DEBUG_WAIT_OFF); /* 开机开关在启用档，不能自动运行。 */
    in.enable_switch=-1; step(1);
    in.enable_switch=1; in.stick=660;
    for (unsigned n=0;n<400;++n) step(1);
    assert(d.state != MOTOR_DEBUG_RUNNING); /* 摇杆未回中。 */
    arm();
    in.stick=660; for(unsigned n=0;n<1000;++n) step(1);
    assert(d.command_raw>0);
    in.rc_valid=false; step(1); zero_and_rearm_required(MOTOR_DEBUG_RC_OFFLINE);
    in.motor_received=false; step(1); zero_and_rearm_required(MOTOR_DEBUG_MOTOR_OFFLINE);
    in.now_ms += c.feedback_timeout_ms;
    in.rc_ms=in.now_ms; in.period_ms=1;
    MotorDebug_Step(&d,&c,&in); zero_and_rearm_required(MOTOR_DEBUG_MOTOR_OFFLINE);
    in.now_ms += c.rc_timeout_ms; in.feedback_ms=in.now_ms;
    MotorDebug_Step(&d,&c,&in); zero_and_rearm_required(MOTOR_DEBUG_RC_OFFLINE);
    in.can_ok=false; step(1); zero_and_rearm_required(MOTOR_DEBUG_CAN_FAULT);
    MotorDebug_TxFailed(&d); zero_and_rearm_required(MOTOR_DEBUG_CAN_FAULT);
    step(6); zero_and_rearm_required(MOTOR_DEBUG_TIMING_FAULT);
    step(0); zero_and_rearm_required(MOTOR_DEBUG_TIMING_FAULT);
    in.temperature=c.max_temperature; step(1); zero_and_rearm_required(MOTOR_DEBUG_TEMPERATURE);
    in.speed=c.overspeed; step(1); zero_and_rearm_required(MOTOR_DEBUG_SPEED_FAULT);
    in.speed=NAN; step(1); zero_and_rearm_required(MOTOR_DEBUG_SPEED_FAULT);
    in.measured_current=INFINITY; step(1); zero_and_rearm_required(MOTOR_DEBUG_SPEED_FAULT);
    in.stick=661; step(1); zero_and_rearm_required(MOTOR_DEBUG_RC_OFFLINE);
    in.stick=0;
    arm();
    in.enable_switch=0; step(1);
    assert(d.command_raw==0 && !d.off_seen);
    in.enable_switch=1;
    for(unsigned n=0;n<400;++n) step(1);
    assert(d.state!=MOTOR_DEBUG_RUNNING);
    arm();
    for(unsigned n=0;n<c.max_run_ms;++n) step(1);
    zero_and_rearm_required(MOTOR_DEBUG_RUN_TIMEOUT);
    c.motor_id=0; step(1);
    assert(d.reason==MOTOR_DEBUG_BAD_CONFIG && d.command_raw==0);
    c=MotorDebug_DefaultConfig(); c.enabled=true; c.kp=NAN; step(1);
    assert(d.reason==MOTOR_DEBUG_BAD_CONFIG && d.command_raw==0);
    c=MotorDebug_DefaultConfig(); c.enabled=false; in.stick=660; step(1);
    assert(d.state==MOTOR_DEBUG_DISABLED && d.command_raw==0);
}

static void ramp_and_pi(void)
{
    reset(); arm();
    in.stick=660;
    for (unsigned n=0;n<1000;++n) {
        const float before=d.reference;
        step(n%2 ? 3U : 1U);
        assert(fabsf(d.reference-before) <= c.accel*in.period_ms*0.001f+1e-6f);
    }
    assert(fabsf(d.reference-c.max_speed)<1e-5f);
    assert(d.command_raw <= (int16_t)(c.current_limit*16384.0f/20.0f));
    const float windup=d.integral_a;
    for(unsigned n=0;n<1000;++n) step(1);
    assert(d.integral_a==windup); /* 固定堵转输入使输出饱和，积分不再增加。 */
    in.stick=-660;
    for(unsigned n=0;n<3001;++n) step(1);
    assert(d.reference< -2.99f && d.command_raw<0);

    /* 简化惯性+粘性阻力模型仅验证闭环符号/单位/恢复，不代表实物参数。 */
    reset(); c.kp=0.8f; c.ki=0.8f; arm(); in.stick=660;
    for(unsigned n=0;n<5000;++n) {
        step(1);
        in.speed += (d.output_a*10.0f - in.speed)*0.001f;
    }
    assert(fabsf(in.speed-c.max_speed)<0.05f);
    in.stick=0;
    for(unsigned n=0;n<3500;++n) {
        step(1);
        in.speed += (d.output_a*10.0f - in.speed)*0.001f;
    }
    assert(fabsf(in.speed)<0.05f);

    /* HAL 毫秒计数回绕不应误触发时间保护。 */
    reset(); in.now_ms=UINT32_MAX-150U; arm();
    assert(d.state==MOTOR_DEBUG_RUNNING);
}

int main(void)
{
    MotorDebugConfig defaults=MotorDebug_DefaultConfig();
    assert(!defaults.enabled && defaults.kp==0 && defaults.ki==0);
    safety(); ramp_and_pi();
    puts("PASS: default off, arming/rearming, timeouts/CAN/timing/temperature/invalid data, ramp/reversal, PI anti-windup and simulated tracking, tick wrap.");
    return 0;
}
