#include "motor_debug.h"
#include "motor_debug_config.h"
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
    in.target_rad_s = (float)in.stick * (c.max_speed / 660.0f);
    MotorDebug_Step(&d, &c, &in);
    assert(fabsf(d.output_a) <= c.current_limit);
    assert(fabsf(d.integral_a) <= c.integral_limit);
}

static void reset(void)
{
    c = MotorDebug_DefaultConfig();
    c.enabled = true; c.kp = 0.5f; c.ki = 0.4f;
    in = (MotorDebugInput){.rc_valid=true, .rc_received=true, .motor_received=true,
        .can_ok=true, .enable_switch=-1, .temperature=25};
    MotorDebug_Init(&d);
}

static void arm(void)
{
    in.enable_switch=1; in.stick = 0; in.speed = 0.0f;
    step(1);
    in.enable_switch = 0; step(1); /* 三档开关经过中档。 */
    in.enable_switch=-1;
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
    in.enable_switch=1; step(1);
    in.enable_switch=-1; in.stick=660;
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
    in.can_ok=false; step(1);
#if MOTOR_DEBUG_CAN_FAULT_ENABLE
    zero_and_rearm_required(MOTOR_DEBUG_CAN_FAULT);
    MotorDebug_TxFailed(&d); zero_and_rearm_required(MOTOR_DEBUG_CAN_FAULT);
#else
    assert(d.state==MOTOR_DEBUG_RUNNING && d.reason==MOTOR_DEBUG_OK);
    in.stick=660; step(1);
    MotorDebug before=d;
    MotorDebug_TxFailed(&d);
    assert(d.state==before.state && d.command_raw==before.command_raw &&
           d.integral_a==before.integral_a && d.reference==before.reference);
    /* CAN 异常持续时，反馈离线保护依然生效。 */
    in.motor_received=false; step(1); zero_and_rearm_required(MOTOR_DEBUG_MOTOR_OFFLINE);
#endif
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
    in.enable_switch=-1;
    for(unsigned n=0;n<400;++n) step(1);
    assert(d.state!=MOTOR_DEBUG_RUNNING);
    arm();
    c.motor_id=0; step(1);
    assert(d.reason==MOTOR_DEBUG_BAD_CONFIG && d.command_raw==0);
    c=MotorDebug_DefaultConfig(); c.enabled=true; c.kp=NAN; step(1);
    assert(d.reason==MOTOR_DEBUG_BAD_CONFIG && d.command_raw==0);
    c=MotorDebug_DefaultConfig(); c.enabled=false; in.stick=660; step(1);
    assert(d.state==MOTOR_DEBUG_DISABLED && d.command_raw==0);
}

static void step_target_and_pi(void)
{
    reset(); arm();
    in.stick=660;
    step(1);
    assert(fabsf(d.reference-c.max_speed)<1e-5f);
    assert(d.command_raw <= (int16_t)(c.current_limit*16384.0f/20.0f));
    const float windup=d.integral_a;
    for(unsigned n=0;n<1000;++n) step(1);
    assert(d.integral_a==windup); /* 固定堵转输入使输出饱和，积分不再增加。 */
    in.stick=-660;
    step(1);
    assert(fabsf(d.reference+c.max_speed)<0.01f && d.command_raw<0);

    in.stick=0; step(1);
    assert(d.reference == 0.0f);

    /* 简化惯性+粘性阻力模型仅验证闭环符号/单位/恢复，不代表实物参数。 */
    reset(); c.max_speed=3.0f; c.kp=0.8f; c.ki=0.8f; arm(); in.stick=660;
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

static void continuous_running(void)
{
    for(unsigned wrap=0;wrap<2;++wrap) {
        reset();
        if(wrap) in.now_ms=UINT32_MAX-20000U;
        arm(); in.stick=330;
        for(unsigned n=0;n<60000U;++n) {
            step(1);
            assert(d.state==MOTOR_DEBUG_RUNNING && d.reason==MOTOR_DEBUG_OK);
        }
        assert(d.command_raw>0);
        /* 长时间运行后仍能通过拨杆关闭，再执行正常启用流程。 */
        in.enable_switch=1; step(1);
        assert(d.state==MOTOR_DEBUG_WAIT_CENTER && d.command_raw==0);
        arm(); assert(d.state==MOTOR_DEBUG_RUNNING);
    }
}

int main(void)
{
    MotorDebugConfig defaults=MotorDebug_DefaultConfig();
    assert(defaults.enabled && defaults.kp == MOTOR_DEBUG_KP && defaults.ki == MOTOR_DEBUG_KI);
    assert(fabsf(defaults.max_speed * 60.0f / 6.283185307179586f - 400.0f) < 0.001f);
    safety(); step_target_and_pi(); continuous_running();
    reset(); arm();
    for (int stick = -660; stick <= 660; ++stick) {
        in.stick=(int16_t)stick; step(1);
        assert(fabsf(d.target * 60.0f / 6.283185307179586f - stick * 400.0f / 660.0f) < 0.001f);
    }
    reset(); in.enable_switch=1; step(1); in.enable_switch=-1;
    step(1);
    for (unsigned n=0;n<299;++n) step(1);
    assert(d.state != MOTOR_DEBUG_RUNNING);
    in.stick=41; step(1); in.stick=0; step(1);
    for (unsigned n=0;n<299;++n) step(1);
    assert(d.state != MOTOR_DEBUG_RUNNING);
    step(1); assert(d.state == MOTOR_DEBUG_RUNNING);
    printf("PASS: CAN_FAULT protection=%d; arming, linear mapping, faults, direct speed reference, PI, 60s continuous operation and tick wrap.\n", MOTOR_DEBUG_CAN_FAULT_ENABLE);
    return 0;
}
