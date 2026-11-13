#include "motor_debug.h"
#include "motor_debug_config.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

static float clampf(float value, float low, float high)
{
    return value < low ? low : value > high ? high : value;
}

MotorDebugConfig MotorDebug_DefaultConfig(void)
{
    const MotorDebugConfig config = {
        .enabled = MOTOR_DEBUG_ENABLE != 0, .motor_id = MOTOR_DEBUG_MOTOR_ID,
        .kp = MOTOR_DEBUG_KP, .ki = MOTOR_DEBUG_KI,
        .max_speed = MOTOR_DEBUG_MAX_SPEED,
        .current_limit = MOTOR_DEBUG_CURRENT_LIMIT, .integral_limit = MOTOR_DEBUG_INTEGRAL_LIMIT,
        .overspeed = MOTOR_DEBUG_OVERSPEED, .deadband = MOTOR_DEBUG_STICK_DEADBAND,
        .max_temperature = MOTOR_DEBUG_MAX_TEMPERATURE,
        .arm_hold_ms = MOTOR_DEBUG_ARM_HOLD_MS,
        .rc_timeout_ms = MOTOR_DEBUG_RC_TIMEOUT_MS,
        .feedback_timeout_ms = MOTOR_DEBUG_FEEDBACK_TIMEOUT_MS,
        .max_period_ms = MOTOR_DEBUG_MAX_PERIOD_MS
    };
    return config;
}

MotorDebugConfig MotorDebug_WheelConfig(uint8_t motor_id)
{
    /* M1～M4 对应 C620 ID 1～4。CH8 切换目标算法时，每台电机继续使用
     * 自己的 Kp、Ki 和积分状态。轮位与 ID 的对应关系在麦轮解算中处理。 */
    static const float kp[4] = {MOTOR_DEBUG_M1_KP, MOTOR_DEBUG_M2_KP, MOTOR_DEBUG_M3_KP, MOTOR_DEBUG_M4_KP};
    static const float ki[4] = {MOTOR_DEBUG_M1_KI, MOTOR_DEBUG_M2_KI, MOTOR_DEBUG_M3_KI, MOTOR_DEBUG_M4_KI};
    MotorDebugConfig c = MotorDebug_DefaultConfig();
    c.motor_id = motor_id;
    if (motor_id >= 1U && motor_id <= 4U) { c.kp = kp[motor_id-1U]; c.ki = ki[motor_id-1U]; }
    return c;
}

void MotorDebug_Init(MotorDebug *debug)
{
    if (debug != NULL) {
        memset(debug, 0, sizeof(*debug));
        debug->state = MOTOR_DEBUG_WAIT_OFF;
    }
}

static void stop(MotorDebug *debug, MotorDebugReason reason)
{
    MotorDebug_Init(debug);
    debug->reason = reason;
}

void MotorDebug_TxFailed(MotorDebug *debug)
{
#if MOTOR_DEBUG_CAN_FAULT_ENABLE
    if (debug != NULL) stop(debug, MOTOR_DEBUG_CAN_FAULT);
#else
    (void)debug;
#endif
}

/* 台架模式额外限制参数范围，避免单位误填绕过低速/低电流边界。 */
static bool config_valid(const MotorDebugConfig *c)
{
    return c->motor_id >= 1U && c->motor_id <= 4U &&
        isfinite(c->kp) && c->kp >= 0.0f && c->kp <= 100.0f &&
        isfinite(c->ki) && c->ki >= 0.0f && c->ki <= 100.0f &&
        isfinite(c->max_speed) && c->max_speed > 0.0f && c->max_speed <= MOTOR_DEBUG_RPM_TO_RAD_S(450.0f) &&
        isfinite(c->current_limit) && c->current_limit > 0.0f && c->current_limit <= 20.0f &&
        isfinite(c->integral_limit) && c->integral_limit >= 0.0f &&
        c->integral_limit <= c->current_limit &&
        isfinite(c->overspeed) && c->overspeed > c->max_speed && c->overspeed <= MOTOR_DEBUG_RPM_TO_RAD_S(600.0f) &&
        c->deadband >= 0 && c->deadband < 660 &&
        c->max_temperature > 0U && c->max_temperature <= 125U &&
        c->arm_hold_ms >= 300U && c->arm_hold_ms <= 3000U &&
        c->rc_timeout_ms > 0U && c->rc_timeout_ms <= 100U &&
        c->feedback_timeout_ms > 0U && c->feedback_timeout_ms <= 100U &&
        c->max_period_ms > 0U && c->max_period_ms <= 5U;
}

void MotorDebug_Step(MotorDebug *d, const MotorDebugConfig *c, const MotorDebugInput *in)
{
    if (d == NULL || c == NULL || in == NULL) return;
    if (!c->enabled) {
        stop(d, MOTOR_DEBUG_OK);
        d->state = MOTOR_DEBUG_DISABLED;
        return;
    }
    MotorDebugReason reason = MOTOR_DEBUG_OK;
    if (!config_valid(c)) reason = MOTOR_DEBUG_BAD_CONFIG;
#if MOTOR_DEBUG_CAN_FAULT_ENABLE
    else if (!in->can_ok) reason = MOTOR_DEBUG_CAN_FAULT;
#endif
    /* 无符号毫秒差可跨 tick 回绕。帧龄恰好到超时阈值就失效，
     * 控制周期则允许等于 max_period_ms，零间隔或更长的间隔才判故障。 */
    else if (!in->rc_received || !in->rc_valid ||
             (uint32_t)(in->now_ms - in->rc_ms) >= c->rc_timeout_ms ||
             in->stick < -660 || in->stick > 660 ||
             in->enable_switch < -1 || in->enable_switch > 1) reason = MOTOR_DEBUG_RC_OFFLINE;
    else if (!in->motor_received ||
             (uint32_t)(in->now_ms - in->feedback_ms) >= c->feedback_timeout_ms)
        reason = MOTOR_DEBUG_MOTOR_OFFLINE;
    else if (in->period_ms == 0U || in->period_ms > c->max_period_ms)
        reason = MOTOR_DEBUG_TIMING_FAULT;
    else if (in->temperature >= c->max_temperature) reason = MOTOR_DEBUG_TEMPERATURE;
    else if (!isfinite(in->speed) || !isfinite(in->measured_current) ||
             fabsf(in->speed) >= c->overspeed) reason = MOTOR_DEBUG_SPEED_FAULT;
    if (reason != MOTOR_DEBUG_OK) {
        stop(d, reason);
        return;
    }

    if (in->enable_switch != -1) {
        /* 三档开关从 +1 经过 0 到 -1 时保留关闭确认；
         * 从运行态回到 MIDDLE 则撤销确认，必须再次到 +1。 */
        const bool off_seen = in->enable_switch == 1 ||
            (d->off_seen && d->state != MOTOR_DEBUG_RUNNING);
        stop(d, MOTOR_DEBUG_OK);
        d->off_seen = off_seen;
        d->state = d->off_seen ? MOTOR_DEBUG_WAIT_CENTER : MOTOR_DEBUG_WAIT_OFF;
        return;
    }
    if (!d->off_seen) return;
    if (d->state != MOTOR_DEBUG_RUNNING) {
        if (in->stick < -c->deadband || in->stick > c->deadband ||
            in->lateral_stick < -c->deadband || in->lateral_stick > c->deadband) {
            d->centering = false;
            return;
        }
        if (!d->centering) {
            d->centering = true;
            d->center_ms = in->now_ms;
        }
        if ((uint32_t)(in->now_ms - d->center_ms) < c->arm_hold_ms) return;
        /* 不在启用瞬间制动仍在转动的轮子；等待其基本静止。 */
        if (fabsf(in->speed) > 0.5f) {
            d->centering = false;
            return;
        }
        d->state = MOTOR_DEBUG_RUNNING;
    }
    d->target = in->target_rad_s;
    const float dt = (float)in->period_ms * 0.001f;
    d->reference = d->target; /* 目标直接进入速度环；运行中回中时，PI 仍可输出制动电流。 */
    const float error = d->reference - in->speed;
    const float proportional = c->kp * error;
    const float candidate = clampf(d->integral_a + c->ki * error * dt,
                                   -c->integral_limit, c->integral_limit);
    const float sum = proportional + candidate;
    /* 条件积分：饱和时阻止积分继续朝饱和方向增大，允许反向释放。 */
    if ((sum <= c->current_limit && sum >= -c->current_limit) ||
        (sum > c->current_limit && error < 0.0f) ||
        (sum < -c->current_limit && error > 0.0f)) d->integral_a = candidate;
    d->output_a = clampf(proportional + d->integral_a, -c->current_limit, c->current_limit);
    d->command_raw = (int16_t)(d->output_a * (16384.0f / 20.0f));
}
