#include "mecanum.h"
#include "motor_debug_config.h"
#include <math.h>

/* 每个轴独立换算：死区内取零，死区外剩余行程线性映射到满量程，负半轴对称。 */
static float stick_fraction(int value, int deadband)
{
    if (value > deadband) return (float)(value - deadband) / (660 - deadband);
    if (value < -deadband) return (float)(value + deadband) / (660 - deadband);
    return 0.0f;
}

static void mix(float forward, float right, float clockwise, float max_speed, float target_by_id[4])
{
    /* 轮速以向前滚为正。顺时针转动时，左侧轮向前，右侧轮向后。 */
    const float front_left = forward + right + clockwise;
    const float front_right = forward - right - clockwise;
    const float rear_left = forward - right + clockwise;
    const float rear_right = forward + right - clockwise;
    /* 找出叠加后最快的一轮，四轮一起缩放以保留平移与旋转的比例。 */
    const float peak = fmaxf(fmaxf(fabsf(front_left), fabsf(front_right)),
                             fmaxf(fabsf(rear_left), fabsf(rear_right)));
    const float scale = max_speed / fmaxf(1.0f, peak);
    target_by_id[MECANUM_FRONT_LEFT_ID - 1] = front_left * scale;
    target_by_id[MECANUM_FRONT_RIGHT_ID - 1] = front_right * scale;
    target_by_id[MECANUM_REAR_LEFT_ID - 1] = rear_left * scale;
    target_by_id[MECANUM_REAR_RIGHT_ID - 1] = rear_right * scale;
}

void Mecanum_Mix(int16_t vertical, int16_t horizontal, int16_t turn, int16_t deadband,
                 float max_speed, float target_by_id[4])
{
    mix(MECANUM_FORWARD_SIGN * stick_fraction(vertical, deadband),
        MECANUM_RIGHT_SIGN * stick_fraction(horizontal, deadband),
        MECANUM_TURN_SIGN * stick_fraction(turn, deadband), max_speed, target_by_id);
}

#if ROBOT_CONTROL_ENABLE
void Mecanum_RobotTargets(const RobotControlState *s, float max_speed, float targets[4])
{
    float forward=0, right=0, turn=0;
    if (s->ready && (s->mode==ROBOT_DRIVE || s->mode==ROBOT_WRIST) && max_speed>0.0f) {
        const bool upper=s->mode==ROBOT_DRIVE;
        float ratio=upper ? ROBOT_UPPER_TURN_FRACTION : ROBOT_LOWER_TURN_FRACTION;
        if (ROBOT_WHEELBASE_M>0.0f && ROBOT_TRACK_M>0.0f && ROBOT_WHEEL_RADIUS_M>0.0f) {
            float yaw=upper ? ROBOT_UPPER_MAX_YAW_DEG_S : ROBOT_LOWER_MAX_YAW_DEG_S;
            /* 车体 rad/s × (半轴距+半轮距) / 轮半径 = 轮端 rad/s。 */
            ratio=yaw*(3.14159265f/180.0f)*(ROBOT_WHEELBASE_M+ROBOT_TRACK_M)/
                  (2.0f*ROBOT_WHEEL_RADIUS_M*max_speed);
        }
        turn=MECANUM_TURN_SIGN*stick_fraction(s->rc.rocker_r_h, ROBOT_CHASSIS_DEADBAND)*
             fminf(1.0f,fmaxf(0.0f,ratio));
        if (upper) {
            forward=MECANUM_FORWARD_SIGN*stick_fraction(s->rc.rocker_l_v, ROBOT_CHASSIS_DEADBAND);
            right=MECANUM_RIGHT_SIGN*stick_fraction(s->rc.rocker_l_h, ROBOT_CHASSIS_DEADBAND);
        }
    }
    mix(forward,right,turn,max_speed,targets);
}
#endif
