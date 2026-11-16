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

void Mecanum_Mix(int16_t vertical, int16_t horizontal, int16_t turn, int16_t deadband,
                 float max_speed, float target_by_id[4])
{
    const float forward = MECANUM_FORWARD_SIGN * stick_fraction(vertical, deadband);
    const float right = MECANUM_RIGHT_SIGN * stick_fraction(horizontal, deadband);
    const float clockwise = MECANUM_TURN_SIGN * stick_fraction(turn, deadband);
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
