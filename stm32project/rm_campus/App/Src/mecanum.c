#include "mecanum.h"
#include "motor_debug_config.h"
#include <math.h>

/* 每个轴独立换算。以死区 40 为例，±40 内取零，从 41 到 660 连续映射到
 * 1/620 到 1；负半轴对称。直推时少量横向偏移不会进入轮速计算。 */
static float stick_fraction(int value, int deadband)
{
    if (value > deadband) return (float)(value - deadband) / (660 - deadband);
    if (value < -deadband) return (float)(value + deadband) / (660 - deadband);
    return 0.0f;
}

void Mecanum_Translate(int16_t vertical, int16_t horizontal, int16_t deadband,
                       float max_speed, float target_by_id[4])
{
    const float forward = MECANUM_FORWARD_SIGN * stick_fraction(vertical, deadband);
    const float right = MECANUM_RIGHT_SIGN * stick_fraction(horizontal, deadband);
    /* 按车身定义前、右为正，轮速以向前滚为正。X 型轮组纯平移时，
     * 左前与右后同速，右前与左后同速；纯右移时两组方向相反。 */
    const float front_left = forward + right;
    const float front_right = forward - right;
    /* 四轮等比例缩放，斜推满杆时保留平移方向。 */
    const float peak = fmaxf(fabsf(front_left), fabsf(front_right));
    const float scale = max_speed / fmaxf(1.0f, peak);
    target_by_id[MECANUM_FRONT_LEFT_ID - 1] = front_left * scale;
    target_by_id[MECANUM_FRONT_RIGHT_ID - 1] = front_right * scale;
    target_by_id[MECANUM_REAR_LEFT_ID - 1] = front_right * scale;
    target_by_id[MECANUM_REAR_RIGHT_ID - 1] = front_left * scale;
}
