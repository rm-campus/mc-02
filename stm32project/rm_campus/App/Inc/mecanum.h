#ifndef MECANUM_H
#define MECANUM_H
#include <stdint.h>

/* vertical/horizontal 分别是 CH2/CH4 原始值（-660～660），deadband 使用同一量程。
 * max_speed 为输出轴角速度上限（rad/s）。target_by_id[0..3] 依次交给 ID1～4
 * 的速度 PI，各轴正方向及四个轮位对应的 ID 在 motor_debug_config.h 中配置。 */
void Mecanum_Translate(int16_t vertical, int16_t horizontal, int16_t deadband,
                       float max_speed, float target_by_id[4]);
#endif
