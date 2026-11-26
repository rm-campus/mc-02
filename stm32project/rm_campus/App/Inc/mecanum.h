#ifndef MECANUM_H
#define MECANUM_H
#include <stdint.h>
#include "robot_control_config.h"
#if ROBOT_CONTROL_ENABLE
#include "robot_control.h"
void Mecanum_RobotTargets(const RobotControlState *state, float max_speed, float target_by_id[4]);
#endif

/* vertical/horizontal/turn 分别是 CH2 前后、CH4 左右、CH1 旋转（-660～660）。
 * 平移轴用 deadband，自转轴用 MECANUM_TURN_DEADBAND。角速度正值对应俯视顺时针。
 * max_speed 为输出轴角速度上限（rad/s）。target_by_id[0..3] 依次交给 ID1～4
 * 的速度 PI，各轴正方向及四个轮位对应的 ID 在 motor_debug_config.h 中配置。 */
void Mecanum_Mix(int16_t vertical, int16_t horizontal, int16_t turn, int16_t deadband,
                 float max_speed, float target_by_id[4]);
#endif
