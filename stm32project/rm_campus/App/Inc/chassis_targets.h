#ifndef CHASSIS_TARGETS_H
#define CHASSIS_TARGETS_H
#include <stdbool.h>
/* 四轮顺序为电调 ID 1～4，单位为车轮输出轴 rpm，符号与 motor.velocity 一致。
 * 后续底盘控制每帧发布混控/限幅后的真实目标；当前零电流程序不发布虚假目标。 */
void Chassis_SetTargetWheelRpm(const float rpm[4]);
void Chassis_InvalidateTargets(void);
bool Chassis_CopyTargetWheelRpm(float rpm[4]);
#endif
