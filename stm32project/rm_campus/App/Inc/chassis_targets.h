#ifndef CHASSIS_TARGETS_H
#define CHASSIS_TARGETS_H
#include <stdbool.h>
/** 底盘目标快照接口，供任务发布和查询；当前四轮调试 PI 直接读取摇杆。
 * @param rpm 按电调 ID 1～4 排列的四个输出轴转速（rpm），正方向与 motor.velocity 一致。
 * 传 NULL 或任一元素为 NaN/无穷时使整组失效；四个零是有效的停车目标。 */
void Chassis_SetTargetWheelRpm(const float rpm[4]);
/* 撤销上次发布的整组目标，下次成功发布后恢复有效。 */
void Chassis_InvalidateTargets(void);
/** @param[out] rpm 调用方提供的四元素数组。
 * @return true 表示已复制有效目标；未发布、已撤销、到达超时阈值或空指针返回 false，保留原输出。 */
bool Chassis_CopyTargetWheelRpm(float rpm[4]);
#endif
