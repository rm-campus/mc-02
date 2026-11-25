#ifndef ROBOT_SERVO_H
#define ROBOT_SERVO_H
#include <stdbool.h>
#include <stdint.h>
void RobotServo_Process(uint32_t now);
/* 总线将要启动发送时再核对一遍，避免被 RcTask 抢占期间发送上一档的排队目标。 */
bool RobotServo_CanTransmit(uint8_t id, uint32_t now);
#endif
