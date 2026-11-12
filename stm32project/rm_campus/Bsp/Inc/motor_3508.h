/**
 * @file motor_3508.h
 * @brief M3508 + C620 电流控制接口。
 *
 * 控制帧：0x200
 * 反馈帧：0x201～0x204
 *
 * 一个 Motor3508 对象管理四个槽位。
 * 槽位号范围 0～3，对应反馈 ID 0x201～0x204。
 */

#ifndef MOTOR_3508_H
#define MOTOR_3508_H

#include <stdbool.h>
#include <stdint.h>

#include "dji_motor.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 控制帧 ID，对应槽位 0～3 的反馈 ID 为 0x201～0x204。 */
#define MOTOR3508_TX_ID 0x200U
#define MOTOR3508_RX_ID 0x201U

/* C620 原始电流范围。 */
#define MOTOR3508_CURRENT_MAX 16384

/* M3508 对象与公共电机结构使用同一组字段。 */
typedef DjiMotor Motor3508;

/**
 * @brief 初始化 M3508 对象。
 *
 * @param motor   电机对象。
 * @param reversed 方向位掩码，bit0～bit3 对应槽位 0～3。
 *                 对应位为 1 时，该槽位的控制量和反馈量取反。
 *
 * 初始化完成后：
 *   command、position、velocity、current 全部清零；
 *   encoder 写入初值 0xFFFF；
 *   round 清零。
 */
void Motor3508_Init(Motor3508 *motor, uint8_t reversed);

/**
 * @brief 设置指定槽位的原始电流。
 *
 * @param motor   电机对象。
 * @param slot    槽位号，范围 0～3。
 * @param current 原始电流，范围 -16384～+16384。
 *
 * 函数内部完成范围限制和方向处理，结果写入 command[slot]。
 */
/* current 是有符号原始指令，不是安培或转速；本函数只改缓存，不立即发送 CAN。
 * ±16384 是协议上限，不等于机构允许持续使用的电流；实际限幅由控制策略决定。 */
void Motor3508_SetCurrent(Motor3508 *motor, uint8_t slot, int16_t current);

/**
 * @brief 解析一帧 M3508 反馈。
 *
 * @param motor 电机对象。
 * @param id    CAN 帧 ID。
 * @param data  数据缓冲区，长度 8 字节。
 * @param size  数据长度。
 *
 * 函数处理 ID 为 0x201～0x204、长度为 8 的帧；
 * 其他帧直接返回 false。成功更新返回 true。该函数在 CAN 接收回调中调用。
 */
bool Motor3508_Update(Motor3508 *motor,
                      uint32_t id,
                      const uint8_t *data,
                      uint8_t size);

/**
 * @brief 发送 0x200 控制帧。
 *
 * @param motor 电机对象。
 *
 * @return true  帧已写入 CAN 发送 FIFO。
 * @return false 对象为空，或 CAN 发送返回 HAL_ERROR。
 *
 * 四个槽位的 command 按大端 int16 打包为 8 字节。
 */
/* 四个槽位共用一帧，应用应由一个任务统一组织，不能为每台电机分别覆盖发送。 */
bool Motor3508_Send(Motor3508 *motor);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_3508_H */
