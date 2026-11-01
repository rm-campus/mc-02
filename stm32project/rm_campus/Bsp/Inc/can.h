/**
 * @file can.h
 * @brief FDCAN 经典帧收发接口。
 *
 * 硬件连接：
 *   FDCAN3_RX = PD12
 *   FDCAN3_TX = PD13
 *
 * 通信参数：
 *   经典帧，1 Mbps，标准 ID。
 *
 * 接收方式：
 *   Can_Init() 使能 RX FIFO0 新消息中断。
 *   应用层实现 HAL_FDCAN_RxFifo0Callback()，在回调中读取帧并分发。
 */

#ifndef CAN_H
#define CAN_H

#include <stdbool.h>
#include <stdint.h>

#include "fdcan.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 FDCAN 并启动 RX FIFO0 中断。
 *
 * @param hcan 已由 MX_FDCANx_Init() 初始化的 FDCAN 句柄，例如 &hfdcan3。
 *
 * @return true  过滤器配置、外设启动和中断使能全部成功。
 * @return false 句柄为空，或上述任一步骤返回 HAL_ERROR。
 *
 * @note 每个 FDCAN 句柄调用一次。调用前完成 MX_FDCANx_Init()。
 */
/* 本工程实现仅保存一个句柄，接收范围固定为 0x201～0x204；
 * StdFiltersNbr 必须至少为 1，RX FIFO0 与 TX FIFO 必须预先分配元素。 */
bool Can_Init(FDCAN_HandleTypeDef *hcan);

/**
 * @brief 发送一帧标准经典 CAN 数据帧。
 *
 * @param id   11 位标准 ID，范围 0x000～0x7FF。
 * @param data 数据缓冲区首地址。
 * @param size 数据长度，范围 0～8 字节。
 *
 * @return true  帧已写入 FDCAN 发送 FIFO。
 * @return false 句柄为空、参数超出范围，或发送 FIFO 写入返回 HAL_ERROR。
 *
 * @note 该函数完成入队后立即返回。总线发送由 FDCAN 外设完成。
 */
/* 当前由 ChassisTask 单一发送者使用；没有内部互斥或发送完成等待。 */
bool Can_Send(uint32_t id, const uint8_t *data, uint8_t size);

#ifdef __cplusplus
}
#endif

#endif /* CAN_H */
