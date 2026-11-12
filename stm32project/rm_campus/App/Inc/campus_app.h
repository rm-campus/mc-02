/**
 * @file campus_app.h
 * @brief 校赛应用层接口：把底层驱动组织到现有 FreeRTOS 任务中。
 * 数据路径：UART5/DMA -> 遥控解析；FDCAN3 中断 -> 电机反馈；监测任务 -> LED。
 * 默认零电流；可配置单电机速度 PI 调试，未实现麦轮运动学或 UART 舵机协议。
 * 快照接口用于任务上下文；不要在高优先级中断里调用它们。
 */
#ifndef CAMPUS_APP_H
#define CAMPUS_APP_H
#include <stdbool.h>
#include <stdint.h>
#include "motor_3508.h"
#include "fxi6x.h"
#include "motor_debug.h"

typedef struct {
    MotorDebug control;
    MotorDebugInput input;
    uint8_t motor_id;
} CampusMotorDebugSnapshot;

typedef struct
{
    /* can_ready：初始化是否成功，不代表电机在线或总线无故障。
     * can_bus_off：最近一次监测到的 CAN 错误隔离状态；这里只报告，不自动恢复。
     * rc_online：已收到正常遥控帧、帧未失控且距离最后有效帧小于 100 ms。 */
    bool can_ready, can_bus_off, rc_online;
    /* bit0～bit3 分别对应 C620 ID 1～4；0x0F 表示四台电机近期均有反馈。 */
    uint8_t motor_online_mask;
    /* CAN 发送入队失败累计次数；入队成功本身不保证帧已获总线 ACK。 */
    uint32_t can_tx_failures;
    /* 任务最近运行的 HAL 毫秒时间戳；rc_heartbeat 不是最后收到遥控帧的时刻。 */
    uint32_t chassis_heartbeat_ms, rc_heartbeat_ms;
    /* 监测任务最近完成一次状态刷新的时刻，供调试或未来看门狗策略使用。 */
    uint32_t monitor_heartbeat_ms;
} CampusStatus;

/* 所有 MX_xxx_Init 完成后、调度器启动前调用一次。先清零电机对象，再启动 CAN。 */
void Campus_Init(void);
/* 将共享状态复制到调用方；内部使用短临界区，out == NULL 时不执行复制。 */
void Campus_CopyStatus(CampusStatus *out);
/* 复制四电机的一致性快照，避免复制过程中 CAN 中断改写某些字段。 */
void Campus_CopyMotors(Motor3508 *out);
/* 调试器可调用或通过 USB 文本查看该快照；仅底盘任务更新控制状态。 */
void Campus_CopyMotorDebug(CampusMotorDebugSnapshot *out);
/* 以下函数均为常驻任务主体，内部自带循环；入口参数当前未使用。 */
void Campus_RcTask(void *argument);
void Campus_ChassisTask(void *argument);
void Campus_MonitorTask(void *argument);
void Campus_StatusTask(void *argument);
#endif
