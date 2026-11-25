#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include "robot_control_config.h"
#include "fxi6x.h"

typedef enum { ROBOT_WAIT, ROBOT_DRIVE, ROBOT_ARM, ROBOT_WRIST } RobotMode;

typedef struct {
    RobotMode mode;
    bool ready;            /* 切档消抖期间链路仍可用，mode 暂为 WAIT。 */
    uint32_t generation;   /* 跨任务漏看短暂的退出/失联，也应撤销旧运动。 */
    uint32_t ready_count;  /* 仅在遥控重新就绪时递增，切档不触发蜂鸣。 */
    Fxi6x_Data rc;
} RobotControlState;

RobotControlState RobotControl_Get(uint32_t now);
#endif
