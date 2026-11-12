/**
 * @file motor_3508.c
 * @brief M3508 + C620 电流控制实现。
 *
 * 换算关系：
 *   电流(A)   = 原始电流 × (20 / 16384) × 方向
 *   转速(rad/s) = 转子 rpm × (2π / 60) / 减速比 × 方向
 *   角度(rad) = ((圈数 × 8192) + 编码器) × (2π / 8192) / 减速比 × 方向
 */

#include "motor_3508.h"

#include "can.h"

#include <stddef.h>

/* C620 反馈电流换算系数：20 A 对应 16384。 */
#define MOTOR3508_CURRENT_TO_AMP (20.0f / 16384.0f)

/* M3508 减速比。 */
#define MOTOR3508_GEAR_RATIO (3591.0f / 187.0f)

/* 圆周常数。 */
#define MOTOR3508_TWO_PI 6.283185307179586f

/* 编码器一圈计数。 */
#define MOTOR3508_ENCODER_MAX 8192

/* 编码器半圈计数，用于跨圈判断。 */
#define MOTOR3508_ENCODER_HALF 4096

void Motor3508_Init(Motor3508 *motor, uint8_t reversed)
{
    if (motor == NULL)
    {
        return;
    }

    for (uint8_t slot = 0U; slot < 4U; ++slot)
    {
        /* 控制量与反馈量清零。 */
        motor->command[slot] = 0;
        motor->position[slot] = 0.0f;
        motor->velocity[slot] = 0.0f;
        motor->rotor_rpm[slot] = 0;
        motor->current[slot] = 0.0f;
        motor->temperature[slot] = 0U;
        motor->last_feedback_ms[slot] = 0U;

        /* 编码器初值使用 0xFFFF，首帧到达后写入实际值。 */
        motor->encoder[slot] = 0xFFFFU;

        /* 多圈计数清零。 */
        motor->round[slot] = 0;
    }

    motor->reversed = reversed;
    motor->received_mask = 0U;
}

void Motor3508_SetCurrent(Motor3508 *motor, uint8_t slot, int16_t current)
{
    if ((motor == NULL) || (slot >= 4U))
    {
        return;
    }

    /* 限制到 C620 原始电流范围。 */
    if (current > MOTOR3508_CURRENT_MAX)
    {
        current = MOTOR3508_CURRENT_MAX;
    }
    else if (current < -MOTOR3508_CURRENT_MAX)
    {
        current = -MOTOR3508_CURRENT_MAX;
    }

    /* 对应槽位的方向位为 1 时，控制量取反。 */
    if ((motor->reversed & (1U << slot)) != 0U)
    {
        current = (int16_t)-current;
    }

    motor->command[slot] = current;
}

bool Motor3508_Update(Motor3508 *motor,
                      uint32_t id,
                      const uint8_t *data,
                      uint8_t size)
{
    uint8_t slot;
    uint16_t encoder;
    int16_t speed_rpm;
    int16_t current_raw;
    int32_t difference;
    float direction;

    /* 反馈帧固定为 8 字节。 */
    if ((motor == NULL) || (data == NULL) || (size != 8U))
    {
        return false;
    }

    /* 本对象处理 0x201～0x204。 */
    if ((id < MOTOR3508_RX_ID) || (id >= (MOTOR3508_RX_ID + 4U)))
    {
        return false;
    }

    /* 反馈 ID 与槽位的对应关系：0x201 -> 0，0x204 -> 3。 */
    slot = (uint8_t)(id - MOTOR3508_RX_ID);

    /* 编码器按 16 位值读取，转速和电流按 int16 大端读取。 */
    encoder = (uint16_t)(((uint16_t)data[0] << 8U) | data[1]);
    /* 非法编码器值不更新在线时间，避免异常反馈让监测误判为正常在线。 */
    if (encoder >= MOTOR3508_ENCODER_MAX)
    {
        return false;
    }
    speed_rpm = (int16_t)(((uint16_t)data[2] << 8U) | data[3]);
    current_raw = (int16_t)(((uint16_t)data[4] << 8U) | data[5]);

    /* data[6] 为温度，稍后保存；data[7] 为保留字节。 */

    /*
     * 多圈计数：
     *   编码器从接近 8191 的值跳到接近 0 的值时，判定为正转跨圈，
     *   round 加一；
     *   编码器从接近 0 的值跳到接近 8191 的值时，判定为反转跨圈，
     *   round 减一。
     *
     * 首帧到达时建立基准，round 保持为 0。
     */
    if (motor->encoder[slot] != 0xFFFFU)
    {
        /* 半圈阈值法假定相邻有效反馈期间转子转过不足半圈。
         * 高速运行或长时间丢帧后，可能漏圈；此累计角度不是掉电保持的绝对位置。 */
        difference = (int32_t)encoder - (int32_t)motor->encoder[slot];

        if (difference < -MOTOR3508_ENCODER_HALF)
        {
            motor->round[slot]++;
        }
        else if (difference > MOTOR3508_ENCODER_HALF)
        {
            motor->round[slot]--;
        }
    }
    else
    {
        motor->round[slot] = 0;
    }

    motor->encoder[slot] = encoder;
    motor->rotor_rpm[slot] = speed_rpm;
    motor->temperature[slot] = data[6];
    /* 只有 ID、长度和编码器值合法的反馈才刷新时间戳和接收掩码。 */
    motor->last_feedback_ms[slot] = HAL_GetTick();
    motor->received_mask |= (uint8_t)(1U << slot);

    /* 方向位为 1 时取反。 */
    direction = ((motor->reversed & (1U << slot)) != 0U) ? -1.0f : 1.0f;

    /*
     * 输出轴累计角度：
     *   (round × 8192 + encoder) × 2π / 8192 / 减速比。
     */
    motor->position[slot] = direction *
        ((float)motor->round[slot] * (float)MOTOR3508_ENCODER_MAX +
         (float)encoder) *
        (MOTOR3508_TWO_PI / (float)MOTOR3508_ENCODER_MAX) /
        MOTOR3508_GEAR_RATIO;

    /*
     * 输出轴角速度：
     *   转子 rpm × 2π / 60 / 减速比。
     */
    motor->velocity[slot] = direction *
        (float)speed_rpm * (MOTOR3508_TWO_PI / 60.0f) /
        MOTOR3508_GEAR_RATIO;

    /*
     * 反馈电流：
     *   原始电流 × 20 A / 16384。
     */
    motor->current[slot] =
        direction * (float)current_raw * MOTOR3508_CURRENT_TO_AMP;
    return true;
}

bool Motor3508_Send(Motor3508 *motor)
{
    uint8_t data[8];

    if (motor == NULL)
    {
        return false;
    }

    /*
     * 按大端 int16 打包四个槽位的控制量：
     *   data[0..1] = 槽位 0
     *   data[2..3] = 槽位 1
     *   data[4..5] = 槽位 2
     *   data[6..7] = 槽位 3
     */
    for (uint8_t slot = 0U; slot < 4U; ++slot)
    {
        /* 转为无符号后取高低字节，保留负电流 int16 的补码位型。 */
        uint16_t value = (uint16_t)motor->command[slot];

        data[2U * slot] = (uint8_t)(value >> 8U);
        data[2U * slot + 1U] = (uint8_t)value;
    }

    return Can_Send(MOTOR3508_TX_ID, data, 8U);
}
