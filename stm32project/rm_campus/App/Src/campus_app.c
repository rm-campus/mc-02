/**
 * @file campus_app.c
 * @brief 应用层任务与驱动的接线位置。
 * CAN 回调只做读帧和反馈解析；遥控协议在 RcTask 中解析；LED 仅由 StatusTask 更新。
 * 所有 ms 时间戳来自 HAL_GetTick；任务等待使用 FreeRTOS tick，工程 tick 为 1 kHz。
 * 此版本的 ChassisTask 始终发送四路零电流，绿色 LED 也不表示已启用运动。
 */
#include "campus_app.h"
#include "can.h"
#include "led.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stddef.h>

/* 遥控超过此时间没有正常帧即离线；与任务心跳分别统计。 */
#define RC_TIMEOUT_MS 100U
/* 电机是否在线取决于反馈是否新鲜，不取决于是否曾经收到过反馈。 */
#define MOTOR_TIMEOUT_MS 50U
/* command 由底盘任务写入；编码器、速度、电流和时间戳由 CAN 接收中断更新。 */
static Motor3508 s_motors;
static CampusStatus s_status;

/**
 * @brief 初始化四台 M3508 的软件状态并启动 CAN 接收。
 * @note 调用前必须完成 MX_FDCAN3_Init；对象必须先于接收中断初始化。
 * 方向掩码 0U 表示暂不反向；轮序和实际正方向尚待上板标定。
 */
void Campus_Init(void)
{
    /* 必须先初始化对象，再开启可能触发反馈解析的 CAN 中断。 */
    Motor3508_Init(&s_motors, 0U);
    s_status.can_ready = Can_Init(&hfdcan3);
}

/**
 * @brief 在任务中获取四个电机字段的同一时段快照。
 * 临界区在当前工程中会屏蔽优先级数值 5 及以下紧迫性的 RTOS 兼容中断，
 * 因此优先级为 5 的 CAN 接收回调不会在结构体复制到一半时更新数据。
 * @param out 调用者提供的输出对象，允许为 NULL。
 */
void Campus_CopyMotors(Motor3508 *out)
{
    if (out != NULL)
    {
        taskENTER_CRITICAL();
        *out = s_motors;
        taskEXIT_CRITICAL();
    }
}

/**
 * @brief 复制应用状态供其他任务或调试代码使用。
 * @note 临界区内只有结构体复制，不等待外设、不延时、不调用阻塞接口。
 */
void Campus_CopyStatus(CampusStatus *out)
{
    if (out != NULL)
    {
        taskENTER_CRITICAL();
        *out = s_status;
        taskEXIT_CRITICAL();
    }
}

/**
 * @brief HAL 调用的 FDCAN 接收中断回调，当前仅处理 FDCAN3。
 * 一次回调取空 FIFO，按帧类型和长度筛选，再交给 Motor3508_Update 按 ID 解析。
 * 这里运行在中断上下文，不能执行任务延时、等待互斥量或长时间打印。
 * @param flags HAL 传入的事件标志，当前逻辑不逐项使用。
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t flags)
{
    (void)flags;
    if (hfdcan != &hfdcan3)
    {
        return;
    }
    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
    {
        FDCAN_RxHeaderTypeDef header;
        /* FDCAN 最大数据区为 64 字节，先给足读取空间，再检查是否为 8 字节经典帧。
         * 当前硬件配置为经典 CAN；此处避免未来修改外设模式后读帧覆盖栈。 */
        uint8_t data[64];
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, data) != HAL_OK)
        {
            break;
        }
        if (header.IdType == FDCAN_STANDARD_ID &&
            header.RxFrameType == FDCAN_DATA_FRAME &&
            header.FDFormat == FDCAN_CLASSIC_CAN &&
            header.DataLength == FDCAN_DLC_BYTES_8)
        {
            Motor3508_Update(&s_motors, header.Identifier, data, 8U);
        }
    }
}

/**
 * @brief 高优先级的遥控接收处理任务（低于有界的 IMU 采集/解算任务）。
 * 初始化失败时休眠后重试；正常时阻塞等待接收事件，不持续轮询占用 CPU。
 * 接收异常的重启和协议解析均由本任务处理，IRQ 只负责搬运字节并通知。
 */
void Campus_RcTask(void *argument)
{
    (void)argument;
    while (!Fxi6x_Init())
    {
        osDelay(100U);
    }
    for (;;)
    {
        /* IDLE/半满/全满事件唤醒，超时也处理错误恢复。 */
        /* 标志仅表示缓冲区可能有新字节，不能把一次唤醒当作恰好一帧。
         * 即便没有新数据，最多等待 10 tick 后也会执行串口错误恢复。 */
        (void)osThreadFlagsWait(FXI6X_RX_FLAG, osFlagsWaitAny, 10U);
        (void)Fxi6x_Update();
        taskENTER_CRITICAL();
        /* 这里记录任务活性；遥控在线判断另查 Fxi6x_Data.last_valid_ms。 */
        s_status.rc_heartbeat_ms = HAL_GetTick();
        taskEXIT_CRITICAL();
    }
}

/**
 * @brief 每 1 ms 发送四路零电流，验证 CAN 发送与电机反馈链路。
 * Motor3508_SetCurrent 的参数是协议原始电流值，0 表示不给驱动电流。
 * 当前不读取摇杆来驱动电机，后续接 PID 前应补充使能条件、轮序和方向标定。
 * can_tx_failures 统计提交 FIFO 失败，未统计物理总线的全部错误。
 */
void Campus_ChassisTask(void *argument)
{
    (void)argument;
    TickType_t wake = xTaskGetTickCount();
    for (;;)
    {
        /* 本次只接通驱动链路：四路每 1 ms 发送零电流。
         * 后续确认轮序、方向及 PID 参数后，再接入麦轮闭环目标。 */
        for (uint8_t slot = 0U; slot < 4U; ++slot)
        {
            Motor3508_SetCurrent(&s_motors, slot, 0);
        }
        if (s_status.can_ready && !Motor3508_Send(&s_motors))
        {
            taskENTER_CRITICAL();
            ++s_status.can_tx_failures;
            taskEXIT_CRITICAL();
        }
        taskENTER_CRITICAL();
        s_status.chassis_heartbeat_ms = HAL_GetTick();
        taskEXIT_CRITICAL();
        /* 按绝对唤醒时刻安排 1 ms 周期，避免把本轮计算时间叠加进周期。
         * 这不是硬件定时保证；后续 PID 接入后仍需测量执行时间和调度抖动。 */
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(1U));
    }
}

/**
 * @brief 每 10 ms 汇总遥控在线、电机在线和 CAN Bus-Off 状态。
 * 先在临界区内复制快照，再在临界区外计算，避免长时间屏蔽中断。
 * 监测结果只供状态指示/后续控制策略使用；本版本没有看门狗喂狗逻辑。
 */
void Campus_MonitorTask(void *argument)
{
    (void)argument;
    TickType_t wake = xTaskGetTickCount();
    for (;;)
    {
        Fxi6x_Data rc;
        Motor3508 motors;
        Fxi6x_CopyData(&rc);
        Campus_CopyMotors(&motors);
        uint32_t now = HAL_GetTick();
        /* received_mask 排除“启动时间小于超时阈值、但从未收到反馈”的误判。 */
        uint8_t online = 0U;
        for (uint8_t slot = 0U; slot < 4U; ++slot)
        {
            if ((motors.received_mask & (1U << slot)) != 0U &&
                (uint32_t)(now - motors.last_feedback_ms[slot]) < MOTOR_TIMEOUT_MS)
            {
                online |= (uint8_t)(1U << slot);
            }
        }
        FDCAN_ProtocolStatusTypeDef protocol = {0};
        /* 初始化成功后才查询控制器；Bus-Off 只更新状态，不在此处重启外设。 */
        bool bus_off = s_status.can_ready &&
            HAL_FDCAN_GetProtocolStatus(&hfdcan3, &protocol) == HAL_OK && protocol.BusOff;
        taskENTER_CRITICAL();
        /* uint32_t 时间差允许 HAL 毫秒计数器回绕；只在差值远小于回绕周期时使用。
         * 丢帧/失控导致 valid=false，完全断线则通过最后有效帧时间检测。 */
        s_status.rc_online = rc.received && rc.valid &&
            (uint32_t)(now - rc.last_valid_ms) < RC_TIMEOUT_MS;
        s_status.motor_online_mask = online;
        s_status.can_bus_off = bus_off;
        s_status.monitor_heartbeat_ms = now;
        taskEXIT_CRITICAL();
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(10U));
    }
}

/**
 * @brief 单独管理板载 RGB LED，避免多个任务同时使用静态发送缓冲区。
 * 状态优先级：CAN 故障（紫）> 遥控无效（红）> 电机不齐（黄）> 全在线（绿）。
 * @note 绿色仅表示通信在线；当前仍输出零电流。LED 发送失败返回值目前未上报。
 */
void Campus_StatusTask(void *argument)
{
    (void)argument;
    (void)Led_Init();
    for (;;)
    {
        CampusStatus status;
        Campus_CopyStatus(&status);
        if (!status.can_ready || status.can_bus_off)
        {
            (void)Led_SetRGB(16U, 0U, 16U);  /* 紫：CAN 初始化失败或 Bus-Off。 */
        }
        else if (!status.rc_online)
        {
            (void)Led_SetRGB(24U, 0U, 0U);   /* 红：无有效遥控或遥控失联。 */
        }
        else if (status.motor_online_mask != 0x0FU)
        {
            (void)Led_SetRGB(16U, 12U, 0U);  /* 黄：遥控正常，但电机未全部在线。 */
        }
        else
        {
            (void)Led_SetRGB(0U, 20U, 0U);   /* 绿：遥控及四电机在线，仍为零电流。 */
        }
        /* 状态指示不参与快速控制；每 50 tick 更新一次，给高优先级任务让出 CPU。 */
        osDelay(50U);
    }
}
