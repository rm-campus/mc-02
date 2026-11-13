/**
 * @file campus_app.c
 * @brief 应用层任务与驱动的接线位置。
 * CAN 回调只做读帧和反馈解析；遥控协议在 RcTask 中解析；LED 仅由 StatusTask 更新。
 * 所有 ms 时间戳来自 HAL_GetTick；任务等待使用 FreeRTOS tick，工程 tick 为 1 kHz。
 * 单电机调试启用后仍需遥控关闭档确认及回中。绿色不表示运动使能。
 */
#include "campus_app.h"
#include "can.h"
#include "led.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "motor_debug_config.h"
#include "mecanum.h"
#include <stddef.h>
#include <string.h>

/* 遥控超过此时间没有正常帧即离线；与任务心跳分别统计。 */
#define RC_TIMEOUT_MS 100U
/* 电机是否在线取决于反馈是否新鲜，不取决于是否曾经收到过反馈。 */
#define MOTOR_TIMEOUT_MS MOTOR_DEBUG_FEEDBACK_TIMEOUT_MS
/* command 由底盘任务写入；编码器、速度、电流和时间戳由 CAN 接收中断更新。 */
static Motor3508 s_motors;
static CampusStatus s_status;
static CampusMotorDebugSnapshot s_motor_debug;
static CampusCanDiagnostics s_can_diagnostics;

void Campus_CopyMotorStatus(CampusMotorStatusSnapshot *out)
{
    if (out == NULL) return;
    taskENTER_CRITICAL();
    out->now_ms = HAL_GetTick();
    out->motors = s_motors;
    out->status = s_status;
    out->debug = s_motor_debug;
    out->can = s_can_diagnostics;
    Can_CopyRecovery(&out->recovery);
    Fxi6x_CopyData(&out->rc);
    taskEXIT_CRITICAL();
}

/* PSR 读操作会影响部分事件字段，集中到控制任务，监测/USB 只读缓存。 */
static bool sample_can_status(void)
{
    FDCAN_ProtocolStatusTypeDef protocol = {0};
    FDCAN_ErrorCountersTypeDef errors = {0};
    bool valid = s_status.can_ready &&
        HAL_FDCAN_GetProtocolStatus(&hfdcan1, &protocol) == HAL_OK;
    bool counters_valid = s_status.can_ready &&
        HAL_FDCAN_GetErrorCounters(&hfdcan1, &errors) == HAL_OK;
    CanRecoveryStatus recovery;
    Can_CopyRecovery(&recovery);
    uint32_t now = HAL_GetTick();
    taskENTER_CRITICAL();
    s_can_diagnostics.status_valid = valid;
    s_can_diagnostics.counters_valid = counters_valid;
    s_can_diagnostics.sample_ms = now;
    s_can_diagnostics.bus_off = protocol.BusOff != 0U;
    s_can_diagnostics.passive = protocol.ErrorPassive != 0U;
    s_can_diagnostics.warning = protocol.Warning != 0U;
    s_can_diagnostics.tec = errors.TxErrorCnt;
    s_can_diagnostics.rec = errors.RxErrorCnt;
    /* FDCAN 在 Bus-Off 恢复期间也用 BIT0 表示观察到恢复用隐性位序列，
     * 这不是新的物理发送故障，不覆盖触发本次恢复的历史错误。 */
    bool recovery_progress = recovery.state == CAN_RECOVERY_WAIT_BUS && protocol.LastErrorCode == 5U;
    if (valid && !recovery_progress && protocol.LastErrorCode >= 1U && protocol.LastErrorCode <= 6U) {
        s_can_diagnostics.last_error = protocol.LastErrorCode;
        s_can_diagnostics.last_error_ms = now;
    }
    /* 查询失败也不能让 LED 报告总线健康。 */
    s_status.can_bus_off = !valid || protocol.BusOff != 0U;
    taskEXIT_CRITICAL();
    return valid && !protocol.BusOff;
}

void Campus_CopyMotorDebug(CampusMotorDebugSnapshot *out)
{
    if (out != NULL) {
        taskENTER_CRITICAL();
        *out = s_motor_debug;
        taskEXIT_CRITICAL();
    }
}

/**
 * @brief 初始化四台 M3508 的软件状态并启动 CAN 接收。
 * @note 调用前必须完成 MX_FDCAN1_Init；对象必须先于接收中断初始化。
 * MOTOR_REVERSED_MASK 按 C620 ID 修正电流与反馈方向，默认 0U；按实际安装标定。
 */
void Campus_Init(void)
{
    /* 必须先初始化对象，再开启可能触发反馈解析的 CAN 中断。 */
    memset(&s_status, 0, sizeof(s_status));
    memset(&s_can_diagnostics, 0, sizeof(s_can_diagnostics));
    memset(&s_motor_debug, 0, sizeof(s_motor_debug));
    s_motor_debug.motor_id = MOTOR_DEBUG_MOTOR_ID;
    MotorDebug_Init(&s_motor_debug.control);
    for (unsigned i = 0; i < 4U; ++i) MotorDebug_Init(&s_motor_debug.wheel[i].control);
    Motor3508_Init(&s_motors, MOTOR_REVERSED_MASK);
    s_status.can_ready = Can_Init(&hfdcan1);
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
 * @brief HAL 调用的 FDCAN 接收中断回调，当前仅处理 FDCAN1。
 * 一次回调取空 FIFO，按帧类型和长度筛选，再交给 Motor3508_Update 按 ID 解析。
 * 这里运行在中断上下文，不能执行任务延时、等待互斥量或长时间打印。
 * @param flags 含 RX FIFO0 丢失事件；计数是事件次数，不保证等于丢帧数。
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t flags)
{
    if (hfdcan != &hfdcan1)
    {
        return;
    }
    if ((flags & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != 0U) ++s_can_diagnostics.fifo_lost_events;
    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
    {
        FDCAN_RxHeaderTypeDef header;
        /* FDCAN 最大数据区为 64 字节，先给足读取空间，再检查是否为 8 字节经典帧。
         * 当前硬件配置为经典 CAN；此处避免未来修改外设模式后读帧覆盖栈。 */
        uint8_t data[64];
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, data) != HAL_OK)
        {
            ++s_can_diagnostics.read_fail;
            break;
        }
        if (header.IdType == FDCAN_STANDARD_ID &&
            header.RxFrameType == FDCAN_DATA_FRAME &&
            header.FDFormat == FDCAN_CLASSIC_CAN &&
            header.DataLength == FDCAN_DLC_BYTES_8 &&
            header.Identifier >= 0x201U && header.Identifier <= 0x204U)
        {
            uint32_t slot = header.Identifier - 0x201U;
            if (Motor3508_Update(&s_motors, header.Identifier, data, 8U)) {
                ++s_can_diagnostics.valid_frames[slot];
                Can_RecoveryFeedback((uint8_t)slot);
            }
            else ++s_can_diagnostics.bad_encoder[slot];
        }
        else ++s_can_diagnostics.bad_format;
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
 * @brief 1 ms 更新单电机或四个独立 PI，统一发送一个 0x200 电流帧。
 * 每轮独立启用/停机；仅遥控、调度或总线等公共事件会影响所有轮。
 */
void Campus_ChassisTask(void *argument)
{
    (void)argument;
    const MotorDebugConfig base = MotorDebug_DefaultConfig();
#if MOTOR_DEBUG_MODE == MOTOR_DEBUG_MODE_FOUR
    const uint8_t selected = 0U, active_mask = 0x0FU;
#else
    const uint8_t selected = base.motor_id >= 1U && base.motor_id <= 4U ? base.motor_id - 1U : 0U;
    const uint8_t active_mask = (uint8_t)(1U << selected);
#endif
    MotorDebugConfig configs[4];
    CampusMotorDebugSnapshot frame = {0};
    frame.active_mask = active_mask;
    frame.motor_id = base.motor_id;
    for (uint8_t i = 0; i < 4U; ++i) {
#if MOTOR_DEBUG_MODE == MOTOR_DEBUG_MODE_FOUR
        configs[i] = MotorDebug_WheelConfig(i + 1U);
#else
        configs[i] = base;
#endif
        MotorDebug_Init(&frame.wheel[i].control);
        if ((active_mask & (1U << i)) == 0U) frame.wheel[i].control.state = MOTOR_DEBUG_DISABLED;
    }
    TickType_t wake = xTaskGetTickCount();
    uint32_t previous_ms = HAL_GetTick();
    for (;;) {
        Fxi6x_Data rc;
        Motor3508 motors;
        Fxi6x_CopyData(&rc);
        Campus_CopyMotors(&motors);
        const bool can_ok = sample_can_status();
        const uint32_t now = HAL_GetTick();
        const bool tx_allowed = Can_ProcessRecovery(s_can_diagnostics.status_valid,
            s_can_diagnostics.bus_off, now, active_mask);
        const uint8_t recovery_mask = tx_allowed ? Can_RecoveryOutputMask() : 0U;
        const uint8_t previous_running = frame.running_mask;
        frame.running_mask = 0U;
        frame.common_target = (float)rc.rocker_l_v * (base.max_speed / 660.0f);
        float targets[4] = {frame.common_target, frame.common_target,
                            frame.common_target, frame.common_target};
#if MOTOR_DEBUG_MODE == MOTOR_DEBUG_MODE_FOUR
        /* CH8 每周期读取：-1 用平移目标，0/+1 用同速目标。使能仍由 CH7 决定。 */
        frame.mecanum_mode = rc.switch_b == FXI6X_LOW;
        if (frame.mecanum_mode) {
            Mecanum_Translate(rc.rocker_l_v, rc.rocker_l_h, base.deadband, base.max_speed, targets);
        }
#endif
        for (uint8_t i = 0; i < 4U; ++i) {
            MotorDebugInput *input = &frame.wheel[i].input;
            *input = (MotorDebugInput){
                .now_ms = now, .period_ms = (uint32_t)(now - previous_ms),
                .rc_valid = rc.valid, .rc_received = rc.received,
                .motor_received = (motors.received_mask & (1U << i)) != 0U,
                .can_ok = can_ok, .rc_ms = rc.last_valid_ms,
                .feedback_ms = motors.last_feedback_ms[i],
                .enable_switch = rc.switch_a, .stick = rc.rocker_l_v,
                .lateral_stick = frame.mecanum_mode ? rc.rocker_l_h : 0,
                .target_rad_s = targets[i],
                .speed = motors.velocity[i], .measured_current = motors.current[i],
                .temperature = motors.temperature[i]
            };
            if ((active_mask & (1U << i)) != 0U)
                MotorDebug_Step(&frame.wheel[i].control, &configs[i], input);
            /* bit i 对应电调 ID i+1。总线恢复后，仅允许已有新反馈的活动轮使用 PI 输出；
             * 其余槽位写零，循环结束后再把四个槽位打包到同一帧 0x200。 */
            int16_t command = (active_mask & recovery_mask & (1U << i)) != 0U ?
                frame.wheel[i].control.command_raw : 0;
            Motor3508_SetCurrent(&s_motors, i, command);
        }
        previous_ms = now;
        if (s_status.can_ready && tx_allowed && !Motor3508_Send(&s_motors)) {
            for (uint8_t i = 0; i < 4U; ++i) {
                if ((active_mask & (1U << i)) != 0U) MotorDebug_TxFailed(&frame.wheel[i].control);
#if MOTOR_DEBUG_CAN_FAULT_ENABLE
                Motor3508_SetCurrent(&s_motors, i, 0);
#endif
            }
            taskENTER_CRITICAL();
            ++s_status.can_tx_failures;
            taskEXIT_CRITICAL();
        }
        for (uint8_t i = 0; i < 4U; ++i)
            if (frame.wheel[i].control.state == MOTOR_DEBUG_RUNNING) frame.running_mask |= (uint8_t)(1U << i);
        if (previous_running == 0U && frame.running_mask != 0U) ++frame.run_id;
        /* 保留单电机快照接口；四轮诊断/回传使用 wheel[] 和 running_mask。 */
        frame.control = frame.wheel[selected].control;
        frame.input = frame.wheel[selected].input;
        taskENTER_CRITICAL();
        s_status.chassis_heartbeat_ms = now;
        s_motor_debug = frame;
        taskEXIT_CRITICAL();
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
        taskENTER_CRITICAL();
        /* uint32_t 时间差允许 HAL 毫秒计数器回绕；只在差值远小于回绕周期时使用。
         * 丢帧/失控导致 valid=false，完全断线则通过最后有效帧时间检测。 */
        s_status.rc_online = rc.received && rc.valid &&
            (uint32_t)(now - rc.last_valid_ms) < RC_TIMEOUT_MS;
        s_status.motor_online_mask = online;
        s_status.monitor_heartbeat_ms = now;
        taskEXIT_CRITICAL();
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(10U));
    }
}

/**
 * @brief 单独管理板载 RGB LED，避免多个任务同时使用静态发送缓冲区。
 * 状态优先级：CAN 故障（紫）> 遥控无效（红）> 电机不齐（黄）> 全在线（绿）。
 * @note 绿色仅表示通信在线，不表示调试器已启用。LED 发送失败目前未上报。
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
            (void)Led_SetRGB(0U, 20U, 0U);   /* 绿：遥控及四电机在线。 */
        }
        /* 状态指示不参与快速控制；每 50 tick 更新一次，给高优先级任务让出 CPU。 */
        osDelay(50U);
    }
}
