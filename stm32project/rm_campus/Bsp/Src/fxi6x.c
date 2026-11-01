/**
 * @file fxi6x.c
 * @brief UART5 循环 DMA + 软件环形缓冲区 + 任务级 SBUS 解包。
 * 数据流：iA6B -> DMA 缓冲区 -> IRQ 搬运 -> s_ring -> RcTask 组帧 -> s_data。
 * DMA 缓冲区由外设写入；s_ring 是中断单生产者、RcTask 单消费者队列。
 * UART5 与 DMA IRQ 都是优先级 5，不互相抢占；更改优先级时需重新检查并发假设。
 * 这里只解析协议与上报状态，不进行底盘、夹爪或电机控制。
 */
#include "fxi6x.h"
#include "usart.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

/* DMA 持续循环写入 256 字节；半满/全满事件保证持续数据流也能被及时取走。 */
#define DMA_SIZE 256U
/* 软件缓冲区使用空出一格区分满/空，有效容量为 511 字节。 */
#define RING_SIZE 512U
/* SBUS 固定帧长；UART 的 8E2 是字节传输格式，与 25 字节帧结构不同。 */
#define FRAME_SIZE 25U

/* 专用链接段位于 RAM_D2，DMA1 无法访问默认的 DTCM。 */
static uint8_t s_dma[DMA_SIZE] __attribute__((section(".dma_rx"), aligned(32)));
static uint8_t s_ring[RING_SIZE];
/* head 是 IRQ 写入位置，tail 是任务消费位置；volatile 使每次访问观察真实索引，
 * 不代表整个数据结构自动具有原子性，字节发布顺序还依赖下面的 __DMB。 */
static volatile uint16_t s_head, s_tail;
/* 上一次已搬运的 DMA 位置；重复回调出现相同位置时不会重复复制字节。 */
static uint16_t s_dma_pos;
/* IRQ 只置异常标志；复杂恢复操作放到任务上下文完成。 */
static volatile bool s_restart, s_overflow;
static volatile uint32_t s_uart_errors, s_overflows;
/* 组帧状态跨 Update 调用保存，因此一帧分成多次接收也可以拼起来。 */
static uint8_t s_frame[FRAME_SIZE];
static uint8_t s_len;
static Fxi6x_Data s_data;
static osThreadId_t s_task;
/* 首次启动清空统计；后续错误恢复保留历史计数。 */
static bool s_initialized;

/**
 * @brief 通知 RcTask 缓冲区或异常状态有变化。
 * 在接收回调中调用；使用允许从 ISR 调用的线程标志置位接口。
 * 标志只负责唤醒，不携带字节，也不保证每一次 IRQ 都单独唤醒一次任务。
 */
static void notify_task(void)
{
    if (s_task != NULL)
    {
        (void)osThreadFlagsSet(s_task, FXI6X_RX_FLAG);
    }
}

/**
 * @brief 使当前操控数据归零并标记无效。
 * 开关置为 HIGH 枚举值，不是几何意义的中位；上层必须优先检查 valid。
 * 保留失控标志、接收历史和时间戳，以便监测任务区分失控、未收到和超时。
 */
static void clear_controls(void)
{
    s_data.rocker_l_h = s_data.rocker_l_v = 0.0f;
    s_data.rocker_r_h = s_data.rocker_r_v = 0.0f;
    s_data.knob_l = s_data.knob_r = 0.0f;
    s_data.switch_a = s_data.switch_b = FXI6X_HIGH;
    s_data.switch_c = s_data.switch_d = FXI6X_HIGH;
    s_data.valid = false;
}

/**
 * @brief 在 RcTask 内启动或重启 UART5 的接收到空闲循环 DMA。
 * @return true 表示本次启动接收成功；失败后由调用任务休眠重试。
 * 记录当前线程句柄供 IRQ 通知；不能从 main 的调度器启动前阶段调用。
 * Init/Update 只允许由一个接收任务调用，其他任务使用 CopyData。
 * 缓存维护范围为整个独占的 256 字节块，首地址与长度都满足 32 字节缓存行对齐。
 */
bool Fxi6x_Init(void)
{
    s_task = osThreadGetId();
    /* 先停旧接收再重建索引，避免旧 DMA 继续写入而软件已从头计数。 */
    (void)HAL_UART_AbortReceive(&huart5);
    taskENTER_CRITICAL();
    s_head = s_tail = s_dma_pos = 0U;
    s_len = 0U;
    s_restart = s_overflow = false;
    if (!s_initialized)
    {
        memset(&s_data, 0, sizeof(s_data));
        s_initialized = true;
    }
    clear_controls();
    taskEXIT_CRITICAL();
    /* 如以后启用 DCache，整个 DMA 缓冲区独占缓存行。 */
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U)
    {
        SCB_CleanInvalidateDCache_by_Addr((uint32_t *)s_dma, sizeof(s_dma));
    }
    return HAL_UARTEx_ReceiveToIdle_DMA(&huart5, s_dma, DMA_SIZE) == HAL_OK;
}

/* UART IDLE、DMA 半满和全满共用此入口；两路 NVIC 优先级均为 5。 */
/**
 * @brief IDLE、DMA 半满和全满触发的 HAL 接收事件回调。
 * @param size 本次事件报告的 DMA 写入位置，范围 0～256。
 * 回调仅搬运新增字节并通知任务；DMA 仍在循环写入，不能在这里重新启动同一接收。
 * 需要保持半满事件启用，否则极端连续流可能写满一整圈而旧、新位置恰好相同。
 * 缓存失效后才读取 DMA 写过的区域，避免未来启用 DCache 后 CPU 读到旧缓存。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart != &huart5 || size > DMA_SIZE)
    {
        return;
    }
    /* size 是当前缓冲区位置，不是本次新增的字节数。
     * DMA 全满时 HAL 可传入 256，映射为位置 0；半满事件避免整圈数据被误认为无新增。 */
    uint16_t pos = (uint16_t)(size % DMA_SIZE);
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U)
    {
        SCB_InvalidateDCache_by_Addr((uint32_t *)s_dma, sizeof(s_dma));
    }
    /* 只复制从旧位置到新位置的新增字节，取模实现缓冲区末尾到开头的环绕。 */
    while (s_dma_pos != pos)
    {
        uint16_t next = (uint16_t)((s_head + 1U) % RING_SIZE);
        if (next == s_tail)
        {
            /* 缓冲区满时不覆盖尚未消费的数据。任务随后清空积压并重新组帧，
             * 此处推进 DMA 消费位置，避免反复复制同一批已经丢弃的字节。 */
            s_overflow = true;
            ++s_overflows;
            s_dma_pos = pos;
            break;
        }
        /* 先写入数据，经 __DMB 确认顺序后再发布 head，避免消费者看到旧字节。 */
        s_ring[s_head] = s_dma[s_dma_pos];
        __DMB();
        s_head = next;
        s_dma_pos = (uint16_t)((s_dma_pos + 1U) % DMA_SIZE);
    }
    notify_task();
}

/**
 * @brief UART 接收出现奇偶校验、帧格式、溢出等错误时记录并请求恢复。
 * 这里只置位和通知，不在 ISR 中执行 Abort、重启、延时或打印。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart5)
    {
        ++s_uart_errors;
        s_restart = true;
        notify_task();
    }
}

/**
 * @brief 从一帧 SBUS 中提取指定通道的 11 位无符号值，再减去中值 1024。
 * @param frame 至少有 25 字节且已通过头尾检查的帧；本驱动只传入通道索引 0～9。
 * @return 原始有符号值，理论范围 -1024～1023，尚未应用遥控器端点校准。
 */
static int16_t channel(const uint8_t *frame, uint8_t index)
{
    /* SBUS 通道连续按 11 位压紧，没有每通道独立的字节边界。
     * 从第 1 字节开始定位通道起始位，读取 3 字节足以覆盖任意跨字节的 11 位值。 */
    uint16_t bit = (uint16_t)index * 11U;
    uint8_t byte = (uint8_t)(1U + bit / 8U);
    uint32_t raw = frame[byte] | ((uint32_t)frame[byte + 1U] << 8U) |
                   ((uint32_t)frame[byte + 2U] << 16U);
    return (int16_t)((int32_t)((raw >> (bit % 8U)) & 0x7FFU) - 1024);
}

/**
 * @brief 按参考遥控端点换算为 -1～1 的浮点量。
 * 正端点 783、负端点 -784 不对称，因此两侧使用不同除数。
 * 调用前 decode 已检查范围；本函数不另做饱和限幅。
 */
static float normalize(int16_t value)
{
    return value >= 0 ? (float)value / 783.0f : (float)value / 784.0f;
}

/**
 * @brief 用 ±392 阈值将辅助通道分为三个软件档位。
 * 当前负值对应 HIGH，正值对应 LOW；需在实际遥控器上核对物理拨动方向。
 */
static uint8_t switch_value(int16_t value)
{
    return value < -392 ? FXI6X_HIGH : (value > 392 ? FXI6X_LOW : FXI6X_MIDDLE);
}

/**
 * @brief 解包已经收齐且帧头帧尾正确的 25 字节候选帧。
 * 先解析 10 个通道和接收机标志，再验证前 6 个模拟通道。
 * SBUS 没有在此提供额外的帧级 CRC，头尾和范围检查不能替代无线链路的状态标志。
 * @return true 表示本帧可作为控制输入，false 表示已经清零并判无效。
 */
static bool decode(void)
{
    int16_t ch[10];
    for (uint8_t i = 0U; i < 10U; ++i)
    {
        ch[i] = channel(s_frame, i);
    }
    /* 先保存失控信息；即使本帧不允许控制，也必须让上层看到失控状态。 */
    s_data.received = true;
    ++s_data.frame_count;
    s_data.frame_lost = (s_frame[23] & 0x04U) != 0U;
    s_data.failsafe = (s_frame[23] & 0x08U) != 0U;
    /* 本工程采取保守策略：单帧丢失或 failsafe 均使本次控制值无效。 */
    bool valid = !s_data.frame_lost && !s_data.failsafe;
    for (uint8_t i = 0U; i < 6U; ++i)
    {
        valid = valid && ch[i] >= -784 && ch[i] <= 783;
    }
    if (!valid)
    {
        clear_controls();
        return false;
    }
    s_data.rocker_r_h = normalize(ch[0]);
    s_data.rocker_r_v = normalize(ch[1]);
    s_data.rocker_l_v = normalize(ch[2]);
    s_data.rocker_l_h = normalize(ch[3]);
    s_data.knob_l = normalize(ch[4]);
    s_data.knob_r = normalize(ch[5]);
    s_data.switch_a = switch_value(ch[6]);
    s_data.switch_b = switch_value(ch[7]);
    s_data.switch_c = switch_value(ch[8]);
    s_data.switch_d = switch_value(ch[9]);
    /* 只有真正正常的帧才刷新在线时间，串口噪声和失控帧不能续命。 */
    s_data.last_valid_ms = HAL_GetTick();
    s_data.valid = true;
    return true;
}

/**
 * @brief 消费软件环形缓冲区，完成错误恢复、找帧头、拼帧和通道解析。
 * 只在 RcTask 中调用；一次最多处理 RING_SIZE 字节，执行时间有界。
 * 跨多次回调的半帧会保留；头尾失配后逐字节滑动重同步。
 * @return 本轮是否曾遇到正常帧；在线/失控状态须另外读取最新快照。
 */
bool Fxi6x_Update(void)
{
    /* UART 错误后由任务重启 DMA；重启失败则保留标志，下次唤醒继续重试。 */
    if (s_restart)
    {
        if (!Fxi6x_Init())
        {
            s_restart = true;
        }
        return false;
    }
    if (s_overflow)
    {
        taskENTER_CRITICAL();
        /* 清空所有积压字节，同时丢弃半帧，防止丢字节后继续拼成错误通道值。 */
        s_tail = s_head;
        s_len = 0U;
        s_overflow = false;
        clear_controls();
        taskEXIT_CRITICAL();
    }
    bool got = false;
    /* 每轮处理有界，避免高优先级任务因异常数据一直运行。 */
    for (uint16_t n = 0U; n < RING_SIZE && s_tail != s_head; ++n)
    {
        __DMB();
        uint8_t byte = s_ring[s_tail];
        s_tail = (uint16_t)((s_tail + 1U) % RING_SIZE);
        if (s_len == 0U && byte != 0x0FU)
        {
            continue;
        }
        s_frame[s_len++] = byte;
        if (s_len == FRAME_SIZE)
        {
            if (s_frame[0] == 0x0FU && s_frame[24] == 0x00U)
            {
                /* got 保留“本轮出现过正常帧”的信息；最终有效性仍由最新 s_data 决定，
                 * 例如先正常后失控时 got 仍可为 true，但 s_data.valid 已为 false。 */
                got = decode() || got;
                s_len = 0U;
            }
            else
            {
                /* 错位时滑动一字节，重新寻找帧头。 */
                memmove(s_frame, s_frame + 1, FRAME_SIZE - 1U);
                s_len = FRAME_SIZE - 1U;
                while (s_len > 0U && s_frame[0] != 0x0FU)
                {
                    memmove(s_frame, s_frame + 1, --s_len);
                }
            }
        }
    }
    return got;
}

/**
 * @brief 在任务上下文复制最新遥控状态及累计错误计数。
 * @param out 输出对象，NULL 时直接返回。
 * valid 只说明最近一次解包结果；完全断线时不会产生新帧，调用者必须检查 last_valid_ms。
 */
void Fxi6x_CopyData(Fxi6x_Data *out)
{
    if (out != NULL)
    {
        taskENTER_CRITICAL();
        *out = s_data;
        /* IRQ 维护的统计也在同一短临界区里复制，保持本次快照一致。 */
        out->overflow_count = s_overflows;
        out->uart_error_count = s_uart_errors;
        taskEXIT_CRITICAL();
    }
}
