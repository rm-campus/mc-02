/**
 * @file fxi6x.h
 * @brief FS-i6X + iA6B 的 SBUS 接收接口。
 * UART5：100000 baud，9 位字长含偶校验位，2 停止位；板载电路已反相。
 * 驱动输出前 10 个通道，模拟量按参考端点映射到 -660～660 的整数。
 * 只有 RcTask 调用 Init/Update；其他任务通过 CopyData 获取快照。
 */
#ifndef FXI6X_H
#define FXI6X_H
#include <stdbool.h>
#include <stdint.h>

/* 这些是软件枚举值，不能直接视为舵机位置；开关物理方向应实测确认。 */
#define FXI6X_HIGH ((int8_t)1)
#define FXI6X_MIDDLE ((int8_t)0)
#define FXI6X_LOW ((int8_t)-1)
/* RcTask 的线程通知位；多个接收事件可能合并，数据依靠缓冲区保存。 */
#define FXI6X_RX_FLAG (1UL << 0)

typedef struct
{
    /* l/r 分别为左/右摇杆，h/v 为水平/垂直；整数值域 -660～660。
     * 协议 CH1/CH2 对应右摇杆，CH3/CH4 对应左摇杆，仍需核对遥控器设置。 */
    int16_t rocker_l_h, rocker_l_v, rocker_r_h, rocker_r_v;
    /* CH5/CH6 旋钮，映射方式与摇杆一致。 */
    int16_t knob_l, knob_r;
    /* CH7～CH10 对应四个辅助开关；HIGH/MIDDLE/LOW 分别为 1/0/-1。 */
    int8_t switch_a, switch_b, switch_c, switch_d;
    /* frame_lost/failsafe：最近结构正确帧的接收机标志。
     * valid：最近帧无丢帧/失控且模拟通道在有效范围；它不包含超时判断。
     * received：至少解析过一次帧头帧尾正确的候选帧，并不保证该帧可用于控制。 */
    bool frame_lost, failsafe, valid, received;
    /* last_valid_ms：最近正常控制帧的 HAL 毫秒时间戳；需与 valid 一起判断在线。
     * frame_count：结构正确候选帧计数，含携带失控标志的帧。
     * overflow_count：软件环形缓冲区满事件数，不是丢失字节数。
     * uart_error_count：UART 错误回调次数；计数在错误重启后保留，便于排查。 */
    uint32_t last_valid_ms, frame_count, overflow_count, uart_error_count;
} Fxi6x_Data;

/* 在 RcTask 中调用初始化和更新；不要同时启动单字节中断接收。 */
bool Fxi6x_Init(void);
/* 处理接收缓存与错误恢复；返回 true 仅表示本轮曾解析到正常帧。
 * 同一轮之后仍可能收到失控帧，因此最终控制决策应读取快照和时间戳。 */
bool Fxi6x_Update(void);
/* 任务上下文使用；out 由调用方分配，内部短临界区保护结构体复制。 */
void Fxi6x_CopyData(Fxi6x_Data *out);
#endif
