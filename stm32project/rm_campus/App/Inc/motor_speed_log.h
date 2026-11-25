/** 四轮离线日志：从控制快照采样，经 FlashService 写入，收到 read 后从 Flash 回放。 */
#ifndef MOTOR_SPEED_LOG_H
#define MOTOR_SPEED_LOG_H
#include <stdbool.h>
#include <stdint.h>
typedef enum {
    MOTOR_LOG_BOOT, MOTOR_LOG_PREPARING, MOTOR_LOG_READY, MOTOR_LOG_RECORDING,
    MOTOR_LOG_FLUSHING, MOTOR_LOG_WAIT_READ, MOTOR_LOG_REPLAYING,
    MOTOR_LOG_ERASING, MOTOR_LOG_ERROR
} MotorLogState;
typedef struct {
    MotorLogState state;
    uint32_t records, error, stop_reason;
    uint32_t period_ms; /* 当前日志采样/回放周期；READY 时为新记录的默认周期。 */
    bool reached, target_valid;
} MotorLogStatus;
/* 创建日志任务。功能编译关闭时直接返回 true；启用时返回是否创建成功。 */
bool MotorLog_Create(void);
/* 推进一次采样、写入或回放状态机。now 为毫秒时间；采样判定使用快照自带的时间。 */
void MotorLog_Process(uint32_t now);
/* 仅在待回放、无冲突请求且取得 USB 输出权时受理。true 表示请求已交给日志任务。 */
bool MotorLog_RequestRead(void);
/* 请求擦除日志分区。等待回放、就绪或错误状态可受理，true 表示已受理，擦除异步执行。 */
bool MotorLog_RequestClear(void);
void MotorLog_GetStatus(MotorLogStatus *out);
#endif
