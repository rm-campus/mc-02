/** 串行执行 W25Q64 请求。业务任务提交 FlashJob，服务任务操作芯片并通知提交者。 */
#ifndef FLASH_SERVICE_H
#define FLASH_SERVICE_H
#include "w25q64.h"
#include "cmsis_os2.h"
#include <stdbool.h>
typedef enum { FLASH_READ, FLASH_WRITE, FLASH_ERASE } FlashOperation;
typedef enum { FLASH_SERVICE_STARTING, FLASH_SERVICE_READY, FLASH_SERVICE_ERROR } FlashServiceState;
typedef struct {
    uint8_t data[W25Q64_PAGE_SIZE];
    uint32_t address;
    uint16_t length;
    FlashOperation operation;
    volatile bool pending, done;
    W25Q64Result result;
    osThreadId_t waiter;
} FlashJob;
#define FLASH_APP_WAKE_FLAG 1U
bool FlashService_Create(void);
FlashServiceState FlashService_GetState(void);
W25Q64Result FlashService_GetError(void);
/** @param[in,out] job 调用方持有的任务对象，首次使用前清零；写入前填 data，读完成后从 data 取数。
 * @param op 读取、写入或擦除一个扇区。
 * @param address 芯片内的字节地址；擦除时传扇区首地址。
 * @param length 读写字节数，范围 1～sizeof(job->data)；擦除忽略此值。
 * @return true 表示请求已入队。队列存的是指针，取走完成结果前需保留 job 和 data。 */
bool FlashService_Submit(FlashJob *job, FlashOperation op, uint32_t address, uint16_t length);
/** job 和 result 均须有效。返回 true 时写出芯片操作结果并释放该 job 供复用；
 * false 表示当前没有可取结果，result 保持原值。 */
bool FlashService_TakeResult(FlashJob *job, W25Q64Result *result);
#endif
