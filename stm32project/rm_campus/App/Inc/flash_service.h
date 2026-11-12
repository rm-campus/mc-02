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
/* job 及其 data 必须在完成前一直有效且不得改写；每个业务使用独立静态 job。 */
bool FlashService_Submit(FlashJob *job, FlashOperation op, uint32_t address, uint16_t length);
bool FlashService_TakeResult(FlashJob *job, W25Q64Result *result);
#endif
