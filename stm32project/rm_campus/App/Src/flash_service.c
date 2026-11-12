#include "flash_service.h"
#include "flash_tasks_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stddef.h>

static osMessageQueueId_t s_queue;
static volatile FlashServiceState s_state = FLASH_SERVICE_STARTING;
static volatile W25Q64Result s_error;

static void service_task(void *argument)
{
    (void)argument;
    s_error = W25Q64_Init();
    s_state = s_error == W25Q64_OK ? FLASH_SERVICE_READY : FLASH_SERVICE_ERROR;
    for (;;)
    {
        FlashJob *job;
        if (osMessageQueueGet(s_queue, &job, NULL, osWaitForever) != osOK) continue;
        W25Q64Result r = s_state == FLASH_SERVICE_READY ? W25Q64_OK : s_error;
        if (r == W25Q64_OK)
        {
            if (job->operation == FLASH_READ) r = W25Q64_Read(job->address, job->data, job->length);
            else if (job->operation == FLASH_WRITE) r = W25Q64_Write(job->address, job->data, job->length);
            else r = W25Q64_EraseSector(job->address);
        }
        taskENTER_CRITICAL();
        job->result = r;
        job->pending = false;
        job->done = true;
        taskEXIT_CRITICAL();
        if (job->waiter != NULL) (void)osThreadFlagsSet(job->waiter, FLASH_APP_WAKE_FLAG);
    }
}

bool FlashService_Create(void)
{
    s_queue = osMessageQueueNew(4U, sizeof(FlashJob *), NULL);
    if (s_queue == NULL) return false;
    const osThreadAttr_t attrs = {.name = "FlashServiceTask", .stack_size = 2048U,
                                 .priority = osPriorityBelowNormal};
    return osThreadNew(service_task, NULL, &attrs) != NULL;
}

FlashServiceState FlashService_GetState(void) { return s_state; }
W25Q64Result FlashService_GetError(void) { return s_error; }

bool FlashService_Submit(FlashJob *job, FlashOperation op, uint32_t address, uint16_t length)
{
    if (job == NULL || s_queue == NULL || job->pending || job->done ||
        op > FLASH_ERASE || (op != FLASH_ERASE && (length == 0U || length > sizeof(job->data))))
        return false;
    job->operation = op; job->address = address; job->length = length;
    job->waiter = osThreadGetId();
    job->pending = true;
    if (osMessageQueuePut(s_queue, &job, 0U, 0U) != osOK)
    {
        job->pending = false;
        return false;
    }
    return true;
}

bool FlashService_TakeResult(FlashJob *job, W25Q64Result *result)
{
    taskENTER_CRITICAL();
    bool done = job->done;
    if (done) { *result = job->result; job->done = false; }
    taskEXIT_CRITICAL();
    return done;
}
