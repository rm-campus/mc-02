#include "chassis_targets.h"
#include "flash_tasks_config.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <string.h>
#include <stddef.h>

static float s_targets[4];
static uint32_t s_time;
static bool s_valid;

void Chassis_InvalidateTargets(void)
{
    taskENTER_CRITICAL();
    s_valid = false;
    taskEXIT_CRITICAL();
}

void Chassis_SetTargetWheelRpm(const float rpm[4])
{
    if (rpm == NULL) { Chassis_InvalidateTargets(); return; }
    for (unsigned i = 0; i < 4; ++i)
        if (!isfinite(rpm[i])) { Chassis_InvalidateTargets(); return; }
    taskENTER_CRITICAL();
    memcpy(s_targets, rpm, sizeof(s_targets));
    s_time = HAL_GetTick();
    s_valid = true;
    taskEXIT_CRITICAL();
}

bool Chassis_CopyTargetWheelRpm(float rpm[4])
{
    if (rpm == NULL) return false;
    taskENTER_CRITICAL();
    bool valid = s_valid && (uint32_t)(HAL_GetTick() - s_time) < MOTOR_LOG_TARGET_TIMEOUT_MS;
    if (valid) memcpy(rpm, s_targets, sizeof(s_targets));
    taskEXIT_CRITICAL();
    return valid;
}
