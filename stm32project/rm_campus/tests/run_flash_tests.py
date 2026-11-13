"""Flash 驱动与业务状态机主机测试：模拟 NOR 时序、USB 完成回调及遥控/电机数据。"""
from pathlib import Path
import argparse
import shutil
import subprocess
import tempfile

CMSIS = r'''
#pragma once
#include <stdint.h>
#include <stddef.h>
typedef void *osThreadId_t;
typedef void *osMessageQueueId_t;
typedef int32_t osStatus_t;
typedef struct { const char *name; uint32_t stack_size; int priority; } osThreadAttr_t;
#define osOK 0
#define osWaitForever 0xffffffffU
#define osFlagsWaitAny 0U
#define osPriorityLow 8
#define osPriorityBelowNormal 16
#define osPriorityNormal 24
osThreadId_t osThreadNew(void (*fn)(void *), void *, const osThreadAttr_t *);
osThreadId_t osThreadGetId(void);
uint32_t osThreadFlagsSet(osThreadId_t, uint32_t);
uint32_t osThreadFlagsWait(uint32_t, uint32_t, uint32_t);
int32_t osDelay(uint32_t);
'''
OSPI = r'''
#pragma once
#include "main.h"
typedef struct { int unused; } OSPI_HandleTypeDef;
extern OSPI_HandleTypeDef hospi2;
typedef struct {
 uint32_t OperationType, FlashId, Instruction, InstructionMode, InstructionSize,
 InstructionDtrMode, AddressMode, AddressSize, Address, AddressDtrMode,
 AlternateBytesMode, DataMode, NbData, DataDtrMode, DQSMode, SIOOMode;
} OSPI_RegularCmdTypeDef;
#define HAL_OK 0
#define HAL_ERROR 1
#define HAL_OSPI_OPTYPE_COMMON_CFG 0
#define HAL_OSPI_FLASH_ID_1 1
#define HAL_OSPI_INSTRUCTION_1_LINE 1
#define HAL_OSPI_INSTRUCTION_8_BITS 8
#define HAL_OSPI_INSTRUCTION_DTR_DISABLE 0
#define HAL_OSPI_ADDRESS_1_LINE 1
#define HAL_OSPI_ADDRESS_NONE 0
#define HAL_OSPI_ADDRESS_24_BITS 24
#define HAL_OSPI_ADDRESS_DTR_DISABLE 0
#define HAL_OSPI_ALTERNATE_BYTES_NONE 0
#define HAL_OSPI_DATA_1_LINE 1
#define HAL_OSPI_DATA_NONE 0
#define HAL_OSPI_DATA_DTR_DISABLE 0
#define HAL_OSPI_DQS_DISABLE 0
#define HAL_OSPI_SIOO_INST_EVERY_CMD 0
int HAL_OSPI_Command(OSPI_HandleTypeDef *, OSPI_RegularCmdTypeDef *, uint32_t);
int HAL_OSPI_Receive(OSPI_HandleTypeDef *, uint8_t *, uint32_t);
int HAL_OSPI_Transmit(OSPI_HandleTypeDef *, uint8_t *, uint32_t);
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cc', default=shutil.which('gcc') or r'E:\CLion\CLion 2026.2.2\bin\mingw\bin\gcc.exe')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    (root / 'build').mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='flash-tests-', dir=root / 'build') as directory:
        folder = Path(directory)
        stubs = {'cmsis_os2.h': CMSIS, 'octospi.h': OSPI,
                 'main.h': '#pragma once\n#include <stdint.h>\nuint32_t HAL_GetTick(void);\nvoid Error_Handler(void);\n',
                 'FreeRTOS.h': '#pragma once\n',
                 'task.h': '#pragma once\n#define taskENTER_CRITICAL() ((void)0)\n#define taskEXIT_CRITICAL() ((void)0)\n'}
        for name, data in stubs.items():
            (folder / name).write_text(data, encoding='utf-8')
        common = [args.cc, '-std=c11', '-O2', '-Wall', '-Wextra', '-Werror',
                  '-I' + str(folder), '-I' + str(root / 'App/Inc'), '-I' + str(root / 'Bsp/Inc')]
        incompatible = subprocess.run(common + ['-DMOTOR_SPEED_LOG_TASK_ENABLE=1', '-DMOTOR_DEBUG_MODE=0',
                                               '-x', 'c', '-fsyntax-only', '-'],
                                      input='#include "flash_tasks_config.h"\n', text=True,
                                      capture_output=True, timeout=30)
        assert incompatible.returncode != 0 and 'requires four-motor mode' in incompatible.stderr
        print('PASS: single-motor mode is rejected when Flash logging is enabled.')
        driver = folder / 'driver.exe'
        subprocess.run(common + [str(root / 'tests/w25q64_test.c'), str(root / 'Bsp/Src/w25q64.c'),
                                  '-o', str(driver)], check=True, timeout=30)
        subprocess.run([str(driver)], check=True, timeout=20)
        for test_on, log_on in ((1, 1), (1, 0), (0, 1), (0, 0)):
            exe = folder / f'app_{test_on}_{log_on}.exe'
            sources = ['tests/flash_app_test.c', 'App/Src/flash_tasks.c', 'App/Src/flash_test.c',
                       'App/Src/motor_speed_log.c']
            subprocess.run(common + [f'-DFLASH_TEST_TASK_ENABLE={test_on}',
                                      f'-DMOTOR_SPEED_LOG_TASK_ENABLE={log_on}', '-DFLASH_LOG_END=0xA000UL'] +
                           [str(root / name) for name in sources] + ['-lm', '-o', str(exe)], check=True, timeout=30)
            scenarios = ['switches']
            if test_on:
                scenarios += ['echo', 'echo_reconnect', 'parser']
            if log_on:
                scenarios += ['log', 'dynamic', 'start_conditions', 'nonzero_boot', 'reconnect', 'recovery',
                              'incomplete', 'full', 'tick_wrap', 'rc_loss', 'late', 'enqueue_race',
                              'local_fault', 'fault_not_reached', 'control_stale', 'switch_off', 'all_stopped',
                              'invalid_target', 'overflow', 'legacy', 'erase_recovery',
                              'cadence100', 'old_period', 'old_erase_recovery', 'bad_period']
            for scenario in scenarios:
                subprocess.run([str(exe), scenario], check=True, timeout=20)


if __name__ == '__main__':
    main()
