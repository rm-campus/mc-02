"""Host regression for the single-motor bench controller; does not access hardware."""
import argparse
from pathlib import Path
import shutil
import subprocess
import tempfile

CAN_STUB = r'''
#ifndef TEST_CAN_H
#define TEST_CAN_H
#include <stdbool.h>
#include <stdint.h>
typedef struct {int unused;} FDCAN_HandleTypeDef;
typedef struct {uint32_t BusOff,ErrorPassive,Warning,LastErrorCode;} FDCAN_ProtocolStatusTypeDef;
typedef struct {uint32_t TxErrorCnt,RxErrorCnt;} FDCAN_ErrorCountersTypeDef;
typedef struct {uint32_t Identifier,IdType,RxFrameType,FDFormat,DataLength;} FDCAN_RxHeaderTypeDef;
typedef int HAL_StatusTypeDef;
#define HAL_OK 0
#define FDCAN_RX_FIFO0 0
#define FDCAN_STANDARD_ID 0
#define FDCAN_DATA_FRAME 0
#define FDCAN_CLASSIC_CAN 0
#define FDCAN_DLC_BYTES_8 8
#define FDCAN_IT_RX_FIFO0_MESSAGE_LOST 1U
extern FDCAN_HandleTypeDef hfdcan1;
uint32_t HAL_GetTick(void);
bool Can_Init(FDCAN_HandleTypeDef *);
bool Can_Send(uint32_t,const uint8_t *,uint8_t);
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(FDCAN_HandleTypeDef *,FDCAN_ProtocolStatusTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_GetErrorCounters(FDCAN_HandleTypeDef *,FDCAN_ErrorCountersTypeDef *);
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *,uint32_t);
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *,uint32_t,FDCAN_RxHeaderTypeDef *,uint8_t *);
#endif
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", default=shutil.which("gcc"))
    args = parser.parse_args()
    if not args.cc:
        parser.error("GCC not found: pass --cc path/to/gcc")
    root = Path(__file__).resolve().parents[1]
    build = root / "build"
    build.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="motor-debug-test-", dir=build) as temporary:
        for protection in (0, 1):
            exe = Path(temporary) / f"motor_debug_test_{protection}.exe"
            subprocess.run([
                args.cc, "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
                f"-DMOTOR_DEBUG_CAN_FAULT_ENABLE={protection}",
                "-I" + str(root / "App/Inc"),
                str(root / "tests/motor_debug_test.c"),
                str(root / "App/Src/motor_debug.c"), "-lm", "-o", str(exe)
            ], check=True, timeout=30)
            subprocess.run([str(exe)], check=True, timeout=10)

        folder = Path(temporary)
        stubs = {
            "can.h": CAN_STUB,
            "led.h": '#include <stdbool.h>\n#include <stdint.h>\nbool Led_Init(void);\nbool Led_SetRGB(uint8_t,uint8_t,uint8_t);\n',
            "FreeRTOS.h": '#pragma once\n#include <stdint.h>\ntypedef uint32_t TickType_t;\n#define pdMS_TO_TICKS(x) (x)\n',
            "task.h": '#include "FreeRTOS.h"\nvoid test_enter(void);\nvoid test_exit(void);\n'
                      '#define taskENTER_CRITICAL() test_enter()\n#define taskEXIT_CRITICAL() test_exit()\n'
                      'TickType_t xTaskGetTickCount(void);\nvoid vTaskDelayUntil(TickType_t *,TickType_t);\n',
            "cmsis_os2.h": '#include <stdint.h>\n#define osFlagsWaitAny 0\nint32_t osDelay(uint32_t);\nuint32_t osThreadFlagsWait(uint32_t,uint32_t,uint32_t);\n',
        }
        for name, content in stubs.items():
            (folder / name).write_text(content, encoding="utf-8")
        for enabled, protection in ((0, 0), (0, 1), (1, 0), (1, 1)):
            exe = folder / f"app_{enabled}_{protection}.exe"
            subprocess.run([
                args.cc, "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
                "-I" + str(folder), "-I" + str(root / "App/Inc"), "-I" + str(root / "Bsp/Inc"),
                f"-DMOTOR_DEBUG_ENABLE={enabled}", "-DMOTOR_DEBUG_MOTOR_ID=3", "-DMOTOR_DEBUG_KP=0.5f",
                "-DMOTOR_DEBUG_MODE=0",
                f"-DMOTOR_DEBUG_CAN_FAULT_ENABLE={protection}",
                str(root / "tests/motor_debug_app_test.c"), str(root / "App/Src/motor_debug.c"),
                str(root / "App/Src/campus_app.c"), str(root / "Bsp/Src/motor_3508.c"),
                "-lm", "-o", str(exe)
            ], check=True, timeout=30)
            subprocess.run([str(exe)], check=True, timeout=10)

        exe = folder / "telemetry.exe"
        subprocess.run([
            args.cc, "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
            "-DMOTOR_DEBUG_MODE=0",
            "-I" + str(folder),
            "-I" + str(root / "App/Inc"), "-I" + str(root / "Bsp/Inc"),
            str(root / "tests/motor_debug_usb_test.c"), str(root / "App/Src/motor_debug_usb.c"),
            "-lm", "-o", str(exe)
        ], check=True, timeout=30)
        subprocess.run([str(exe)], check=True, timeout=10)


        # Complete USB command path, independent of telemetry enable.
        (folder / "main.h").write_text('#pragma once\n', encoding="utf-8")
        with (folder / "cmsis_os2.h").open("a", encoding="utf-8") as f:
            f.write('typedef void *osThreadId_t;\n')
        for telemetry in (0, 1):
            exe = folder / f"status_{telemetry}.exe"
            subprocess.run([
                args.cc, "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
                "-I" + str(folder), "-I" + str(root / "App/Inc"), "-I" + str(root / "Bsp/Inc"),
                f"-DMOTOR_DEBUG_TELEMETRY={telemetry}",
                "-DMOTOR_DEBUG_MODE=0",
                *[str(root / p) for p in ["tests/motor_status_usb_test.c", "App/Src/motor_status_usb.c",
                    "App/Src/motor_debug_usb.c", "App/Src/flash_tasks.c", "App/Src/usb_app.c"]],
                "-lm", "-o", str(exe)
            ], check=True, timeout=30)
            subprocess.run([str(exe)], check=True, timeout=10)

        for enabled in (0, 1):
            exe = folder / f"four_app_{enabled}.exe"
            subprocess.run([
                args.cc, "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
                "-I" + str(folder), "-I" + str(root / "App/Inc"), "-I" + str(root / "Bsp/Inc"),
                "-DMOTOR_DEBUG_MODE=1", f"-DMOTOR_DEBUG_ENABLE={enabled}",
                "-DMOTOR_DEBUG_M1_KP=0.05f", "-DMOTOR_DEBUG_M2_KP=0.10f", "-DMOTOR_DEBUG_M3_KP=0.15f", "-DMOTOR_DEBUG_M4_KP=0.20f",
                "-DMOTOR_DEBUG_M1_KI=0.0f", "-DMOTOR_DEBUG_M2_KI=0.01f", "-DMOTOR_DEBUG_M3_KI=0.02f", "-DMOTOR_DEBUG_M4_KI=0.03f",
                *[str(root / p) for p in ["tests/four_motor_app_test.c", "App/Src/motor_debug.c",
                    "App/Src/campus_app.c", "App/Src/mecanum.c", "Bsp/Src/motor_3508.c"]], "-lm", "-o", str(exe)
            ], check=True, timeout=30)
            subprocess.run([str(exe)], check=True, timeout=10)
        exe = folder / "four_usb.exe"
        subprocess.run([
            args.cc, "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror", "-DMOTOR_DEBUG_MODE=1",
            "-I" + str(folder), "-I" + str(root / "App/Inc"), "-I" + str(root / "Bsp/Inc"),
            *[str(root / p) for p in ["tests/four_motor_usb_test.c", "App/Src/motor_status_usb.c",
                "App/Src/motor_debug_usb.c", "App/Src/flash_tasks.c", "App/Src/usb_app.c"]],
            "-lm", "-o", str(exe)
        ], check=True, timeout=30)
        subprocess.run([str(exe)], check=True, timeout=10)

if __name__ == "__main__":
    main()
