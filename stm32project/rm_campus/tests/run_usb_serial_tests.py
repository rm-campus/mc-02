"""USB BSP 内部回归检查：模拟类驱动事件，不连接串口、不需要 pyserial。"""
import argparse
from pathlib import Path
import shutil
import subprocess
import tempfile

STUB = r'''
#ifndef USB_CDC_STUB_H
#define USB_CDC_STUB_H
#include <stdint.h>
#include <stddef.h>
#define UNUSED(x) ((void)(x))
#define USBD_OK 0U
#define USBD_BUSY 1U
#define USBD_FAIL 2U
#define USBD_STATE_CONFIGURED 3U
#define USBD_STATE_SUSPENDED 4U
#define USBD_SPEED_HIGH 0U
#define USBD_SPEED_FULL 1U
#define CDC_DATA_HS_MAX_PACKET_SIZE 512U
#define CDC_DATA_FS_MAX_PACKET_SIZE 64U
#define CDC_SEND_ENCAPSULATED_COMMAND 0x00U
#define CDC_GET_ENCAPSULATED_RESPONSE 0x01U
#define CDC_SET_COMM_FEATURE 0x02U
#define CDC_GET_COMM_FEATURE 0x03U
#define CDC_CLEAR_COMM_FEATURE 0x04U
#define CDC_SET_LINE_CODING 0x20U
#define CDC_GET_LINE_CODING 0x21U
#define CDC_SET_CONTROL_LINE_STATE 0x22U
#define CDC_SEND_BREAK 0x23U
typedef struct { void *pClassData; uint8_t dev_state, dev_speed; } USBD_HandleTypeDef;
typedef struct {
    uint8_t *TxBuffer, *RxBuffer;
    uint32_t TxLength, TxState;
} USBD_CDC_HandleTypeDef;
typedef struct {
    int8_t (*Init)(void);
    int8_t (*DeInit)(void);
    int8_t (*Control)(uint8_t, uint8_t *, uint16_t);
    int8_t (*Receive)(uint8_t *, uint32_t *);
    int8_t (*TransmitCplt)(uint8_t *, uint32_t *, uint8_t);
} USBD_CDC_ItfTypeDef;
uint8_t USBD_CDC_SetTxBuffer(USBD_HandleTypeDef *, uint8_t *, uint32_t);
uint8_t USBD_CDC_SetRxBuffer(USBD_HandleTypeDef *, uint8_t *);
uint8_t USBD_CDC_TransmitPacket(USBD_HandleTypeDef *);
uint8_t USBD_CDC_ReceivePacket(USBD_HandleTypeDef *);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t);
#endif
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", default=shutil.which("gcc") or
                        r"E:\CLion\CLion 2026.2.2\bin\mingw\bin\gcc.exe")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    build = root / "build"
    build.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="usb-serial-test-", dir=build) as temporary:
        folder = Path(temporary)
        (folder / "usbd_cdc.h").write_text(STUB, encoding="utf-8")
        (folder / "usb_device.h").write_text(
            '#include "usbd_cdc.h"\nextern USBD_HandleTypeDef hUsbDeviceHS;\n', encoding="utf-8")
        (folder / "cmsis_os2.h").write_text(
            '#include <stdint.h>\nint32_t osDelay(uint32_t ticks);\n', encoding="utf-8")
        (folder / "FreeRTOS.h").write_text("#pragma once\n", encoding="utf-8")
        (folder / "task.h").write_text(
            'void test_enter_critical(void);\nvoid test_exit_critical(void);\n'
            '#define taskENTER_CRITICAL() test_enter_critical()\n'
            '#define taskEXIT_CRITICAL() test_exit_critical()\n', encoding="utf-8")
        exe = folder / "usb_serial_tests.exe"
        subprocess.run([
            args.cc, "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror", "-DMOTOR_DEBUG_TELEMETRY=0",
            "-I" + str(folder),
            "-I" + str(root / "Bsp/Inc"), "-I" + str(root / "App/Inc"),
            "-I" + str(root / "USB_DEVICE/App"),
            str(root / "tests/usb_serial_test.c"), str(root / "Bsp/Src/usb_serial.c"),
            str(root / "App/Src/usb_app.c"), str(root / "USB_DEVICE/App/usbd_cdc_if.c"),
            str(root / "App/Src/motor_debug_usb.c"),
            "-o", str(exe)], check=True, timeout=30)
        subprocess.run([str(exe)], check=True, timeout=10)


if __name__ == "__main__":
    main()
