"""Run the integrated remote link and mode selection regression."""
import argparse
from pathlib import Path
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument("--cc", default=shutil.which("gcc") or r"E:\CLion\CLion 2026.2.2\bin\mingw\bin\gcc.exe")
args = parser.parse_args()
with tempfile.TemporaryDirectory() as tmp:
    where = Path(tmp)
    (where / "FreeRTOS.h").write_text("#pragma once\n", encoding="utf-8")
    (where / "task.h").write_text(
        "#define taskENTER_CRITICAL() ((void)0)\n#define taskEXIT_CRITICAL() ((void)0)\n",
        encoding="utf-8",
    )
    exe = where / "robot_link.exe"
    subprocess.run([args.cc, "-std=c11", "-Wall", "-Wextra", "-Werror", "-DROBOT_CONTROL_ENABLE=1",
                    "-I", str(where), "-I", "App/Inc", "-I", "Bsp/Inc", "tests/robot_link_test.c",
                    "App/Src/robot_control.c", "-o", str(exe)], cwd=root, check=True)
    subprocess.run([str(exe)], check=True)
    # 保证改号、日志、手动舵机和台架调试不会在整机遥控中争用执行器。
    headers = '#include "flash_tasks_config.h"\n#include "arm_config.h"\n'
    compile_config = [args.cc, "-std=c11", "-I", "App/Inc", "-DROBOT_CONTROL_ENABLE=1",
                      "-x", "c", "-fsyntax-only"]
    subprocess.run([*compile_config, "-"], input=headers, text=True, cwd=root, check=True)
    for conflict in ("SERVO_ID_SETUP", "MOTOR_DEBUG_ENABLE", "SERVO_USB_DEBUG_ENABLE", "SERVO_RC_ENABLE",
                     "MOTOR_DEBUG_TELEMETRY", "ARM_VELOCITY_IK_ENABLE", "MOTOR_SPEED_LOG_TASK_ENABLE",
                     "FLASH_TEST_TASK_ENABLE"):
        rejected = subprocess.run([*compile_config, "-D" + conflict + "=1", "-"],
                                  input=headers, text=True, cwd=root, capture_output=True)
        if rejected.returncode == 0:
            raise RuntimeError(f"Incompatible robot profile accepted {conflict}")
