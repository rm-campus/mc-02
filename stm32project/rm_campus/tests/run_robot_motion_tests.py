"""Exercise individual and combined robot motion paths with the UART7 fixture."""
import argparse
from pathlib import Path
import shutil
import subprocess
import tempfile
from run_servo_tests import UART, CMSIS

root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument("--cc", default=shutil.which("gcc") or r"E:\CLion\CLion 2026.2.2\bin\mingw\bin\gcc.exe")
args = parser.parse_args()
sources = ["tests/robot_motion_test.c", "App/Src/robot_control.c", "App/Src/robot_servo.c",
           "App/Src/motor_debug.c", "App/Src/mecanum.c", "App/Src/servo_app.c",
           "Bsp/Src/servo_uart.c", "Bsp/Src/zp_servo.c", "Bsp/Src/fs_servo.c", "App/Src/usb_app.c",
           "App/Src/flash_tasks.c", "App/Src/motor_debug_usb.c"]
with tempfile.TemporaryDirectory() as d:
    directory = Path(d)
    for name, data in {"usart.h": UART, "cmsis_os2.h": CMSIS, "main.h": '#include "usart.h"\n',
                       "FreeRTOS.h": "#pragma once\n",
                       "task.h": "void test_enter(void);\nvoid test_exit(void);\n#define taskENTER_CRITICAL() test_enter()\n#define taskEXIT_CRITICAL() test_exit()\n"}.items():
        (directory/name).write_text(data, encoding="utf-8")
    for overrides in ([], ["-DROBOT_TRACK_M=0.30f", "-DROBOT_WHEELBASE_M=0.40f",
                           "-DROBOT_JOINT_SPEED_SCALE=3.0f"]):
        exe = directory/"motion.exe"
        subprocess.run([args.cc, "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
                        "-DROBOT_CONTROL_ENABLE=1", *overrides, "-I", str(directory), "-I", "App/Inc",
                        "-I", "Bsp/Inc", *sources, "-lm", "-o", str(exe)], check=True,cwd=root)
        subprocess.run([str(exe)], check=True)
        subprocess.run([str(exe), "sparse"], check=True)
        subprocess.run([str(exe), "wrist-silence"], check=True)
        subprocess.run([str(exe), "joint-lead-limit"], check=True)
        subprocess.run([str(exe), "joint-speed-limit"], check=True)
        for case in ("automatic-metadata-window", "automatic-metadata-window-wrist-start",
                     "neutral-frame", "first-stick-frame",
                     "released-stick-frame", "gripper-center-frame", "second-axis-frame"):
            subprocess.run([str(exe), case], check=True)
    subprocess.run([args.cc, "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
                    "-DROBOT_CONTROL_ENABLE=1", "-DSERVO_FASHIONSTAR_GRIPPER=1",
                    "-I", str(directory), "-I", "App/Inc", "-I", "Bsp/Inc",
                    *sources, "-lm", "-o", str(exe)], check=True, cwd=root)
    subprocess.run([str(exe), "fashionstar"], check=True)
    subprocess.run([str(exe), "joint-lead-limit"], check=True)
    for case in ("automatic-metadata-window", "neutral-frame", "first-stick-frame",
                 "released-stick-frame", "second-axis-frame"):
        subprocess.run([str(exe), case], check=True)
