"""Exercise CH9 servo control and telemetry with real app/UART/USB sources."""
from pathlib import Path
import argparse, subprocess, tempfile, shutil
from run_servo_tests import UART, CMSIS

def main():
    p=argparse.ArgumentParser();p.add_argument('--cc',default=shutil.which('gcc') or r'E:\CLion\CLion 2026.2.2\bin\mingw\bin\gcc.exe');a=p.parse_args()
    root=Path(__file__).resolve().parents[1];(root/'build').mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='servo-rc-test-',dir=root/'build') as temp:
        folder=Path(temp)
        stubs={'usart.h':UART,'main.h':'#pragma once\n#include "usart.h"\n','cmsis_os2.h':CMSIS,'FreeRTOS.h':'#pragma once\n',
               'task.h':'#pragma once\nvoid test_enter(void);\nvoid test_exit(void);\n#define taskENTER_CRITICAL() test_enter()\n#define taskEXIT_CRITICAL() test_exit()\n'}
        for name,data in stubs.items():(folder/name).write_text(data,encoding='utf-8')
        sources=['tests/servo_rc_test.c','Bsp/Src/zp_servo.c','Bsp/Src/servo_uart.c','App/Src/servo_app.c',
                 'App/Src/servo_rc.c','App/Src/servo_rc_usb.c','App/Src/servo_usb.c','App/Src/flash_tasks.c',
                 'App/Src/usb_app.c','App/Src/motor_debug_usb.c']
        for enabled,bus,telemetry in [(1,1,1),(1,1,0),(0,1,1),(1,0,1)]:
            exe=folder/f'rc{enabled}_bus{bus}_tx{telemetry}.exe'
            subprocess.run([a.cc,'-std=c11','-O2','-Wall','-Wextra','-Werror',f'-DSERVO_RC_ENABLE={enabled}',
                f'-DSERVO_BUS_ENABLE={bus}',f'-DSERVO_RC_TELEMETRY_ENABLE={telemetry}',
                '-I'+str(folder),'-I'+str(root/'App/Inc'),'-I'+str(root/'Bsp/Inc'),
                *[str(root/s) for s in sources],'-lm','-o',str(exe)],check=True,timeout=30)
            subprocess.run([str(exe)],check=True,timeout=20)
if __name__=='__main__':main()
