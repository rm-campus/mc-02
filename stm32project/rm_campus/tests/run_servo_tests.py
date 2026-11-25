"""Real servo protocol/UART7/task/USB code with simulated HAL TC, bus replies and USB ownership."""
from pathlib import Path
import argparse, subprocess, tempfile, shutil

UART = r'''
#pragma once
#include <stdint.h>
typedef struct { int id; } UART_HandleTypeDef;
extern UART_HandleTypeDef huart7, huart5;
#define HAL_OK 0
#define HAL_ERROR 1
#define UART_CLEAR_PEF 1
#define UART_CLEAR_FEF 2
#define UART_CLEAR_NEF 4
#define UART_CLEAR_OREF 8
#define UART_RXDATA_FLUSH_REQUEST 1
#define __HAL_UART_CLEAR_FLAG(h,f) ((void)(h),(void)(f))
#define __HAL_UART_SEND_REQ(h,f) ((void)(h),(void)(f))
#define __DMB() ((void)0)
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t);
uint32_t HAL_GetTick(void);
int HAL_UART_Abort(UART_HandleTypeDef *);
int HAL_UART_AbortReceive(UART_HandleTypeDef *);
int HAL_HalfDuplex_EnableReceiver(UART_HandleTypeDef *);
int HAL_HalfDuplex_EnableTransmitter(UART_HandleTypeDef *);
int HAL_UART_Receive_IT(UART_HandleTypeDef *,uint8_t *,uint16_t);
int HAL_UART_Transmit_IT(UART_HandleTypeDef *,const uint8_t *,uint16_t);
'''
CMSIS=r'''
#pragma once
#include <stdint.h>
typedef void *osThreadId_t;
#define osFlagsWaitAny 0
osThreadId_t osThreadGetId(void);
uint32_t osThreadFlagsSet(osThreadId_t,uint32_t);
uint32_t osThreadFlagsWait(uint32_t,uint32_t,uint32_t);
int32_t osDelay(uint32_t);
'''
def main():
    p=argparse.ArgumentParser();p.add_argument('--cc',default=shutil.which('gcc') or r'E:\CLion\CLion 2026.2.2\bin\mingw\bin\gcc.exe');a=p.parse_args()
    root=Path(__file__).resolve().parents[1];(root/'build').mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='servo-test-',dir=root/'build') as temp:
        folder=Path(temp)
        stubs={'usart.h':UART,'main.h':'#pragma once\n#include "usart.h"\n','cmsis_os2.h':CMSIS,'FreeRTOS.h':'#pragma once\n',
               'task.h':'#pragma once\nvoid test_enter(void);\nvoid test_exit(void);\n#define taskENTER_CRITICAL() test_enter()\n#define taskEXIT_CRITICAL() test_exit()\n'}
        for name,data in stubs.items():(folder/name).write_text(data,encoding='utf-8')
        sources=['tests/servo_test.c','Bsp/Src/zp_servo.c','Bsp/Src/servo_uart.c','App/Src/servo_app.c',
                 'App/Src/servo_usb.c','App/Src/flash_tasks.c','App/Src/usb_app.c','App/Src/motor_debug_usb.c']
        for enable,monitor,jog in ((0,0,0),(1,0,0),(1,1,0),(1,1,1)):
            exe=folder/f'servo{enable}_monitor{monitor}_jog{jog}.exe'
            test_sources=['tests/servo_jog_test.c',*sources[1:]] if jog else sources
            subprocess.run([a.cc,'-std=c11','-DSERVO_RC_ENABLE=0','-O2','-Wall','-Wextra','-Werror',f'-DSERVO_BUS_ENABLE={enable}',
                f'-DSERVO_STATUS_ONLY={monitor}',f'-DSERVO_JOG_TEST_ENABLE={jog}','-DSERVO_RX_TRACE_ENABLE=0',
                '-I'+str(folder),'-I'+str(root/'App/Inc'),'-I'+str(root/'Bsp/Inc'),
                *[str(root/s) for s in test_sources],'-lm','-o',str(exe)],check=True,timeout=30)
            for case in (['motions','modes','faults','session','queue'] if jog else [None]):
                subprocess.run([str(exe),*([case] if case else [])],check=True,timeout=20)
        exe=folder/'feedback.exe'
        subprocess.run([a.cc,'-std=c11','-O2','-Wall','-Wextra','-Werror','-DSERVO_STATUS_ONLY=1',
            '-I'+str(folder),'-I'+str(root/'App/Inc'),'-I'+str(root/'Bsp/Inc'),
            str(root/'tests/servo_feedback_test.c'),*[str(root/s) for s in sources[1:]],
            '-Wl,--wrap=ServoUart_TakeEvents','-lm','-o',str(exe)],check=True,timeout=30)
        for case in ('early','edge','missing_tc','retries'):
            subprocess.run([str(exe),case],check=True,timeout=20)
        exe=folder/'rx_trace.exe'
        subprocess.run([a.cc,'-std=c11','-O2','-Wall','-Wextra','-Werror','-DSERVO_STATUS_ONLY=1',
            '-I'+str(folder),'-I'+str(root/'App/Inc'),'-I'+str(root/'Bsp/Inc'),
            str(root/'tests/servo_rx_trace_test.c'),*[str(root/s) for s in sources[1:]],
            '-lm','-o',str(exe)],check=True,timeout=30)
        for case in ('raw','overflow','burst','reconnect','commands','pver15','pver30','pmod90','pmod110'):
            subprocess.run([str(exe),case],check=True,timeout=20)
if __name__=='__main__':main()
