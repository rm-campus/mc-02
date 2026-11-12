"""Real CAN BSP + simulated FDCAN registers; no hardware access."""
import argparse
from pathlib import Path
import shutil
import subprocess
import tempfile

HAL = r'''
#pragma once
#include <stdint.h>
typedef struct {uint32_t CCCR,TXBRP;} FDCAN_GlobalTypeDef;
typedef struct {FDCAN_GlobalTypeDef *Instance; unsigned State;} FDCAN_HandleTypeDef;
typedef struct {uint32_t IdType,FilterIndex,FilterType,FilterConfig,FilterID1,FilterID2;} FDCAN_FilterTypeDef;
typedef struct {uint32_t Identifier,IdType,TxFrameType,DataLength,ErrorStateIndicator,BitRateSwitch,FDFormat,TxEventFifoControl,MessageMarker;} FDCAN_TxHeaderTypeDef;
#define HAL_OK 0
#define HAL_ERROR 1
#define FDCAN_CCCR_INIT 1U
#define FDCAN_STANDARD_ID 0U
#define FDCAN_FILTER_RANGE 1U
#define FDCAN_FILTER_TO_RXFIFO0 2U
#define FDCAN_REJECT 3U
#define FDCAN_REJECT_REMOTE 4U
#define FDCAN_IT_RX_FIFO0_NEW_MESSAGE 1U
#define FDCAN_IT_RX_FIFO0_MESSAGE_LOST 2U
#define FDCAN_IT_BUS_OFF 4U
#define FDCAN_RX_FIFO0 0U
#define FDCAN_DATA_FRAME 0U
#define FDCAN_ESI_ACTIVE 0U
#define FDCAN_BRS_OFF 0U
#define FDCAN_CLASSIC_CAN 0U
#define FDCAN_NO_TX_EVENTS 0U
int HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *,FDCAN_FilterTypeDef *);
int HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *,uint32_t,uint32_t,uint32_t,uint32_t);
int HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *,uint32_t,uint32_t);
int HAL_FDCAN_Start(FDCAN_HandleTypeDef *);
int HAL_FDCAN_Stop(FDCAN_HandleTypeDef *);
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *,uint32_t);
int HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *,FDCAN_TxHeaderTypeDef *,uint8_t *);
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *,uint32_t);
'''

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cc', default=shutil.which('gcc') or r'E:\CLion\CLion 2026.2.2\bin\mingw\bin\gcc.exe')
    args = p.parse_args()
    root = Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory(prefix='can-recovery-', dir=root/'build') as tmp:
        folder=Path(tmp)
        (folder/'fdcan.h').write_text(HAL, encoding='utf-8')
        (folder/'FreeRTOS.h').write_text('#pragma once\n', encoding='utf-8')
        (folder/'task.h').write_text('void enter(void);\nvoid leave(void);\n#define taskENTER_CRITICAL() enter()\n#define taskEXIT_CRITICAL() leave()\n', encoding='utf-8')
        exe=folder/'recovery.exe'
        subprocess.run([args.cc,'-std=c11','-O2','-Wall','-Wextra','-Werror',
            '-I'+str(folder),'-I'+str(root/'Bsp/Inc'),'-I'+str(root/'App/Inc'),
            str(root/'tests/can_recovery_test.c'),str(root/'Bsp/Src/can.c'),
            str(root/'App/Src/motor_debug.c'),'-lm','-o',str(exe)],check=True,timeout=30)
        subprocess.run([str(exe)],check=True,timeout=10)

if __name__=='__main__':
    main()
