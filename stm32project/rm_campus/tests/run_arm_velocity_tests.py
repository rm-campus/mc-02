"""真实舵机/UART/USB/速度控制器集成测试；所有关节标定均为模拟数据，不操作硬件。"""
from pathlib import Path
import argparse, shutil, subprocess, tempfile
from run_servo_tests import UART, CMSIS

TEST_CALIBRATION = r'''
/* 仅用于测试：P1500对应肩0、肘90、腕-90、滚0、爪0，禁止复制到实物。 */
#define ARM_JOINT_CALIBRATION_TABLE \
    {3U,1,1.0f,1500U,0.0f,-2.0f,2.0f,1U}, \
    {2U,1,1.0f,1500U,1.5707963268f,-1.0f,4.0f,1U}, \
    {1U,1,1.0f,1500U,-1.5707963268f,-4.0f,1.0f,1U}, \
    {4U,1,1.0f,1500U,0.0f,-2.0f,2.0f,1U}, \
    {5U,1,1.0f,1500U,0.0f,-2.0f,2.0f,1U}
'''

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cc',default=shutil.which('gcc') or r'E:\CLion\CLion 2026.2.2\bin\mingw\bin\gcc.exe')
    args=parser.parse_args();root=Path(__file__).resolve().parents[1];(root/'build').mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='arm-velocity-test-',dir=root/'build') as temporary:
        folder=Path(temporary)
        stubs={'usart.h':UART,'main.h':'#pragma once\n#include "usart.h"\n','cmsis_os2.h':CMSIS,'FreeRTOS.h':'#pragma once\n',
            'task.h':'#pragma once\nvoid test_enter(void);\nvoid test_exit(void);\n#define taskENTER_CRITICAL() test_enter()\n#define taskEXIT_CRITICAL() test_exit()\n'}
        for name,data in stubs.items():(folder/name).write_text(data,encoding='utf-8')
        calibration=folder/'simulation_calibration.h';calibration.write_text(TEST_CALIBRATION,encoding='utf-8')
        sources=['tests/arm_velocity_test.c','Bsp/Src/zp_servo.c','Bsp/Src/servo_uart.c','App/Src/servo_app.c',
            'App/Src/servo_rc.c','App/Src/servo_rc_usb.c','App/Src/servo_usb.c','App/Src/flash_tasks.c','App/Src/usb_app.c',
            'App/Src/motor_debug_usb.c','App/Src/arm_velocity.c','App/Src/arm_usb.c','Algorithm/Src/arm_kinematics.c']
        base=[args.cc,'-std=c11','-O2','-Wall','-Wextra','-Werror','-I'+str(folder),'-I'+str(root/'App/Inc'),
              '-I'+str(root/'Bsp/Inc'),'-I'+str(root/'Algorithm/Inc')]
        # 各速度故障之后启动全新进程；只有静态区重新初始化可解除复位锁存。
        scenarios=['startup','normal','stop_drain','linear','fresh_boot','joint','fresh_boot','gripper','fresh_boot','unreachable','timeout','feedback','tx_failure','manual_halt',
                   'mode','tracking','mechanical_limit','invalid_timing','atomic','reports','wrap']
        for label,enabled,output,valid,injected in [('enabled',1,1,1,True),('disabled',0,0,0,False),
                ('uncalibrated',1,1,0,True),('placeholder',1,1,1,False),('shadow',1,0,1,True)]:
            exe=folder/(label+'.exe')
            flags=[f'-DARM_VELOCITY_IK_ENABLE={enabled}',f'-DARM_VELOCITY_IK_OUTPUT_ENABLE={output}',
                   f'-DARM_JOINT_CALIBRATION_VALID={valid}','-DSERVO_RC_ENABLE=0']
            if injected:flags+=['-include',str(calibration)]
            subprocess.run([*base,*flags,*[str(root/s) for s in sources],'-lm','-o',str(exe)],check=True,timeout=40)
            for scenario in scenarios if label=='enabled' else ['configuration']:
                subprocess.run([str(exe),scenario],check=True,timeout=20)
        # 输出模式与原CH9控制不能同时启用，避免两个模块抢占同一组舵机。
        result=subprocess.run([*base,'-DARM_VELOCITY_IK_ENABLE=1','-DARM_VELOCITY_IK_OUTPUT_ENABLE=1',
            '-DSERVO_RC_ENABLE=1','-c',str(root/'App/Src/arm_velocity.c'),'-o',str(folder/'conflict.o')],capture_output=True,text=True)
        assert result.returncode!=0 and 'ARM output requires' in result.stderr
        print('PASS: ARM/RC physical-output compile-time exclusion.')

if __name__=='__main__':main()
