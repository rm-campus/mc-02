"""Fallback ARM Debug build for this CubeMX C-only template, without CMake execution.

Reads the existing plain source/include lists in CMakeLists.txt. MCU flags mirror
cmake/gcc-arm-none-eabi.cmake; update this recipe if the MCU/toolchain setup changes.
Does not flash or communicate with hardware.
"""
import argparse
from pathlib import Path
import concurrent.futures
import re
import subprocess
import sys

sys.stdout.reconfigure(encoding='utf-8')
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--toolchain-dir', required=True)
parser.add_argument('--build-dir', default='build/BenchDebug')
args = parser.parse_args()
root = Path(__file__).resolve().parents[1]
toolbin = Path(args.toolchain_dir)
gcc = toolbin / 'arm-none-eabi-gcc.exe'
if not gcc.is_file():
    parser.error(f'ARM GCC not found: {gcc}')
out = root / args.build_dir
out.mkdir(parents=True, exist_ok=True)
generated = (root / 'cmake/stm32cubemx/CMakeLists.txt').read_text(encoding='utf-8')
top = (root / 'CMakeLists.txt').read_text(encoding='utf-8')
def var(name):
    match = re.search(r'set\(' + name + r'\s+(.*?)\)', generated, re.S)
    if not match:
        raise ValueError(f'CMake source list changed: missing {name}')
    text = match.group(1)
    if '#' in text:
        raise ValueError(f'Unsupported CMake list syntax in {name}; review build recipe')
    return [s.replace('${CMAKE_CURRENT_SOURCE_DIR}/../../', '') for s in text.split()]
sources = sum([var(n) for n in ['MX_Application_Src','STM32_Drivers_Src','USB_Device_Library_Src','FreeRTOS_Src']], [])
sources += re.findall(r'^\s+([\w/]+\.c)\s*$', top, re.M)
sources = list(dict.fromkeys(sources))
includes = var('MX_Include_Dirs') + ['Bsp/Inc','App/Inc','Algorithm/Inc','Drivers/CMSIS/DSP/Include']
for path in [*sources, *includes, 'STM32H723XG_FLASH.ld']:
    if not (root/path).exists():
        raise FileNotFoundError(f'Build input missing: {path}')
flags = ['-std=gnu11','-mcpu=cortex-m7','-mfpu=fpv5-d16','-mfloat-abi=hard','-O0','-g3',
         '-Wall','-fdata-sections','-ffunction-sections','-DDEBUG','-DUSE_PWR_LDO_SUPPLY',
         '-DUSE_HAL_DRIVER','-DSTM32H723xx','-DARM_MATH_MATRIX_CHECK']
flags += ['-I' + str(root / p) for p in includes]
def compile_one(item):
    i, source = item
    obj = out / f'{i:03}.o'
    cmd = [str(gcc),*flags]
    if source.endswith('.s'): cmd += ['-x','assembler-with-cpp']
    cmd += ['-c',str(root/source),'-o',str(obj)]
    result = subprocess.run(cmd,capture_output=True,text=True,encoding='utf-8',errors='replace')
    if result.returncode: raise RuntimeError(source+'\n'+result.stdout+result.stderr)
    if result.stderr: print(result.stderr)
    return str(obj)
with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
    objects = list(pool.map(compile_one,enumerate(sources)))
subprocess.run([str(gcc),'-mcpu=cortex-m7','-mfpu=fpv5-d16','-mfloat-abi=hard',*objects,
                '-T',str(root/'STM32H723XG_FLASH.ld'),'--specs=nano.specs',
                '-Wl,--gc-sections','-Wl,--print-memory-usage','-Wl,-Map='+str(out/'rm_campus.map'),
                '-lm','-o',str(out/'rm_campus.elf')],check=True)
print(f'PASS: {len(sources)} ARM source files compiled and linked: {out / "rm_campus.elf"}')
