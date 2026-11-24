"""主机验证腕参考点速度逆运动学；不访问任何板卡或舵机。"""
from pathlib import Path
import argparse
import shutil
import subprocess
import tempfile

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cc', default=shutil.which('gcc') or r'E:\CLion\CLion 2026.2.2\bin\mingw\bin\gcc.exe')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    (root / 'build').mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='arm-kinematics-', dir=root / 'build') as temporary:
        executable = Path(temporary) / 'arm_kinematics_test.exe'
        subprocess.run([args.cc, '-std=c11', '-O2', '-Wall', '-Wextra', '-Werror',
                        '-I' + str(root / 'Algorithm/Inc'),
                        str(root / 'tests/arm_kinematics_test.c'),
                        str(root / 'Algorithm/Src/arm_kinematics.c'), '-lm', '-o', str(executable)],
                       check=True, timeout=30)
        subprocess.run([str(executable)], check=True, timeout=15)

if __name__ == '__main__':
    main()
