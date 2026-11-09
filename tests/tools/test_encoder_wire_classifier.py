"""Encoder wire classifier truth table, compiled without hardware."""
import shutil
import subprocess
from pathlib import Path

from tools.precommit_checks import compiler_environment

SRC = Path(__file__).resolve().parents[2] / 'ESP32-S3/src'


def test_wire_classifier_truth_table(tmp_path):
    source = r'''
#include <cassert>
#include "hal/EncoderWireClassifier.h"

int main() {
    // 浮空/被拉死 = 无驱动；恒定角度或变化的数据 = 有驱动。
    assert(!wireDriven(0xFFFF, 0x0000));
    assert(!wireDriven(0x0000, 0x0000));
    assert(!wireDriven(0xFFFF, 0xFFFF));
    assert(wireDriven(0x1234, 0x1234));
    assert(wireDriven(0x1234, 0x1240));

    uint16_t up[4] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};
    uint16_t down[4] = {0x0000, 0x0000, 0x0000, 0x0000};
    assert(wireFingerChar(up, down, 4) == 'N');

    uint16_t zeroUp[2] = {0x0000, 0x0000};
    uint16_t zeroDown[2] = {0x0000, 0x0000};
    assert(wireFingerChar(zeroUp, zeroDown, 2) == 'S');

    uint16_t highUp[2] = {0xFFFF, 0xFFFF};
    uint16_t highDown[2] = {0xFFFF, 0xFFFF};
    assert(wireFingerChar(highUp, highDown, 2) == 'H');

    uint16_t goodUp[2] = {0x1234, 0xFFFF};
    uint16_t goodDown[2] = {0x1234, 0x0000};
    assert(wireFingerChar(goodUp, goodDown, 2) == 'O');
    return 0;
}
'''
    cpp = tmp_path / 'wire.cpp'
    cpp.write_text(source, encoding='utf-8')
    env = compiler_environment()
    compiler = next(path for name in ('g++', 'clang++', 'cl')
                    if (path := shutil.which(name, path=env.get('PATH'))))
    executable = tmp_path / 'wire.exe'
    if Path(compiler).name.lower() in ('cl', 'cl.exe'):
        args = [compiler, '/std:c++17', '/EHsc', '/utf-8', f'/I{SRC}', str(cpp),
                f'/Fo{tmp_path.resolve()}\\', f'/Fe{executable}']
    else:
        args = [compiler, '-std=c++17', f'-I{SRC}', str(cpp), '-o', str(executable)]
    subprocess.run(args, check=True, env=env, cwd=tmp_path, timeout=120)
    subprocess.run([str(executable)], check=True, env=env, cwd=tmp_path, timeout=10)
