"""Build and run the standalone CAN wire-format check (tests/can_codec_test.c).

Kept separate from tools/test_native.py because that suite currently carries
pre-existing failures; a new check added there would be invisible. This one is
narrow and must pass cleanly.

    python tools/run_can_codec_test.py
"""
from pathlib import Path
import os
import subprocess
import sys

root = Path(__file__).resolve().parents[1]
build = root / 'tests/build'
build.mkdir(exist_ok=True)

gcc = Path('E:/CLion 2025.1.2/bin/mingw/bin/gcc.exe')
if not gcc.exists():
    raise SystemExit('Provide a Windows GCC compiler in tools/run_can_codec_test.py')

env = os.environ.copy()
env['PATH'] = str(gcc.parent) + os.pathsep + env.get('PATH', '')

exe = build / 'can_codec_test.exe'
compile_cmd = [
    str(gcc), '-std=c99', '-O1', '-Wall', '-Wextra', '-Werror',
    '-I', 'App',
    'App/pd_can.c', 'tests/can_codec_test.c',
    '-o', str(exe),
]
r = subprocess.run(compile_cmd, cwd=root, env=env, capture_output=True, text=True)
if r.returncode:
    print(r.stdout + r.stderr)
    raise SystemExit('can codec test failed to compile')

r = subprocess.run([str(exe)], cwd=root, env=env, capture_output=True, text=True)
print(r.stdout.rstrip())
if r.stderr.strip():
    print(r.stderr.rstrip())
if r.returncode:
    raise SystemExit('can codec test FAILED')
print('CAN wire format: OK (offsets match can_protocol.h on the HMI side)')
