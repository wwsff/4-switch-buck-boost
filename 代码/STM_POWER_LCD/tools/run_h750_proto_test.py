"""Build and run the H750 CAN protocol state machine test on the host.

Compiles the real User/App/Src/can_proto.c and User/Ui/Src/ui_model.c together
with a fake CAN bus, a fake clock and shadowed logging, then asserts the two
safety properties the plan states in words:

    "H750 updates the 'output ON' display only from the G474 status report."
    "On link loss show 'device offline', and do NOT resend the old start command."

can_proto.c is NOT modified for this: the shim directory is placed first on the
include path so the real main.h and bsp_log.h (which pull in the STM32 HAL) are
shadowed.

    python tools/run_h750_proto_test.py
"""
from pathlib import Path
import os
import subprocess
import sys

proj = Path(__file__).resolve().parents[1]
src = proj / 'STM_POWER_LCD'
build = src / 'User' / 'HostTest' / 'build'
build.mkdir(parents=True, exist_ok=True)

gcc = Path('E:/CLion 2025.1.2/bin/mingw/bin/gcc.exe')
if not gcc.exists():
    raise SystemExit('Provide a Windows GCC compiler in tools/run_h750_proto_test.py')
env = os.environ.copy()
env['PATH'] = str(gcc.parent) + os.pathsep + env.get('PATH', '')

exe = build / 'h750_proto_test.exe'
# ORDER MATTERS: the shim directory must come before anything that would reach
# the real main.h / bsp_log.h.
includes = [
    src / 'User' / 'HostTest' / 'shim',
    src / 'User' / 'App' / 'Inc',
    src / 'User' / 'Ui' / 'Inc',
    src / 'User' / 'Shared',
    src / 'User' / 'Bsp' / 'Inc',
]
inc_args = []
for i in includes:
    inc_args += ['-I', str(i)]

sources = [
    src / 'User' / 'App' / 'Src' / 'can_proto.c',
    src / 'User' / 'Ui' / 'Src' / 'ui_model.c',
    src / 'User' / 'HostTest' / 'h750_proto_test.c',
]

cmd = [str(gcc), '-std=c99', '-O1', '-Wall', '-Wextra', '-Werror',
       *inc_args, *[str(s) for s in sources], '-o', str(exe)]
r = subprocess.run(cmd, cwd=proj, env=env, capture_output=True, text=True, encoding='utf-8', errors='replace')
if r.returncode:
    print(r.stdout + r.stderr)
    raise SystemExit('H750 protocol test failed to compile')

r = subprocess.run([str(exe)], cwd=proj, env=env, capture_output=True, text=True, encoding='utf-8', errors='replace')
print(r.stdout.rstrip())
if r.stderr.strip():
    print(r.stderr.rstrip())
if r.returncode:
    raise SystemExit('H750 protocol test FAILED')
