"""Host test for the G474 communication watchdog (tests/can_watchdog_test.c).

The plan requires the power board to stop about 1 s after the CAN heartbeat stops.
The mechanism already existed for the serial link and the CAN endpoint now feeds
the same state, but nothing verified the TIMING. The existing integration test
calls PD_CanRequest() and never reaches the watchdog path, so this covers a gap
rather than duplicating anything.

Builds the App core against tests/host_port.c and runs it:

    python tools/run_g474_watchdog_test.py
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
    raise SystemExit('Provide a Windows GCC compiler in tools/run_g474_watchdog_test.py')
env = os.environ.copy()
env['PATH'] = str(gcc.parent) + os.pathsep + env.get('PATH', '')

exe = build / 'can_watchdog_test.exe'
sources = ['App/pd_modulation.c', 'App/pd_diag.c', 'App/pd_core.c', 'App/pd_pid.c', 'App/pd_zero.c',
           'App/pd_power.c', 'App/pd_oc.c', 'App/pd_cmp.c', 'App/pd_can.c', 'tests/host_port.c',
           'tests/can_watchdog_test.c']
cmd = [str(gcc), '-std=c99', '-O1', '-Wall', '-Wextra', '-Werror',
       '-DPD_OVERRIDE_NO_LATCH=0', '-DPD_DIAG_MASK_OC=0',
       '-I', 'App', *sources, '-o', str(exe)]
r = subprocess.run(cmd, cwd=root, env=env, capture_output=True, text=True,
                   encoding='utf-8', errors='replace')
if r.returncode:
    print(r.stdout + r.stderr)
    raise SystemExit('watchdog test failed to compile')

r = subprocess.run([str(exe)], cwd=root, env=env, capture_output=True, text=True,
                   encoding='utf-8', errors='replace')
print(r.stdout.rstrip())
if r.stderr.strip():
    print(r.stderr.rstrip())
if r.returncode:
    raise SystemExit('G474 watchdog test FAILED')
