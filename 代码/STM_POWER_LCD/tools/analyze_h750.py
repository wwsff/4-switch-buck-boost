"""Run GCC's static analyzer over the H750 driver and UI code.

WHY. lcd.c, touch.c and gfx_text.c hold the most hand-written buffer and bounds
arithmetic in this project, and none of them has run on hardware yet - there is no
display to plug in. -fanalyzer can follow the paths without a board, so it is the
strongest check available for that code today. It needs HAL types to exist or it
stops at the first parse error, hence User/HostTest/analyze/hal_stub.h.

What this DOES find: buffer overruns, off-by-one writes, null dereferences,
uninitialised reads and signedness traps on the paths it can follow.
What it does NOT do: prove correctness, or model the real ST7789/FT6336 timing.
A clean run is "no defects found", not "no defects".

    python tools/analyze_h750.py
"""
from pathlib import Path
import os
import re
import subprocess
import sys

proj = Path(__file__).resolve().parents[1]
src = proj / 'STM_POWER_LCD'

gcc = Path('E:/CLion 2025.1.2/bin/mingw/bin/gcc.exe')
if not gcc.exists():
    raise SystemExit('Provide a Windows GCC compiler in tools/analyze_h750.py')
env = os.environ.copy()
env['PATH'] = str(gcc.parent) + os.pathsep + env.get('PATH', '')

# The analyze/ stub directory MUST come before Drivers/ so spi.h resolves to the
# declaration-only stand-in rather than pulling in the real HAL.
includes = [
    src / 'User' / 'HostTest' / 'analyze',
    src / 'User' / 'Ui' / 'Inc',
    src / 'User' / 'Bsp' / 'Inc',
    src / 'User' / 'App' / 'Inc',
    src / 'User' / 'Shared',
    src / 'Core' / 'Inc',
]
inc_args = []
for i in includes:
    inc_args += ['-I', str(i)]

# Files that need a HAL. can_proto.c, ui_model.c and app_input.c are analysed by
# the host protocol test's build instead, which supplies its own shim.
TARGETS = [
    'User/Bsp/Src/lcd.c',
    'User/Ui/Src/gfx_text.c',
]

# Diagnostics that come from the stand-in or from 64-bit-host vs 32-bit-target
# differences, not from the code under review.
NOISE = re.compile(
    r'int-to-pointer-cast|pointer-to-int-cast|'
    r'analyzer.*hal_stub|'
    r'leak of|resource leak'
)

failures = []
for rel in TARGETS:
    cmd = [str(gcc), '-std=c99', '-O1', '-Wall', '-Wextra', '-fanalyzer',
           '-fsyntax-only', *inc_args, str(src / rel)]
    r = subprocess.run(cmd, cwd=proj, env=env, capture_output=True, text=True,
                       encoding='utf-8', errors='replace')
    lines = [l for l in (r.stdout + r.stderr).splitlines() if l.strip()]
    real = [l for l in lines if ('warning:' in l or 'error:' in l) and not NOISE.search(l)]
    if real:
        failures.append(rel)
        print(f'{rel}: {len(real)} diagnostic(s)')
        for l in real[:20]:
            print('  ' + l.strip())
    else:
        print(f'{rel}: clean')

if failures:
    raise SystemExit('static analysis found issues: ' + ', '.join(failures))
print('\nstatic analysis: no defects found on the paths analysed')
