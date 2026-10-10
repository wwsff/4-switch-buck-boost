"""Compile the App sources in the DIAGNOSTIC, OVERRIDE and FALLBACK-MODULATOR variants.

The release build keeps every protection switch at 0, so `#if PD_DIAG_MASK_OC`
and `#if PD_OVERRIDE_NO_LATCH` blocks are never compiled by tools/test_native.py.
They are bench-only, but they must still COMPILE: a broken diagnostic build is
discovered at the bench, which is the worst possible moment. This tool compiles
them with the host port and reports errors (no tests are run).

It also compiles the LEGACY modulator (PD_MOD_LINKED = 0). The plan requires the
old A-regulated / B-fixed-at-75 % mode to stay available as a rollback, and a
rollback that no longer compiles is not a rollback. The #else branches in
pd_power.c, commit_duties() and the precharge hand-off only exist in that build.
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
    raise SystemExit('Provide a Windows GCC compiler in tools/compile_variants.py')
env = os.environ.copy()
env['PATH'] = str(gcc.parent) + os.pathsep + env.get('PATH', '')
sources = ['App/pd_modulation.c', 'App/pd_diag.c', 'App/pd_core.c', 'App/pd_pid.c', 'App/pd_zero.c', 'App/pd_power.c',
           'App/pd_oc.c', 'App/pd_cmp.c', 'App/pd_can.c', 'tests/host_port.c']
variants = {
    'release': ['-DPD_OVERRIDE_NO_LATCH=0', '-DPD_DIAG_MASK_OC=0'],
    'diag-oc-masked': ['-DPD_OVERRIDE_NO_LATCH=0', '-DPD_DIAG_MASK_OC=1'],
    'override-nolatch': ['-DPD_OVERRIDE_NO_LATCH=1', '-DPD_DIAG_MASK_OC=0'],
    'override+diag': ['-DPD_OVERRIDE_NO_LATCH=1', '-DPD_DIAG_MASK_OC=1'],
    'legacy-modulator': ['-DPD_OVERRIDE_NO_LATCH=0', '-DPD_DIAG_MASK_OC=0', '-DPD_MOD_LINKED=0'],
}
failed = []
for name, defines in variants.items():
    result = subprocess.run(
        [str(gcc), '-std=c99', '-O1', '-Wall', '-Wextra', '-Werror', '-shared', '-static-libgcc',
         '-Wl,--export-all-symbols', *defines, '-I', 'App', *sources, '-o', str(build / f'variant_{name}.dll')],
        cwd=root, env=env, capture_output=True, text=True)
    state = 'ok' if result.returncode == 0 else 'FAILED'
    print(f'{name:18s} {state}')
    if result.returncode:
        failed.append(name)
        print(result.stdout + result.stderr)
if failed:
    raise SystemExit('variant compile failed: ' + ', '.join(failed))
print('all build variants compile cleanly (-Wall -Wextra -Werror)')
