"""Check actual G474 startup-zero FSM and power-start interlocks on a host."""
from pathlib import Path
import ctypes as C
import os
import re
import subprocess

root = Path(__file__).resolve().parents[1]
build = root / 'tests/build'
build.mkdir(exist_ok=True)
gcc = Path('E:/CLion 2025.1.2/bin/mingw/bin/gcc.exe')
env = os.environ.copy()
env['PATH'] = str(gcc.parent) + os.pathsep + env.get('PATH', '')
sources = ['pd_modulation', 'pd_diag', 'pd_core', 'pd_pid', 'pd_zero', 'pd_power',
           'pd_oc', 'pd_cmp', 'pd_can']
dll = build / 'startup_zero.dll'
subprocess.run([str(gcc), '-std=c99', '-O1', '-Wall', '-Wextra', '-Werror', '-shared',
                '-static-libgcc', '-Wl,--export-all-symbols', '-I', 'App',
                *['App/' + s + '.c' for s in sources], 'tests/host_port.c', '-o', str(dll)],
               cwd=root, env=env, check=True)
lib = C.CDLL(str(dll))

class Zero(C.Structure):
    _fields_ = [('state', C.c_uint8), ('sensor', C.c_uint8), ('applied', C.c_uint8),
                ('reason', C.c_uint8), ('samples', C.c_uint16), ('iout', C.c_uint32),
                ('il', C.c_uint32), ('pp', C.c_uint32), ('span', C.c_uint32)]

lib.PD_ZeroStartupTick.argtypes = [C.c_uint32, C.c_uint8]
lib.PD_ZeroStartupTick.restype = C.c_uint8
lib.PD_ZeroFeed.argtypes = [C.c_uint32, C.c_uint8, C.c_uint8, C.c_uint16]
lib.PD_ZeroGet.argtypes = [C.POINTER(Zero)]
lib.PD_PowerStart.argtypes = [C.c_uint8]
lib.PD_PowerStart.restype = C.c_uint16
lib.PD_GateTest.argtypes = [C.c_uint8, C.POINTER(C.c_uint8), C.c_uint16,
                           C.POINTER(C.c_uint8), C.POINTER(C.c_uint16)]
lib.PD_GateTest.restype = C.c_uint16
lib.PD_ZeroCurrent.argtypes = [C.c_uint8, C.c_uint16]
lib.PD_ZeroCurrent.restype = C.c_int32
lib.PD_ZeroStart.argtypes = [C.c_uint32, C.c_uint8]
header = (root / 'App/powerdesk_protocol.h').read_text(encoding='utf-8')
def error(name):
    return int(re.search(r'#define PD_ERRORS_' + name + r' (\d+)', header).group(1))
checks = 0
def check(name, value):
    global checks
    checks += 1
    if not value:
        raise AssertionError(name)

def reset():
    lib.Test_Reset()
    lib.Test_Sensors(20000, 0, 0)
    lib.Test_Input(0)

def status():
    z = Zero()
    lib.PD_ZeroGet(C.byref(z))
    return z

def qualify():
    check('boot announces waiting', lib.PD_ZeroStartupTick(1, 0) == 1)
    lib.PD_ZeroStartupTick(2, 1)
    check('not started before 500ms stable', lib.PD_ZeroStartupTick(501, 1) == 0 and not lib.PD_ZeroBusy())
    check('stable conditions start calibration', lib.PD_ZeroStartupTick(502, 1) == 2 and lib.PD_ZeroBusy())

reset()
check('mapping is fixed at boot', lib.PD_GateMappingConfirmed() == 1 and lib.PD_GateMappingCode() == 0)
check('missing zero blocks START', lib.PD_PowerStart(1) == error('NOT_READY'))
check('no output while waiting', lib.Test_Gates() == 0 and lib.Test_PowerEnable() == 0)
lib.PD_ZeroStartupTick(1, 0)
lib.PD_ZeroStartupTick(10000, 0)
check('late input can wait indefinitely without starting', not lib.PD_ZeroBusy())
reset(); qualify()
check('calibration itself does not enable gates', lib.Test_Gates() == 0 and lib.Test_PowerEnable() == 0)
check('START is refused while calibrating', lib.PD_PowerStart(1) == error('BUSY'))
for t in range(1002, 5098):
    lib.PD_ZeroFeed(t, 1, 1, 2040)
check('4096 measured samples applied', status().samples == 4096 and status().applied & 1)
check('success announced once', lib.PD_ZeroStartupTick(5098, 1) == 3)
check('successful zero does not repeat', lib.PD_ZeroStartupTick(20000, 1) == 0 and not lib.PD_ZeroBusy())
check('zero removes the measured offset', lib.PD_ZeroCurrent(1, 2040) == 0)
check('power start now needs no manual map confirmation', lib.PD_PowerStart(1) == 0)
check('explicit START enables the simulated output', lib.Test_PowerEnable() == 1 and lib.Test_Gates() == 15)

reset(); qualify()
lib.PD_ZeroFeed(1002, 0, 1, 2040)
check('condition loss cancels the zero', status().state == 4 and status().reason == 1)
check('temporary interruption returns to wait', lib.PD_ZeroStartupTick(1003, 0) == 1)
lib.PD_ZeroStartupTick(3000, 1)
check('retry still requires stable conditions', lib.PD_ZeroStartupTick(3499, 1) == 0)
check('can retry after safe conditions return', lib.PD_ZeroStartupTick(3500, 1) == 2)
lib.PD_ZeroCancel()
check('stop cancellation returns to wait', lib.PD_ZeroStartupTick(3501, 1) == 1)

reset(); qualify()
for t in range(1002, 5098):
    lib.PD_ZeroFeed(t, 1, 1, 800)
check('out-of-range baseline is not applied', not status().applied and status().reason == 2)
check('bad zero latches startup failure', lib.PD_ZeroStartupTick(5098, 1) == 4)
check('bad zero keeps output blocked', lib.PD_PowerStart(1) == error('NOT_READY'))
check('bad zero is not retried silently', lib.PD_ZeroStartupTick(50000, 1) == 0 and not lib.PD_ZeroBusy())
lib.PD_ZeroStartupRetry()
check('explicit retry returns to safe waiting', lib.PD_ZeroStartupState() == 0 and not lib.PD_ZeroApplied(1))
check('retry does not start before stable conditions', lib.PD_ZeroStartupTick(50001, 1) == 1)
lib.PD_ZeroStartupTick(50002, 1)
check('explicit retry can restart calibration after stability', lib.PD_ZeroStartupTick(50502, 1) == 2)

reset(); qualify()
for t in range(1002, 5098):
    lib.PD_ZeroFeed(t, 1, 1, 1900 if t % 2 else 2180)
check('excessive noise is not accepted', not status().applied and status().reason == 3)
check('noisy zero blocks startup', lib.PD_ZeroStartupTick(5098, 1) == 4 and lib.PD_PowerStart(1) == error('NOT_READY'))
clear_op = int(re.search(r'#define PD_COMMANDS_CLEAR_FAULT (\d+)', header).group(1))
lib.PD_CanRequest.argtypes = [C.c_uint8, C.c_uint8, C.POINTER(C.c_uint8), C.c_uint16]
lib.PD_CanRequest.restype = C.c_uint16
empty = (C.c_uint8 * 1)()
check('CAN clear can request a safe calibration retry', lib.PD_CanRequest(clear_op, 1, empty, 0) == 0 and lib.PD_ZeroStartupState() == 0)
check('retry never grants an unmeasured zero or enables output', not lib.PD_ZeroApplied(1) and lib.Test_Gates() == 0 and lib.PD_PowerStart(1) == error('NOT_READY'))

reset()
lib.PD_ZeroStartupTick(1, 0)
lib.PD_ZeroStartupTick(0xFFFFFF00, 1)
check('qualification handles tick wrap', lib.PD_ZeroStartupTick((0xFFFFFF00 + 499) & 0xffffffff, 1) == 0)
check('qualification starts across wrap', lib.PD_ZeroStartupTick((0xFFFFFF00 + 500) & 0xffffffff, 1) == 2)

reset()
reply = (C.c_uint8 * 16)()
out = C.c_uint16()
for mapping in range(4):
    body = (C.c_uint8 * 4)(2, mapping, 0, 0)
    result = lib.PD_GateTest(2, body, 4, reply, C.byref(out))
    check('legacy mapping cannot change fixed wiring', result == (0 if mapping == 0 else error('UNSUPPORTED')) and lib.PD_GateMappingCode() == 0)
body = (C.c_uint8 * 4)(3, 0, 0, 0)
check('legacy clear cannot restore a manual-confirmation gate', lib.PD_GateTest(3, body, 4, reply, C.byref(out)) == 0 and lib.PD_GateMappingConfirmed() == 1)
print(f'G474 startup zero / fixed mapping: {checks} checks, 0 failures (host, not hardware)')
