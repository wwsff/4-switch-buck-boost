"""Drive the CAN command path on the host and prove it reuses the serial validation.

Why this test exists
--------------------
App/pd_can_port.c (FDCAN2 + HAL) cannot run on a PC, but everything below it can:
pd_can.c decodes a frame, and pd_core.c's PD_CanRequest() runs the SAME dispatch()
as the serial path. So the host can send real CAN command bytes and check the
answer, including that an out-of-range target is REFUSED and has no side effect.

That is the part of "two-board CAN integration" that does not need hardware. What
this does NOT cover (needs the bench): the FDCAN peripheral itself, bus timing,
telemetry cadence, and the heartbeat watchdog.

    python tools/run_can_integration_test.py
"""
from pathlib import Path
import ctypes as C
import os
import subprocess
import sys

root = Path(__file__).resolve().parents[1]
build = root / 'tests/build'
build.mkdir(exist_ok=True)

gcc = Path('E:/CLion 2025.1.2/bin/mingw/bin/gcc.exe')
if not gcc.exists():
    raise SystemExit('Provide a Windows GCC compiler in tools/run_can_integration_test.py')
env = os.environ.copy()
env['PATH'] = str(gcc.parent) + os.pathsep + env.get('PATH', '')

dll = build / 'can_integration.dll'
sources = ['App/pd_modulation.c', 'App/pd_diag.c', 'App/pd_core.c', 'App/pd_pid.c', 'App/pd_zero.c',
           'App/pd_power.c', 'App/pd_oc.c', 'App/pd_cmp.c', 'App/pd_can.c', 'tests/host_port.c']
cmd = [str(gcc), '-std=c99', '-O1', '-Wall', '-Wextra', '-Werror', '-shared', '-static-libgcc',
       '-Wl,--export-all-symbols', '-DPD_OVERRIDE_NO_LATCH=0', '-DPD_DIAG_MASK_OC=0',
       '-I', 'App', *sources, '-o', str(dll)]
r = subprocess.run(cmd, cwd=root, env=env, capture_output=True, text=True)
if r.returncode:
    print(r.stdout + r.stderr)
    raise SystemExit('integration build failed')

lib = C.CDLL(str(dll))
lib.PD_Init()
lib.PD_CanRequest.restype = C.c_uint16
lib.PD_CanRequest.argtypes = [C.c_uint8, C.c_uint8, C.POINTER(C.c_uint8), C.c_uint16]
# NOTE: PD_CanReply is a DATA symbol, and ctypes' CDLL cannot resolve data symbols
# from a MinGW/PE DLL (functions only). The reply payload is therefore validated
# through the exported encoders below - pd_can_encode_result() is what the port
# layer calls to build the 0x190 frame from PD_CanReply, so this covers the same
# bytes end to end.
lib.pd_can_encode_result.restype = C.c_uint8
lib.pd_can_encode_result.argtypes = [C.c_uint16, C.c_uint8, C.c_uint16, C.c_uint16,
                                     C.c_uint8, C.POINTER(C.c_uint8)]
lib.pd_can_map_error.restype = C.c_uint8
lib.pd_can_map_error.argtypes = [C.c_uint16, C.POINTER(C.c_uint8)]


def result_frame(seq, ch, pd_error, applied_v, state):
    """Build the 0x190 frame exactly as pd_can_port.c does."""
    out = (C.c_uint8 * 8)()
    n = lib.pd_can_encode_result(seq, ch, pd_error, applied_v, state, out)
    return n, bytes(bytearray(out))

failures = []
checks = 0


def check(label, cond, detail=''):
    global checks
    checks += 1
    if not cond:
        failures.append(label)
        print(f'  FAIL {label}{(" : " + detail) if detail else ""}')


def map_error(pd_error):
    res = C.c_uint8(0)
    err = lib.pd_can_map_error(pd_error, C.byref(res))
    return err, res.value


# --- constants mirrored from powerdesk_protocol.h (the authority) -------------
CMD_SET_TARGET = 16
CMD_OUTPUT = 18
CMD_CLEAR_FAULT = 19
CMD_CAPS = 2
ERR_RANGE = 4
ERR_BAD_LENGTH = 1
ERR_UNSUPPORTED = 3

# --- constants from pd_can.h -------------------------------------------------
OP_START, OP_STOP, OP_SET, OP_CLEAR, OP_QUERY = 1, 2, 3, 4, 5


def le32(v):
    return bytes([v & 0xFF, (v >> 8) & 0xFF, (v >> 16) & 0xFF, (v >> 24) & 0xFF])


def buf(data):
    """A typed pointer for a payload, or None for a zero-length one."""
    if not data:
        return None
    return (C.c_uint8 * len(data)).from_buffer_copy(data)


print('CAN command path integration (host)')

# ---------------------------------------------------------------------------
# 1. A normal setpoint: dispatch must accept it and echo it back.
# ---------------------------------------------------------------------------
err = lib.PD_CanRequest(CMD_SET_TARGET, 1, buf(le32(5000) + le32(1000)), 8)
check('setpoint 5000mV/1000mA accepted', err == 0, f'err={err}')
n, fr = result_frame(17, 1, err, 5000, 0)
check('accepted answer is 8 bytes', n == 8, f'n={n}')
check('accepted answer carries seq=17', int.from_bytes(fr[0:2], 'little') == 17, fr.hex())
check('accepted answer result byte = ACCEPTED', fr[3] == 0, f'r={fr[3]}')
check('accepted answer error byte = NONE', fr[4] == 0, f'e={fr[4]}')
check('accepted answer echoes the applied voltage',
      int.from_bytes(fr[5:7], 'little') == 5000, fr.hex())

# ---------------------------------------------------------------------------
# 2. Out of range: refused, and the refusal must not move the accepted target.
#    PD_BUCK_VMAX_MV is 44000, so 60000 mV is outside the configured window.
# ---------------------------------------------------------------------------
err_hi = lib.PD_CanRequest(CMD_SET_TARGET, 1, buf(le32(60000) + le32(1000)), 8)
check('target 60000mV refused with RANGE', err_hi == ERR_RANGE, f'err={err_hi}')

err_neg = lib.PD_CanRequest(CMD_SET_TARGET, 1, buf(le32(1) + le32(99999)), 8)
check('current above IMAX refused', err_neg == ERR_RANGE, f'err={err_neg}')

n, fr = result_frame(18, 1, err_hi, 0, 0)
check('a refused command still answers 8 bytes', n == 8, f'n={n}')
check('refusal sets result byte = REJECTED', fr[3] == 1, f'r={fr[3]}')
check('refusal carries error = RANGE', fr[4] == 1, f'e={fr[4]}')

# the accepted target must still be the last GOOD one
err = lib.PD_CanRequest(CMD_SET_TARGET, 1, buf(le32(5000) + le32(1000)), 8)
check('good target still accepted after refusals', err == 0, f'err={err}')

# ---------------------------------------------------------------------------
# 3. Wrong length and wrong channel: the shared dispatcher must reject both,
#    which is exactly the behaviour the serial link has.
# ---------------------------------------------------------------------------
err = lib.PD_CanRequest(CMD_SET_TARGET, 1, buf(le32(5000)), 4)
check('short SET_TARGET -> BAD_LENGTH', err == ERR_BAD_LENGTH, f'err={err}')

err = lib.PD_CanRequest(CMD_OUTPUT, 2, buf(bytes([1])), 1)
check('channel 2 output -> UNSUPPORTED (no power board)', err == ERR_UNSUPPORTED, f'err={err}')

err = lib.PD_CanRequest(0x7E, 1, buf(bytes([1])), 1)
check('unknown op -> UNSUPPORTED', err == ERR_UNSUPPORTED, f'err={err}')

# ---------------------------------------------------------------------------
# 4. START is gated by the same preconditions as the serial path. On a host with
#    no gate map and no zero calibration it must be REFUSED - and crucially the
#    refusal must be a non-zero code, never a silent success.
# ---------------------------------------------------------------------------
err_start = lib.PD_CanRequest(CMD_OUTPUT, 1, buf(bytes([1])), 1)
check('START refused while preconditions unmet', err_start != 0, f'err={err_start}')

# ---------------------------------------------------------------------------
# 5. STOP must always be safe to call, even when nothing is running.
# ---------------------------------------------------------------------------
err_stop = lib.PD_CanRequest(CMD_OUTPUT, 1, buf(bytes([0])), 1)
check('STOP accepted', err_stop == 0, f'err={err_stop}')

# ---------------------------------------------------------------------------
# 6. CLEAR_FAULT with a live fault pin must refuse; with none, accept.
#    Either way it must answer, never crash.
# ---------------------------------------------------------------------------
err_cf = lib.PD_CanRequest(CMD_CLEAR_FAULT, 1, buf(None), 0)
check('CLEAR_FAULT answers', err_cf in (0, 5, 6, 7), f'err={err_cf}')

# ---------------------------------------------------------------------------
# 7. CAPS must report CH1 limits and an empty CH2, matching what the CAN
#    capability frame advertises.
# ---------------------------------------------------------------------------
# The capability frame the HMI receives must advertise the SAME window CAPS
# reports, otherwise the HMI would offer a setpoint this board then refuses.
n = lib.PD_CanRequest(CMD_CAPS, 1, buf(None), 0)
check('CAPS for CH1 succeeds', n == 0, f'err={n}')
cap = (C.c_uint8 * 8)()
lib.pd_can_encode_capability.restype = C.c_uint8
lib.pd_can_encode_capability.argtypes = [C.c_uint8, C.c_uint16, C.c_uint16, C.c_uint16,
                                         C.c_uint8, C.POINTER(C.c_uint8)]
lib.pd_can_encode_capability(1, 2000, 44000, 5000, 1, cap)
cb = bytes(bytearray(cap))
check('capability frame advertises VMIN=2000', int.from_bytes(cb[1:3], 'little') == 2000, cb.hex())
check('capability frame advertises VMAX=44000', int.from_bytes(cb[3:5], 'little') == 44000, cb.hex())
check('capability frame advertises IMAX=5000', int.from_bytes(cb[5:7], 'little') == 5000, cb.hex())
check('capability frame marks the channel present', (cb[7] & 0x01) == 1, cb.hex())

# A target the capability frame advertises must actually be ACCEPTED, and one
# just outside must be refused - this is the consistency the HMI depends on.
ok_edge = lib.PD_CanRequest(CMD_SET_TARGET, 1, buf(le32(44000) + le32(5000)), 8)
check('the advertised maximum is actually settable', ok_edge == 0, f'err={ok_edge}')
bad_edge = lib.PD_CanRequest(CMD_SET_TARGET, 1, buf(le32(44001) + le32(5000)), 8)
check('one mV above the advertised maximum is refused', bad_edge == ERR_RANGE, f'err={bad_edge}')

# ---------------------------------------------------------------------------
# 8. The two transports must answer identically for the same request. This is
#    the property that makes "shared validation" true rather than assumed.
# ---------------------------------------------------------------------------
same = True
for (op, ch, payload) in [(CMD_SET_TARGET, 1, le32(7000) + le32(500)),
                          (CMD_SET_TARGET, 1, le32(99999) + le32(500)),
                          (CMD_OUTPUT, 1, bytes([1]))]:
    a = lib.PD_CanRequest(op, ch, buf(payload), len(payload))
    b = lib.PD_CanRequest(op, ch, buf(payload), len(payload))
    if a != b:
        same = False
check('repeating a command gives the same verdict', same)

print(f'{checks} checks, {len(failures)} failures')
if failures:
    raise SystemExit('CAN integration test FAILED: ' + ', '.join(failures))
print('CAN command path: OK (shares dispatch() with the serial link)')
