"""Exercise real NTC conversion/filter code, including unplugging and stale DMA."""
from pathlib import Path
import ctypes as C
import math
import os
import subprocess

root = Path(__file__).resolve().parents[1]
build = root / 'tests/build'
build.mkdir(exist_ok=True)
gcc = Path('E:/CLion 2025.1.2/bin/mingw/bin/gcc.exe')
env = os.environ.copy()
env['PATH'] = str(gcc.parent) + os.pathsep + env.get('PATH', '')
board = (root / 'App/pd_board.c').read_text(encoding='utf-8')
# Compile the exact task-side filter, not a second implementation of it.
filter_code = board[board.index('static int32_t ntc_filter'):board.index('void PD_BoardPoll(void)')]
# Scheduler variables are not used by this standalone test.
filter_code = filter_code.replace('static uint32_t ntc_at;', '').replace('static uint8_t ntc_started;', '')
source = '#include "pd_ntc.h"\n' + filter_code + '''
int16_t convert(uint16_t raw) { return PD_NtcConvert(raw); }
int16_t read_ntc(uint8_t ch, uint8_t sensor, uint16_t raw, uint8_t fresh, uint8_t due)
{ return ntc_read(ch, sensor, raw, fresh, due); }
'''
cfile = build / 'ntc_test.c'
cfile.write_text(source, encoding='utf-8')
dll = build / 'ntc_test.dll'
subprocess.run([str(gcc), '-std=c99', '-O1', '-Wall', '-Wextra', '-Werror', '-shared',
                '-static-libgcc', '-Wl,--export-all-symbols', '-I', str(root / 'App'),
                str(cfile), '-o', str(dll)], env=env, check=True)
lib = C.CDLL(str(dll))
lib.convert.argtypes = [C.c_uint16]
lib.convert.restype = C.c_int16
lib.read_ntc.argtypes = [C.c_uint8, C.c_uint8, C.c_uint16, C.c_uint8, C.c_uint8]
lib.read_ntc.restype = C.c_int16
checks = 0
def check(name, condition):
    global checks
    checks += 1
    if not condition:
        raise AssertionError(name)

def raw(c):
    resistance = 10000 * math.exp(3450 * (1 / (c + 273.15) - 1 / 298.15))
    return round(4095 * resistance / (10000 + resistance))

for c in [-39, -20, 0, 25, 50, 75, 100, 124]:
    check(f'conversion at {c} C', abs(lib.convert(raw(c)) - c * 100) <= 15)
for code in [0, 8, 4087, 4095, 65535, raw(-50), raw(140)]:
    check(f'implausible ADC {code}', lib.convert(code) == -32768)
check('25 C divider midpoint', abs(lib.convert(2048) - 2500) <= 2)
values = [lib.convert(code) for code in range(raw(124), raw(-39))]
check('monotonic NTC curve', all(a >= b for a, b in zip(values, values[1:])))
check('first valid initializes at actual temperature', abs(lib.read_ntc(0, 0, raw(25), 1, 1) - 2500) <= 2)
check('no due tick retains filtered value', abs(lib.read_ntc(0, 0, raw(75), 1, 0) - 2500) <= 2)
filtered = lib.read_ntc(0, 0, raw(75), 1, 1)
check('IIR approaches warmer reading', 2500 < filtered < 7500)
check('open wire invalidates immediately between conversions', lib.read_ntc(0, 0, 4095, 1, 0) == -32768)
check('reconnection initializes instead of keeping old temperature', abs(lib.read_ntc(0, 0, raw(50), 1, 0) - 5000) <= 5)
check('stale DMA cannot retain a plausible reading', lib.read_ntc(0, 0, raw(50), 0, 0) == -32768)
check('out-of-model raw cannot be hidden by the filter', lib.read_ntc(0, 0, raw(140), 1, 0) == -32768)
check('coil is independent from MOS filter', abs(lib.read_ntc(0, 1, raw(0), 1, 1)) <= 5)
check('absent CH2 is invalid', lib.read_ntc(1, 0, raw(25), 0, 1) == -32768)
print(f'NTC conversion/filter: {checks} checks, 0 failures')
