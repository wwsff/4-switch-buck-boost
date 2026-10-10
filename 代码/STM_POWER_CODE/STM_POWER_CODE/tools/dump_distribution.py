"""Print what the compiled distributor actually produces, next to the design table.

Why this exists next to tests/test_modulation.py: the tests assert the properties, this prints the
mapping in the coordinates the bench speaks. Phase B of the plan raises the input voltage and reads
the A/B compare pair out of the 1 Hz log, so the operator needs q -> (A, B) and the inverse
(A, B) -> q in one glance. It runs the real DLL, so the numbers cannot drift from the firmware.

Run `python tools/test_native.py` first - it builds tests/build/firmware_core.dll.
"""
import ctypes as C
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CONFIG = (ROOT / 'App/pd_config.h').read_text(encoding='utf-8')


def cfg_int(name):
    line = re.search(rf'^[ \t]*#define[ \t]+{name}[ \t]+(-?[0-9][0-9_]*)', CONFIG, re.M)
    if line is None:
        raise SystemExit(f'{name} not found in App/pd_config.h')
    return int(line.group(1).replace('_', ''))


class ModResult(C.Structure):
    _fields_ = [('q_request', C.c_int32), ('q_applied', C.c_int32), ('cmp_a', C.c_uint16),
                ('cmp_b', C.c_uint16), ('zone', C.c_uint8), ('limit', C.c_uint8)]


lib = C.CDLL(str(ROOT / 'tests/build/firmware_core.dll'))
lib.PD_ModDistribute.argtypes = [C.c_int32, C.POINTER(ModResult)]
lib.PD_ModAppliedQ.argtypes = [C.c_uint16, C.c_uint16]
lib.PD_ModAppliedQ.restype = C.c_int32

PERIOD = cfg_int('PD_SW_PERIOD')
B_MIN = cfg_int('PD_MOD_B_MIN_COUNTS')
Q_MAX = cfg_int('PD_MOD_Q_MAX_COUNTS')
# The plan's section 4.3 table, so a divergence shows up as a printed difference rather than as a
# claim in a document nobody re-runs.
PLAN = {0.60: (60.00, 75.00), 0.65: (65.00, 75.00), 0.70: (68.75, 73.66),
        0.75: (70.00, 70.00), 0.82: (70.00, 64.02), 0.875: (70.00, 60.00)}
ZONE = {0: 'LOW', 1: 'TRANS', 2: 'HIGH', 3: 'NONE'}

print(f'period {PERIOD} counts; released q range 0.05 .. {Q_MAX / PERIOD:.3f}; '
      f'B floor {B_MIN / PERIOD:.2f}')
print(f'{"q":>7} {"q cnt":>6} {"zone":>6} {"A cnt":>6} {"B cnt":>6} {"A%":>7} {"B%":>7} '
      f'{"A/B":>7} {"plan A%":>8} {"plan B%":>8} {"q_appl":>7} {"limit":>6}')
for q_duty in (0.05, 0.60, 0.649, 0.65, 0.66, 0.70, 0.74, 0.75, 0.76, 0.82, 0.875):
    q = round(q_duty * PERIOD)
    r = ModResult()
    lib.PD_ModDistribute(q, C.byref(r))
    pa, pb = PLAN.get(q_duty, ('', ''))
    print(f'{q_duty:>7.3f} {q:>6} {ZONE.get(r.zone, "?"):>6} {r.cmp_a:>6} {r.cmp_b:>6} '
          f'{100 * r.cmp_a / PERIOD:>7.2f} {100 * r.cmp_b / PERIOD:>7.2f} '
          f'{r.cmp_a / r.cmp_b:>7.4f} {str(pa):>8} {str(pb):>8} {r.q_applied:>7} '
          f'0x{r.limit:02X}')

print('\nlimit bits: 0x01 Q_MAX  0x02 Q_MIN  0x04 A_MAX  0x08 B_MIN  0x10 A_MIN  '
      '0x20 ZERO  0x40 INVALID')
print('0x00 everywhere above means the released range is executable without clamping; a non-zero\n'
      'bit is what the controller must treat as "the actuator did not follow".')
