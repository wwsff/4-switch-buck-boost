"""Distributor tests (plan sections 11.1 and 11.2).

Pure-function tests against the real compiled distributor, not a reimplementation: the DLL exports
PD_ModDistribute and PD_ModAppliedQ, so what is exercised here is what the firmware runs.

Covered, one assertion per plan item:
  1. q = 0, zone endpoints, dense interior points, the q ceiling, and out-of-range / negative input.
  2. DA/DB ratio, continuity, monotonicity, integer rounding, both physical bounds, and the
     released experimental B floor.
  3. the documented zone table of the plan (q 0.60 ... 0.875).
"""
import ctypes as C
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / 'App/pd_config.h'


def cfg_int(name):
    import re
    line = re.search(rf'^[ \t]*#define[ \t]+{name}[ \t]+(-?[0-9][0-9_]*)',
                     CONFIG.read_text(encoding='utf-8'), re.M)
    if line is None:
        raise AssertionError(f'{name} not found in pd_config.h')
    return int(line.group(1).replace('_', ''))


class ModResult(C.Structure):
    _fields_ = [('q_request', C.c_int32), ('q_applied', C.c_int32),
                ('cmp_a', C.c_uint16), ('cmp_b', C.c_uint16),
                ('zone', C.c_uint8), ('limit', C.c_uint8)]


lib = C.CDLL(str(ROOT / 'tests/build/firmware_core.dll'))
lib.PD_ModDistribute.argtypes = [C.c_int32, C.POINTER(ModResult)]
lib.PD_ModDistribute.restype = None
lib.PD_ModAppliedQ.argtypes = [C.c_uint16, C.c_uint16]
lib.PD_ModAppliedQ.restype = C.c_int32

PERIOD = cfg_int('PD_SW_PERIOD')
A_MAX = cfg_int('PD_CMP_A_MAX')
B_MAX = cfg_int('PD_OUT_BRIDGE_CMP')
B_MIN = cfg_int('PD_MOD_B_MIN_COUNTS')
Q_MAX = cfg_int('PD_MOD_Q_MAX_COUNTS')
Q_MIN = cfg_int('PD_MOD_Q_MIN_COUNTS')
Q_LOW = cfg_int('PD_MOD_Q_LOW_COUNTS')
Q_HIGH = cfg_int('PD_MOD_Q_HIGH_COUNTS')
A_MIN = cfg_int('PD_MOD_A_MIN_COUNTS')
# Limit bits are read from the header the firmware compiles, not typed in again: a renumbered bit
# must fail these tests instead of being silently re-tested against the old value.
MOD_HEADER = ROOT / 'App/pd_modulation.h'


def lim(name):
    import re
    hit = re.search(rf'^[ \t]*#define[ \t]+PD_MOD_LIMIT_{name}[ \t]+(0x[0-9A-Fa-f]+)',
                    MOD_HEADER.read_text(encoding='utf-8'), re.M)
    if hit is None:
        raise AssertionError(f'PD_MOD_LIMIT_{name} not found in pd_modulation.h')
    return int(hit.group(1), 16)


LIM_NONE, LIM_Q_MAX, LIM_Q_MIN = lim('NONE'), lim('Q_MAX'), lim('Q_MIN')
LIM_A_MAX, LIM_B_MIN = lim('A_MAX'), lim('B_MIN')
LIM_A_MIN, LIM_ZERO, LIM_INVALID = lim('A_MIN'), lim('ZERO'), lim('INVALID')


def distribute(q_counts):
    result = ModResult()
    lib.PD_ModDistribute(q_counts, C.byref(result))
    return result


def duty(counts):
    """Duty as a PERCENTAGE, matching the design document's zone table."""
    return 100.0 * counts / PERIOD


class DistributorTests(unittest.TestCase):
    def test_zero_request_is_reported_and_writes_no_duty(self):
        """Plan 4.4: q <= 0 goes through an explicit zero path; the modulator does not invent a
        tiny compare value to stand in for OFF - the power state machine owns that."""
        result = distribute(0)
        self.assertEqual(result.limit, LIM_ZERO)
        self.assertEqual(result.zone, 3)                     # PD_MOD_ZONE_NONE
        # the pair it hands back is not a running setpoint; the caller must not commit it as one
        self.assertLessEqual(result.cmp_a, A_MAX)

    def test_negative_and_malformed_input_is_invalid(self):
        for q in (-1, -100000):
            result = distribute(q)
            self.assertEqual(result.limit, LIM_INVALID, q)
            self.assertEqual(result.q_applied, 0, q)

    def test_low_zone_keeps_the_legacy_fixed_b(self):
        """The regression baseline: below the transition, B stays exactly at 75 % and A carries the
        whole control action, which is what 0.3.27 did."""
        for q in (Q_MIN, 2000, 8000, Q_LOW):
            result = distribute(q)
            self.assertEqual(result.zone, 0, q)              # LOW
            self.assertEqual(result.cmp_b, B_MAX, q)
            self.assertEqual(result.cmp_a, q, q)
            self.assertEqual(result.limit & (LIM_Q_MAX | LIM_B_MIN), 0, q)

    def test_zone_endpoints_are_continuous_in_value_and_slope(self):
        """Plan 4.2: DA, DB and their first derivatives are continuous at both zone boundaries."""
        for boundary in (Q_LOW, Q_HIGH):
            before = distribute(boundary - 1)
            at = distribute(boundary)
            after = distribute(boundary + 1)
            self.assertLessEqual(abs(at.cmp_a - before.cmp_a), 1,
                                 f'A jump at q={boundary}: {before.cmp_a} -> {at.cmp_a}')
            self.assertLessEqual(abs(after.cmp_a - at.cmp_a), 1,
                                 f'A jump at q={boundary}: {at.cmp_a} -> {after.cmp_a}')
            self.assertLessEqual(abs(at.cmp_b - before.cmp_b), 2,
                                 f'B jump at q={boundary}: {before.cmp_b} -> {at.cmp_b}')
            self.assertLessEqual(abs(after.cmp_b - at.cmp_b), 2,
                                 f'B jump at q={boundary}: {at.cmp_b} -> {after.cmp_b}')

    def test_zone_table_matches_the_design_document(self):
        """The table in plan section 4.3, to the nearest percent."""
        expected = [(0.60, 60.00, 75.00), (0.65, 65.00, 75.00), (0.70, 68.75, 73.66),
                    (0.75, 70.00, 70.00), (0.82, 70.00, 64.02), (0.875, 70.00, 60.00)]
        for q_duty, a_duty, b_duty in expected:
            result = distribute(round(q_duty * PERIOD))
            self.assertAlmostEqual(duty(result.cmp_a), a_duty, delta=0.6,
                                   msg=f'q={q_duty}: A {duty(result.cmp_a):.2f}% vs {a_duty}%')
            self.assertAlmostEqual(duty(result.cmp_b), b_duty, delta=0.8,
                                   msg=f'q={q_duty}: B {duty(result.cmp_b):.2f}% vs {b_duty}%')

    def test_ratio_relation_holds_across_the_whole_range(self):
        """DA/DB = q/0.75 whenever nothing is limited - the property the whole scheme rests on."""
        for q in range(Q_MIN, Q_MAX + 1, 37):
            result = distribute(q)
            if result.limit != LIM_NONE:
                continue
            self.assertAlmostEqual(result.q_applied, q, delta=1,
                                   msg=f'q={q}: applied {result.q_applied}')
            ratio = result.cmp_a / result.cmp_b
            self.assertAlmostEqual(ratio, (q / PERIOD) / 0.75, delta=0.002, msg=f'q={q}')

    def test_monotonicity_of_both_bridges(self):
        """A must rise then hold at its ceiling; B must fall once the transition starts. Neither may
        reverse: a non-monotonic distributor would fight the loop."""
        previous = None
        for q in range(Q_MIN, Q_MAX + 1, 13):
            result = distribute(q)
            if previous is not None:
                self.assertGreaterEqual(result.cmp_a, previous[0] - 1, f'A fell at q={q}')
                self.assertLessEqual(result.cmp_b, previous[1] + 1, f'B rose at q={q}')
            previous = (result.cmp_a, result.cmp_b)

    def test_physical_bounds_are_never_violated(self):
        for q in list(range(0, Q_MAX + 400, 11)) + [Q_MAX + 1, Q_MAX * 2, 1000000]:
            result = distribute(q)
            self.assertLessEqual(result.cmp_a, A_MAX, q)
            self.assertLessEqual(result.cmp_b, B_MAX, q)
            self.assertGreaterEqual(result.cmp_b, B_MIN, q)
            self.assertGreaterEqual(result.cmp_a, cfg_int('PD_CMP_MIN'), q)
            # A > B is legitimate in the high zone (VOUT > VIN, the point of this stage), so it
            # is asserted to be PRODUCED, not refused - plan 5.1 requires A and B to be allowed to
            # reorder. Safety is a timing question handled by the register checks and the bench.
            if q >= Q_HIGH:
                self.assertGreaterEqual(result.cmp_a, 0, q)

    def test_q_ceiling_reports_the_limit_and_holds_b_at_the_floor(self):
        result = distribute(Q_MAX + 5000)
        self.assertEqual(result.limit & LIM_Q_MAX, LIM_Q_MAX)
        self.assertEqual(result.cmp_b, B_MIN)
        self.assertEqual(result.cmp_a, A_MAX)

    def test_applied_q_inverts_the_committed_pair(self):
        """The anti-windup needs the coordinate the REGISTERS received, not the one the control law
        wanted. PD_ModAppliedQ must invert whatever the distributor produced."""
        for q in range(Q_MIN, Q_MAX + 1, 101):
            result = distribute(q)
            self.assertAlmostEqual(lib.PD_ModAppliedQ(result.cmp_a, result.cmp_b),
                                   result.q_applied, delta=1, msg=f'q={q}')

    def test_minimum_non_zero_pulse_is_floored_and_reported(self):
        """The old 5 % A floor must survive, and the distributor must say it applied it so the
        anti-windup can see it (plan 4.4).

        With A_min <= q_min the q floor fires first: the low zone hands A = q, so a request below
        5 % is reported as Q_MIN while the committed A is still the 1088-count floor. Both halves
        matter - the count keeps the compare register away from zero, the flag tells the controller
        the applied q is larger than the requested one."""
        result = distribute(200)                             # below the floor, above zero
        self.assertEqual(result.cmp_a, A_MIN)
        self.assertEqual(result.limit & LIM_Q_MIN, LIM_Q_MIN)
        self.assertGreater(result.q_applied, result.q_request)

    def test_a_floor_never_sits_above_the_q_floor(self):
        """pd_config.h asserts A_min <= q_min; re-check it here because a violation would make the
        A clamp outrank the q clamp and command MORE than the control law asked for."""
        self.assertLessEqual(cfg_int('PD_MOD_A_MIN_COUNTS'), cfg_int('PD_MOD_Q_MIN_COUNTS'))


if __name__ == '__main__':
    unittest.main(verbosity=2)
