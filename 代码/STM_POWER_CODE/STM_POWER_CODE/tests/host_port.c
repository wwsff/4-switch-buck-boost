/* Test-only board boundary. Linked only into the native test DLL, never Keil. */
#include "pd_cmp.h"
#include "pd_config.h"
#include "pd_core.h"
#include "pd_diag.h"
#include "pd_oc.h"
#include "pd_power.h"
#include "pd_zero.h"
#include <string.h>
static uint32_t now, pins[2];
static uint32_t fast_age_ms;
static uint8_t bytes[65536];
static uint32_t used;
static uint32_t tx_busy, tx_priority;
void Test_TxBusy(uint32_t on) { tx_busy = on; }
uint32_t Test_TxPriority(void) { return tx_priority; }
/* Power-stage port recording. The native tests assert on these. */
static uint32_t power_en, gates, gate_test_index, gate_test_en;
static uint16_t cmp_in, cmp_out, gate_test_cmp;
static uint32_t wait_periods_calls, last_wait_periods;
/* Bridge-mode history: 1 = static low side, 0 = PWM. */
static uint32_t bridge_static[2], bridge_high_out2[2];
/* Every compare value ever written, so a test can prove none is illegal. */
static uint16_t cmp_seen_min, cmp_seen_max;
/* Largest A-bridge value written OUTSIDE the gate test: the normal-modulation
 * ceiling only (PD_CMP_A_MAX). The gate test keeps its own 20 %/80 % pair. */
static uint16_t cmp_a_max;
static uint32_t cmp_illegal, cmp_writes;
/* Fault injection: make the port report failure without touching the values. */
static uint32_t fail_compare, fail_wait;
static uint32_t fault_in_wait, fault_on_gates, invalid_bits, unhealthy;
static uint32_t first_state, first_en, first_seen, unsafe_enable;
void PD_PortStartMark(void) { first_seen = 0; }
/* pd_board.c owns the real ADC-fault debounce; the host build links everything except that file,
 * so the clear path is stubbed here. It has to exist: pd_core.c calls it from CLEAR_FAULT, and
 * without the stub the host build fails to link - which is exactly how this was found. */
void PD_BoardClearAdcFault(void) { }
void PD_BoardClearIlRun(void) { }
/* Also owned by pd_board.c. The host build has no ADC ISR, so the report is a fixed "healthy"
 * answer; the point of the stub is that pd_power.c links, and that the counters it reports come
 * from the same place the real one reads. */
/* pd_board.c owns this; the host build has no injected ADC, so nothing is ever over the limit.
 * The tests drive the trip through Test_Sample() instead. */
uint32_t PD_PortIlOverRun(void) { return 0u; }
uint32_t PD_PortSampleFailMask(uint32_t *fast_age, uint32_t *slow_age0, uint32_t *slow_age1,
                               uint32_t *input_age)
{
    *fast_age = 0u;
    *slow_age0 = 0u;
    *slow_age1 = 0u;
    *input_age = 0u;
    return 0u;
}
void PD_PortFaultSnapshot(uint8_t ch, uint32_t fault)
{
    (void)fault;
    if ((ch == 0 || ch == 1) && !first_seen)
    {
        first_seen = 1;
        first_state = pd_channels[0].state;
        first_en = power_en;
    }
}
uint32_t Test_FirstState(void) { return first_state; }
uint32_t Test_FirstEnable(void) { return first_en; }
uint32_t Test_UnsafeEnable(void) { return unsafe_enable; }
void Test_FaultInWait(uint32_t on) { fault_in_wait = on; }
void Test_FaultOnGates(uint32_t on) { fault_on_gates = on; }
void Test_Invalid(uint32_t mask) { invalid_bits = mask; pd_channels[0].valid &= (uint16_t)~mask; }
void Test_Unhealthy(uint32_t on) { unhealthy = on; }
uint32_t PD_PortNow(void)
{
    return now;
}
uint32_t PD_PortLock(void)
{
    return 0;
}
void PD_PortUnlock(uint32_t key)
{
    (void)key;
}
int PD_PortTransmit(const uint8_t *p, uint16_t n, uint8_t priority)
{
    tx_priority = priority;
    if (tx_busy) return 0;
    if (used + n > sizeof(bytes))
        return 0;
    memcpy(bytes + used, p, n);
    used += n;
    return 1;
}
void PD_PortStop(uint8_t ch)
{
    if (ch == 0 || ch == 1) PD_ZeroCancel();
    if (ch == 0 || ch == 1) { power_en &= (uint32_t)~1u; gates = 0; }
    if (ch == 0 || ch == 2) { power_en &= (uint32_t)~2u; gates &= (uint32_t)~0xf0u; }
}
void PD_PortPowerEnable(uint8_t ch, uint8_t on)
{
    uint32_t bit = ch == 1 ? 1u : 2u;
    if (on && pd_channels[0].fault) unsafe_enable++;
    if (on) power_en |= bit; else power_en &= (uint32_t)~bit;
}
void PD_PortGatesEnable(uint8_t ch, uint8_t on)
{
    uint32_t mask = ch == 1 ? 0x0fu : 0xf0u;
    if (on) gates |= mask; else gates &= (uint32_t)~mask;
    if (on && fault_on_gates)
    {
        fault_on_gates = 0;
        PD_FastSample(1, 1234, 0, 20000, 0, 1);
        PD_LatchFault(1, PD_FAULTS_OVERCURRENT);
    }
}
/* Simulated HRTIM master counter + the same window/retry policy the firmware
   uses (pd_cmp.c is linked into this DLL too, so the tests drive the real
   decision code, not a copy). Modes:
     0 normal: the counter walks into the window and the commit succeeds
     1 the window closes once inside the masked section, then the next period works
     2 the counter never enters the window (stopped/gated) -> wait guard expires
     3 a fault lands while waiting -> stop wins, nothing is written
     4 the masked re-check always fails -> bounded retries are exhausted */
static uint16_t sim_counter, sim_step = 170u;
static uint32_t sim_mode, sim_close_once;
static uint16_t sim_next(void)
{
    sim_counter = (uint16_t)(sim_counter + sim_step);
    if (sim_counter >= (uint16_t)PD_SW_PERIOD) sim_counter = (uint16_t)(sim_counter - PD_SW_PERIOD);
    return sim_counter;
}
void Test_CmpMode(uint32_t mode) { sim_mode = mode; sim_close_once = (mode == 1); }
void Test_CmpCounter(uint16_t value, uint16_t step) { sim_counter = value; sim_step = step; }
uint8_t PD_PortSetCompare(uint8_t ch, uint16_t a, uint16_t b)
{
    uint32_t attempts = 0, guard;
    uint16_t m;
    if (ch != 1) { PD_CmpNoteParam(); return PD_CMP_ERR_PARAM; }
    if (fail_compare) { PD_CmpNoteTimeout(0, 0); return PD_CMP_ERR_TIMEOUT; }
    for (;;)
    {
        guard = PD_CMP_WAIT_GUARD;
        do
        {
            /* mode 2: the counter is stopped/gated at the period value, which is
               permanently OUTSIDE the commit window, so the wait guard expires. */
            m = sim_mode == 2 ? (uint16_t)PD_SW_PERIOD : sim_next();
        } while (!PD_CmpWindowAccepts(m) && --guard);
        if (!guard) { PD_CmpNoteTimeout(m, 1); return PD_CMP_ERR_TIMEOUT; }
        if (sim_mode == 3)
        {
            sim_mode = 0;
            PD_LatchFault(1, PD_FAULTS_OVERCURRENT);
        }
        if (pd_channels[0].fault) { PD_CmpNoteStopped(m); return PD_CMP_ERR_STOPPED; }
        if (sim_mode == 1 && sim_close_once)
        {
            sim_close_once = 0;
            PD_CmpNoteRetry((uint16_t)(PD_CMP_LATEST + 1u), 30u);
            if (!PD_CmpRetryAllowed(++attempts)) { PD_CmpNoteTimeout(m, 0); return PD_CMP_ERR_TIMEOUT; }
            continue;
        }
        if (sim_mode == 4)
        {
            PD_CmpNoteRetry((uint16_t)(PD_CMP_LATEST + 1u), 30u);
            if (!PD_CmpRetryAllowed(++attempts)) { PD_CmpNoteTimeout(m, 0); return PD_CMP_ERR_TIMEOUT; }
            continue;
        }
        cmp_in = a; cmp_out = b;
        cmp_writes++;
        /* PD_CMP_MIN = 96 (0x60) is the documented HRTIM high-resolution floor;
           PD_SW_PERIOD + 1 is the "compare never matches" sentinel used for 100 %.
           Both bounds come from pd_config.h so a period change cannot silently
           turn legal writes into "illegal" ones here. */
        if ((a < PD_CMP_MIN || b < PD_CMP_MIN) ||
            (a > (uint16_t)(PD_SW_PERIOD + 1u) || b > (uint16_t)(PD_SW_PERIOD + 1u)))
            cmp_illegal++;
        if (!cmp_seen_min || a < cmp_seen_min) cmp_seen_min = a;
        if (!cmp_seen_min || b < cmp_seen_min) cmp_seen_min = b;
        if (a > cmp_seen_max) cmp_seen_max = a;
        if (b > cmp_seen_max) cmp_seen_max = b;
        /* The A-bridge ceiling only constrains the NORMAL restricted-buck
           modulation: the gate test deliberately drives 20 %/80 % and must not be
           clamped, which is why the clamp lives in commit_duties() and not in the
           port. Record the normal-modulation maximum separately. */
        if (!gate_test_en && a > cmp_a_max) cmp_a_max = a;
        PD_CmpNoteCommit(m, 30u);
        return PD_CMP_OK;
    }
}
void PD_PortBridgeMode(uint8_t ch, uint8_t high_on_output2, uint8_t static_low_side)
{
    if (ch != 1 && ch != 2) return;
    bridge_static[ch - 1] = static_low_side ? 1u : 0u;
    bridge_high_out2[ch - 1] = high_on_output2 ? 1u : 0u;
}
uint8_t PD_PortWaitPeriods(uint8_t n)
{
    wait_periods_calls++; last_wait_periods = n;
    if (fault_in_wait)
    {
        fault_in_wait = 0;
        PD_LatchFault(1, PD_FAULTS_OVERCURRENT);
    }
    return (uint8_t)(fail_wait ? 0 : 1);
}
uint32_t Test_WaitPeriodsCalls(void) { return wait_periods_calls; }
uint32_t Test_BridgeStatic(uint8_t ch) { return bridge_static[ch - 1]; }
/* Reference triplet as the 1 Hz status line reports it (0.3.20): target, ramp
 * reference, OVP threshold in force, descent supervision flag. */
void Test_Ref(int32_t *tgt, int32_t *ramp, int32_t *ovp, uint32_t *down)
{
    *tgt = PD_PowerTargetMv();
    *ramp = PD_PowerRampMv();
    *ovp = PD_PowerOvpLimitMv();
    *down = PD_PowerDownActive();
}
uint32_t Test_BridgeHighOut2(uint8_t ch) { return bridge_high_out2[ch - 1]; }
uint32_t Test_CompareIllegal(void) { return cmp_illegal; }
uint32_t Test_CompareMaxA(void) { return cmp_a_max; }
uint32_t Test_CompareWrites(void) { return cmp_writes; }
uint32_t Test_CompareMin(void) { return cmp_seen_min; }
uint32_t Test_CompareMax(void) { return cmp_seen_max; }
void Test_FailCompare(uint32_t on) { fail_compare = on; }
void Test_FailWait(uint32_t on) { fail_wait = on; }
uint8_t PD_PortGateTestStart(uint8_t index)
{
    if (index < 1 || index > 4) return 0;
    PD_PortStop(1);
    PD_PortBridgeMode(index <= 2 ? 1 : 2, 0, 0);
    if (!PD_PortWaitPeriods(2) || pd_channels[0].fault || pins[0]) return 0;
    gate_test_index = index;
    gate_test_en = 1;
    power_en |= 1u;
    gates |= (index <= 2) ? 0x03u : 0x0cu;   /* whole half-bridge, not one output */
    gate_test_cmp = (index == 1 || index == 3) ? (uint16_t)(PD_SW_PERIOD / 5u)
                                               : (uint16_t)(PD_SW_PERIOD - PD_SW_PERIOD / 5u);
    return 1;
}
uint32_t Test_GateTestCompare(void) { return gate_test_cmp; }
void PD_PortGateTestStop(void)
{
    gate_test_en = 0;
    gates = 0;
    power_en &= (uint32_t)~1u;
}
uint32_t Test_PowerEnable(void) { return power_en; }
uint32_t Test_Gates(void) { return gates; }
void Test_Compare(uint16_t *a, uint16_t *b) { *a = cmp_in; *b = cmp_out; }
uint32_t Test_GateTestIndex(void) { return gate_test_index; }
uint32_t Test_GateTestRunning(void) { return gate_test_en; }
void Test_Sensors(int32_t vin, int32_t vout, int32_t iout)
{
    pd_channels[0].vin_mv = vin;
    pd_channels[0].vout_mv = vout;
    pd_channels[0].iout_ma = iout;
    pd_channels[0].valid |= PD_VALID_BITS_VIN | PD_VALID_BITS_VOUT | PD_VALID_BITS_IOUT;
    pd_channels[0].valid &= (uint16_t)~invalid_bits;
}
void Test_Input(int32_t ma)
{
    pd_channels[0].iin_ma = ma;
    pd_channels[0].valid |= PD_VALID_BITS_IIN;
}
void Test_Advance(uint32_t ms)
{
    now += ms;
    PD_PowerTick();
    PD_GateTestTick();
    PD_Poll();
}
void Test_ZeroApply(uint8_t sensor, uint16_t raw)
{
    PD_ZeroStart(now, sensor);
    for (int i = 0; i < 4096; i++)
        PD_ZeroFeed(now + 500u + (uint32_t)i, 1, 1, raw);
}
uint16_t PD_PortZero(uint8_t action)
{
    if (!action) return 0;
    if (PD_ZeroBusy()) return PD_ERRORS_BUSY;
    if (pd_channels[0].state != PD_STATES_OFF || pd_channels[0].enabled || pd_channels[0].mode)
        return PD_ERRORS_NOT_OFF;
    if (action <= 2) PD_ZeroStart(now, action);
    else PD_ZeroReset(action - 2);
    return 0;
}
uint32_t PD_PortFaultPins(uint8_t ch)
{
    return pins[ch - 1];
}
uint8_t PD_PortSamplingHealthy(void)
{
    return (uint8_t)!unhealthy;
}
uint32_t PD_PortFastAge(void)
{
    return fast_age_ms;
}
void PD_PortDeviceID(char out[24])
{
    memcpy(out, "C-CORE-TEST-NOT-HARDWARE!!", 24);
}
void Test_Reset(void)
{
    now = 0;
    used = 0;
    tx_busy = tx_priority = 0;
    pins[0] = pins[1] = 0;
    power_en = gates = gate_test_index = gate_test_en = 0;
    cmp_in = cmp_out = gate_test_cmp = 0;
    wait_periods_calls = last_wait_periods = 0;
    bridge_static[0] = bridge_static[1] = 0;
    bridge_high_out2[0] = bridge_high_out2[1] = 0;
    cmp_seen_min = cmp_seen_max = cmp_illegal = cmp_writes = 0;
    cmp_a_max = 0;
    fail_compare = fail_wait = 0;
    fault_in_wait = fault_on_gates = invalid_bits = unhealthy = 0;
    first_state = first_en = first_seen = unsafe_enable = 0;
    sim_mode = sim_close_once = 0;
    sim_counter = 0;
    sim_step = 170u;
    fast_age_ms = 0;
    PD_Init();
    PD_OcInit();
    PD_CmpInit();
    pd_channels[0].valid = pd_channels[1].valid = 31;
}
void Test_Tick(void)
{
    PD_PowerTick();
    PD_GateTestTick();
}
void Test_Time(uint32_t value)
{
    now = value;
    PD_Poll();
}
void Test_Pins(uint8_t ch, uint32_t f)
{
    pins[ch - 1] = f;
}
void Test_FastAge(uint32_t ms)
{
    fast_age_ms = ms;
}
void Test_Sample(uint8_t ch, int32_t il)
{
    PD_FastSample(ch, il, 1000, 20000, 0, 1);
}
uint32_t Test_Read(uint8_t *out, uint32_t n)
{
    if (n > used)
        n = used;
    memcpy(out, bytes, n);
    used -= n;
    memmove(bytes, bytes + n, used);
    return n;
}

/* Test-facing renderers for the rotating diagnostic frames (0.3.23). pd_board.c owns the
 * 1 Hz rotation but is HAL-bound and not part of this DLL, so the suite calls the same
 * formatting helpers the firmware calls and gets the identical bytes. */
uint32_t Test_DiagStatus(char *out, uint32_t n, uint8_t state, uint32_t fault, uint16_t duty_in,
                         uint16_t duty_out, uint8_t saturated, int32_t iout_ma, int32_t vout_mv,
                         int32_t vin_mv, int32_t allowed_vmax_mv)
{
    return PD_DiagStatusLine(out, (uint16_t)n, state, fault, duty_in, duty_out, saturated,
                             iout_ma, vout_mv, vin_mv, allowed_vmax_mv);
}
uint32_t Test_DiagTarget(char *out, uint32_t n, int32_t target_mv, int32_t ramp_mv, int32_t ovp_mv,
                         uint8_t down)
{
    return PD_DiagTargetLine(out, (uint16_t)n, target_mv, ramp_mv, ovp_mv, down);
}

uint32_t Test_DiagControl(char *out, uint32_t n, int32_t ref_mv, int32_t vout_f_mv, int32_t ff_cnt,
                          int32_t p_cnt, int32_t i_cnt, int32_t q_cnt, int32_t cmd_cnt,
                          int32_t q_app_cnt, int32_t cmp_out_cnt, uint8_t mod_limit, uint8_t sat)
{
    return PD_DiagControlLine(out, (uint16_t)n, ref_mv, vout_f_mv, ff_cnt, p_cnt, i_cnt, q_cnt,
                              cmd_cnt, q_app_cnt, cmp_out_cnt, mod_limit, sat);
}
/* Read the live control-term snapshot the firmware recorded on its last tick. */
void Test_CtrlTerms(int32_t *ref_mv, int32_t *vout_f_mv, int32_t *ff_cnt, int32_t *p_cnt,
                    int32_t *i_cnt, int32_t *q_cnt, int32_t *cmd_cnt, int32_t *q_app_cnt,
                    int32_t *cmp_out_cnt, uint32_t *mod_limit, uint32_t *mod_zone, uint32_t *sat)
{
    PD_CtrlTerms t;
    PD_PowerCtrlTerms(&t);
    *ref_mv = t.ref_mv;
    *vout_f_mv = t.vout_f_mv;
    *ff_cnt = t.ff_cnt;
    *p_cnt = t.p_cnt;
    *i_cnt = t.i_cnt;
    *q_cnt = t.q_cnt;
    *cmd_cnt = t.cmd_cnt;
    *q_app_cnt = t.q_app_cnt;
    *cmp_out_cnt = t.cmp_out_cnt;
    *mod_limit = t.mod_limit;
    *mod_zone = t.mod_zone;
    *sat = t.sat;
}
