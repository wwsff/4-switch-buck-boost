/* CH1 restricted buck power stage: voltage outer loop + slow current limit.
 *
 * Everything in this file runs in the 1 kHz task (PD_PowerTick) or in the
 * protocol dispatch (PD_GateTest), except PD_PowerAbort, which is called by
 * the fault ISR after disabling the port outputs.
 */
#include "pd_power.h"
#include "pd_board.h"
#include "pd_cmp.h"
#include "pd_config.h"
#include "pd_core.h"
#include "pd_modulation.h"
#include "pd_zero.h"
#include <stdio.h>
#include <string.h>

/* Full scale of the CONTROLLER's own coordinate (plan phase A).
 *
 * In legacy mode the loop output is the A-bridge duty and its ceiling is
 * PD_DUTY_MAX = 0.70. In linked mode the loop output is q - the equivalent A duty
 * at B = 75 % - whose useful range is wider: q reaches 0.875 when B is down at
 * its 0.60 floor. Clamping q at 0.70 would make the whole upper region
 * unreachable and would put the ceiling in two places, so the controller's full
 * scale follows the mode:
 *
 *   legacy:  PD_DUTY_MAX          (A bridge, B fixed)
 *   linked:  PD_MOD_Q_MAX_DUTY    (q, per the released experimental range)
 *
 * The integrator is bounded by the same value: it must be able to reach the
 * actuator's full range and no further, and the direction-aware anti-windup
 * handles the rest. */
#if PD_MOD_LINKED
#define PD_CTRL_D_MAX PD_MOD_Q_MAX_DUTY
#else
#define PD_CTRL_D_MAX PD_DUTY_MAX
#endif
/* ... and the same full scale in COUNTS, for the code that compares a committed register pair
 * against the end of the range (the regulation-failure backstop). One definition, so the two
 * cannot disagree about where the actuator stops: PD_MOD_Q_MAX_COUNTS is asserted equal to
 * 0.875 * PD_SW_PERIOD in pd_config.h. */
#if PD_MOD_LINKED
#define PD_CTRL_LIMIT_CNT ((int32_t)PD_MOD_Q_MAX_COUNTS)
#else
#define PD_CTRL_LIMIT_CNT ((int32_t)PD_CMP_A_MAX)
#endif

/* Three DIFFERENT quantities, deliberately kept apart (0.3.20):
 *
 *   target_mv     the last ACCEPTED user target. Only a range-checked
 *                 SET_TARGET (or the boot default) writes it.
 *   ramp_mv       the reference the control loop executes right now. It is
 *                 driven ONLY by commands: the soft-start up-ramp, the
 *                 regulation down-ramp and the stop ramp. A measured VOUT never
 *                 raises it; the only place a measurement seeds it is the
 *                 precharge->PWM transition, and there it is clamped to the
 *                 target so the reference can never start above the setpoint.
 *   ovp_limit_mv  the threshold the protection check actually used this cycle,
 *                 recomputed from (target, ramp) at the top of every tick.
 *
 * Consequence: while the reference is still coming down after a smaller target
 * was typed in, the OVP follows the REFERENCE down (max(target, ramp)) instead
 * of jumping to the new target's much lower threshold one tick after the
 * command. */
typedef struct
{
    uint8_t state, pwm_enabled;
    uint32_t state_at, oc_at, reg_at, log_at, ovp_at, in_at, ioc_at;
    float ramp_mv, cc_drop_mv, integ, filt;
    float vout_f, iout_f, vin_f;
    uint16_t cmp_in, cmp_out;
    int32_t target_mv;
    int32_t ovp_limit_mv;
    uint8_t down_active, down_band_cnt;
    int32_t down_from_mv, down_target_mv;
    uint32_t down_start_at, down_deadline;
    /* Input-sag supervision (0.3.22): the accepted target no longer fits the
       range the MEASURED VIN allows. Independent timer - it must never share or
       clear the input-window or input-overcurrent timers. */
    uint32_t range_at;
    /* Control-term instrumentation (0.3.25 step 1). Deliberately NOT part of the control
       path: the loop reads none of these back, it only copies values it already computed,
       so adding them cannot change behaviour. */
    PD_CtrlTerms terms;
    /* Set by the duty path when it limits the output this tick (0.3.26). The anti-windup
       uses it so "limited" means the FINAL actuator limit, not only the one in this file. */
    uint8_t clamped;
} Power;

static Power p;
static uint8_t gate_index;
static uint32_t gate_until;

static void put16(uint8_t *p16, uint16_t v)
{
    p16[0] = (uint8_t)v;
    p16[1] = (uint8_t)(v >> 8);
}
static uint16_t gate_status(uint8_t *reply, uint32_t now)
{
    reply[0] = 1u; /* fixed, previously bench-verified hardware wiring */
    reply[1] = PD_GATE_MAP_DESIGNED;
    reply[2] = gate_index;
    reply[3] = (uint8_t)(gate_until ? 1 : 0);
    put16(reply + 4, gate_index ? (uint16_t)(gate_until > now ? gate_until - now : 0) : 0);
    return PD_GATE_TEST_STATUS_SIZE;
}
static uint8_t in_inv(void) { return (uint8_t)((PD_GATE_MAP_DESIGNED & PD_GATE_MAP_IN_SWAP) != 0); }
static uint8_t out_inv(void) { return (uint8_t)((PD_GATE_MAP_DESIGNED & PD_GATE_MAP_OUT_SWAP) != 0); }
/* Compare value for a requested TA1/TB1 duty. The HRTIM high-resolution mode
 * (MUL32, CKPSC = 0) documents a minimum compare value of 0x60 = PD_CMP_MIN:
 * below it the compare event may not be generated at all, which leaves the
 * high-side output set for the whole period (100 % duty). "High side off" is
 * therefore NOT expressed with a compare value - the bridge is put into the
 * static low-side state instead (PD_PortBridgeMode). */
static uint16_t ratio_cmp(float duty)
{
    float c;
    if (duty <= 0.0f)
        return PD_CMP_MIN;
    if (duty >= 1.0f)
        return (uint16_t)(PD_SW_PERIOD + 1u); /* compare never matches: high side stays on */
    c = duty * (float)PD_SW_PERIOD + 0.5f;
    if (c < (float)PD_CMP_MIN)
        c = (float)PD_CMP_MIN;
    if (c > (float)(PD_SW_PERIOD - PD_CMP_MIN))
        c = (float)(PD_SW_PERIOD - PD_CMP_MIN);
    return (uint16_t)c;
}
/* The single register-write path. Both modulators end here, so the stop priority,
 * the stage tagging, the last-line register guards and the publication of
 * p.cmp_in/p.cmp_out cannot diverge between legacy and linked mode.
 *
 * ca/cb are COMPARE COUNTS, not duties: linked mode gets its pair from the
 * distributor and must reach the registers with the exact counts the distributor
 * produced, otherwise PD_ModAppliedQ() would invert a pair that was never
 * written and the anti-windup would be judging a fiction. */
static uint8_t commit_counts(uint16_t ca, uint16_t cb, uint8_t stage)
{
    uint8_t reason;
    if (p.state == PD_STATES_OFF || p.state == PD_STATES_FAULT_LATCHED)
        return PD_CMP_ERR_STOPPED;
    /* Last line against the two register rules that have no exceptions: never
       below PD_CMP_MIN (the HRTIM high-resolution floor, below which the compare
       event may not be generated at all), and never above the A-bridge ceiling. */
    if (ca < (uint16_t)PD_CMP_MIN)
        ca = (uint16_t)PD_CMP_MIN;
    if (ca > (uint16_t)PD_CMP_A_MAX)
        ca = (uint16_t)PD_CMP_A_MAX;
    if (cb < (uint16_t)PD_CMP_MIN)
        cb = (uint16_t)PD_CMP_MIN;
    PD_CmpSetStage(stage);
    reason = PD_PortSetCompare(1, ca, cb);
    if (reason != PD_CMP_OK)
        return reason;
    /* Stop-priority: a fault may have latched between the write and here. The
       port already disabled the outputs; do not publish a duty for a stage that
       is no longer running. */
    if (pd_channels[0].fault || p.state == PD_STATES_FAULT_LATCHED)
        return PD_CMP_ERR_STOPPED;
    p.cmp_in = ca;
    p.cmp_out = cb;
    return PD_CMP_OK;
}
/* d_in / d_out are the PHYSICAL high-side duties of the two bridges; the port
 * maps them onto OUT1/OUT2 through the confirmed gate mapping, so the duty is
 * never inverted here. A legal compare value is not automatically a usable
 * MOSFET pulse: the driver propagation delay plus the gate charge need roughly
 * PD_DUTY_MIN of on-time, so a non-zero request is floored there.
 *
 * Returns PD_CMP_OK, or a PD_CMP_ERR_* reason. The caller decides whether the
 * reason is a fault (timeout/param) or a legitimate stop (PD_CMP_ERR_STOPPED):
 * a stop that won the race must never be turned into a fault nor re-enabled.
 * This is the LEGACY modulator (A regulated, B fixed at 75 %); linked mode goes
 * through PD_ModDistribute() and commit_counts() instead. It is compiled only when
 * PD_MOD_LINKED = 0, so the fallback build proves it still works instead of leaving
 * dead code that only looks maintained. */
#if !PD_MOD_LINKED
static uint8_t commit_duties(float d_in, float d_out, uint8_t stage)
{
    uint16_t ca, cb;
    if (d_in > 0.0f && d_in < PD_DUTY_MIN)
        d_in = PD_DUTY_MIN;
    ca = ratio_cmp(d_in);
    /* Single INTEGER duty ceiling for the input bridge (0.3.22). Clamping at the
       last place before the register write means no caller - PI output, feed-
       forward, precharge hand-off or a future mode - can program an A compare
       value above PD_CMP_A_MAX, so the A/B edge separation cannot collapse
       because of a rounding difference somewhere else. commit_counts() applies
       it too; it stays here as well so the float-to-count conversion is bounded
       before it is handed on. */
    if (ca > (uint16_t)PD_CMP_A_MAX)
        ca = (uint16_t)PD_CMP_A_MAX;
    cb = ratio_cmp(d_out);
    return commit_counts(ca, cb, stage);
}
#endif
/* Report a commit failure in one place. PD_CMP_ERR_STOPPED is not a fault: the
 * stage was stopped by something else (fault, STOP command) and the remaining
 * code must simply not touch the outputs. */
static void commit_failed(uint8_t reason, const char *what)
{
    char line[96];
    if (reason == PD_CMP_OK || reason == PD_CMP_ERR_STOPPED)
        return;
    /* The cause is a complete literal PER BRANCH, not a %s argument: the width of
       a %s cannot be bounded without running the firmware, and 0.3.22 shipped this
       line at 102 characters, i.e. past the 96-byte log field, so the counters it
       carries were silently lost. Each branch is now 89 characters at the widest. */
    if (reason == PD_CMP_ERR_PARAM)
        (void)snprintf(line, sizeof(line),
                       "%s: HRTIM gated/invalid chan; st=%u rt=%u to=%u gd=%u M=%u",
                       what, PD_CmpStage(), PD_LOG_VALUE(PD_CmpRetries()),
                       PD_LOG_VALUE(PD_CmpTimeouts()), PD_LOG_VALUE(PD_CmpGuardExpired()),
                       PD_LOG_VALUE(PD_CmpLastCounter()));
    else
        (void)snprintf(line, sizeof(line),
                       "%s: window retry limit; st=%u rt=%u to=%u gd=%u M=%u",
                       what, PD_CmpStage(), PD_LOG_VALUE(PD_CmpRetries()),
                       PD_LOG_VALUE(PD_CmpTimeouts()), PD_LOG_VALUE(PD_CmpGuardExpired()),
                       PD_LOG_VALUE(PD_CmpLastCounter()));
    PD_LatchFault(1, PD_FAULTS_INTERNAL);
    PD_Log(1, 2, 16, line);
}
static void set_state(uint8_t s)
{
    p.state = s;
    p.state_at = PD_PortNow();
}
static float limit_current_ma(void)
{
    float iref = (float)pd_channels[0].iref_ma;
    if (iref < 50.0f)
        iref = 50.0f; /* a 0 mA "limit" is not physically reachable */
    return iref;
}
/* Feed-forward duty for the restricted buck. Ideal: VOUT = VIN * d / 0.75.
 * The real stage loses the dead time AND the measured gate-edge skew (0.3.22:
 * 120 ns turn-on / 55 ns turn-off), so the model inverts
 *   VOUT = VIN * (d - d_model) / (0.75 - d_model)
 * instead of over-asking the duty by d_model = PD_MODEL_DT_COUNTS/PD_SW_PERIOD
 * (0.0545 = 5.45 % of the period). With that correction the feed-forward lands
 * on the right operating point at 18 V, so the integrator does not have to make
 * up several percent on its own - which matters because the ceiling is only
 * 0.70. The register checks never use this figure (see pd_config.h).
 *
 * LINKED MODE: the same expression, with d read as q. That is not a
 * coincidence and not an approximation - q is DEFINED as the A-bridge duty the
 * stage would need if B were still at 75 %, so the transfer model, the measured
 * k_v and this inverse all carry over unchanged, and the distributor turns q
 * into whatever pair the current B actually is. The only difference is the
 * ceiling: PD_CTRL_D_MAX (0.875 = the top of the released q range) instead of
 * the A-bridge 0.70, because above q = 0.75 the ceiling belongs to the B floor
 * and clamping the feed-forward at 0.70 would leave the whole boost region
 * without feed-forward and force the integrator to find it. */
static float duty_ff(float vref_mv)
{
    float vin = p.vin_f > 1000.0f ? p.vin_f : 1000.0f;
    const float d_model = (float)PD_MODEL_DT_COUNTS / (float)PD_SW_PERIOD;
    float d = vref_mv * (PD_OUT_BRIDGE_DUTY - d_model) / vin + d_model;
    if (d < 0.0f)
        d = 0.0f;
    if (d > PD_CTRL_D_MAX)
        d = PD_CTRL_D_MAX;
    return d;
}
/* Available output range for this input voltage (0.3.23, MEASURED model).
 *
 * 0.3.22 used the topology model and OVERPREDICTED the stage by about 8 %: the
 * loop then saturated the A-bridge at the 15232 ceiling with VOUT stuck ~1 V
 * below the target, which ran 17 V silently saturated and made 18 V latch
 * PD_FAULTS_INTERNAL through the regulation-failure backstop. The bench transfer
 * (204 unsaturated samples, VIN = 19.83 V) is a straight line with ~zero offset
 * whose gain is 91.7 % of the ideal VIN/B, i.e. VOUT_max = 0.856 * VIN; the 18 V
 * point at 21.0 V in measured 17.891 V, inside 0.5 % of that line.
 *
 * So:  allowed = min(PD_BUCK_VMAX_MV, floor(VIN * PD_RANGE_GAIN) - PD_RANGE_MARGIN_MV)
 * with 64-bit intermediates. The topology model is kept as an UPPER BOUND: if the
 * measured line ever came out above it the fit would be wrong (or the duty
 * ceiling changed), so return the structural bound in that case and let the host
 * suite/host bench catch it. Every failure mode returns 0, which callers must
 * read as "no usable range", never as "at least the minimum". */
/* Precondition of the range model, enforced by the COMPILER rather than by a
 * comment: PD_RANGE_GAIN_* was fitted from uncalibrated VOUT telemetry, so it must be
 * re-measured once the voltage calibration is in force. See pd_config.h. */
#if PD_CALIBRATED && (PD_RANGE_MEASURED_CAL == 0)
#error "PD_CALIBRATED=1 needs PD_RANGE_MEASURED_CAL >= 1: the ratio was fitted from UNCALIBRATED VOUT telemetry. Either re-measure the transfer line with the calibrated firmware (1) or recompute it from the fit and show the effect is negligible (2)."
#endif
#if (PD_RANGE_MEASURED_CAL < 0) || (PD_RANGE_MEASURED_CAL > 2)
#error "PD_RANGE_MEASURED_CAL must be 0 (uncalibrated fit), 1 (re-measured) or 2 (recomputed)."
#endif

int32_t PD_PowerAllowedVmaxMv(int32_t vin_mv)
{
    if (vin_mv < PD_VIN_MIN_MV || vin_mv > PD_VIN_MAX_MV)
        return 0;
#if PD_RANGE_PROBE
    /* The model deliberately does not answer on this build - see PD_RANGE_PROBE in pd_config.h.
       ONE branch here covers all five callers: SET_TARGET, PD_PowerStartable(), the start precheck,
       the REGULATION FAILED log line and the runtime range servo. Overriding them individually is
       the scattered edit that produced three half-fixes in a row. */
    return PD_BUCK_VMAX_MV;
#else
    int64_t measured, structural, span, ceiling;
    /* PD_RANGE_GAIN_ACTIVE_NUM is the measured line expressed in the ACTIVE coordinate: with
       PD_MOD_LINKED it is 1068 (the q ceiling 0.875), otherwise 856 (the A ceiling 0.70).
       pd_config.h derives the relation between the two and marks the linked value as an
       extrapolation to be re-measured in phase B. */
    measured = ((int64_t)vin_mv * (int64_t)PD_RANGE_GAIN_ACTIVE_NUM) / (int64_t)PD_RANGE_GAIN_DEN -
               (int64_t)PD_RANGE_MARGIN_MV;
    /* Topology upper bound in the ACTIVE coordinate: the same VOUT = VIN*(x - dt)/(0.75 - dt)
       relation, evaluated at the ceiling of whichever coordinate the loop drives. */
#if PD_MOD_LINKED
    ceiling = (int64_t)PD_MOD_Q_MAX_COUNTS;
#else
    ceiling = (int64_t)PD_CMP_A_MAX;
#endif
    span = (int64_t)PD_OUT_BRIDGE_CMP - (int64_t)PD_MODEL_DT_COUNTS;
    if (span > 0 && ceiling > (int64_t)(PD_RANGE_RESERVE_COUNTS + PD_MODEL_DT_COUNTS))
        structural = ((int64_t)vin_mv * (ceiling - (int64_t)PD_RANGE_RESERVE_COUNTS -
                                         (int64_t)PD_MODEL_DT_COUNTS)) / span -
                     (int64_t)PD_RANGE_LOSS_MARGIN_MV;
    else
        structural = measured;
    /* The measured relation must not be more permissive than what the duty
       ceiling can deliver; if it is, trust the structure, not the fit. */
    if (measured > structural)
        measured = structural;
    if (measured <= 0)
        return 0;
    if (measured > (int64_t)PD_BUCK_VMAX_MV)
        measured = (int64_t)PD_BUCK_VMAX_MV;
    return (int32_t)measured;
#endif
}
/* Minimum input voltage at which a target becomes settable, for log messages.
 * Inverse of the measured line plus the margin:  target = VIN*gain - margin
 *   -> VIN = ceil((target + margin)/gain). Returns 0 if it exceeds 30000. */
int32_t PD_PowerRequiredVinMv(int32_t target_mv)
{
    int64_t need;
    if (target_mv <= 0)
        return 0;
    need = ((int64_t)(target_mv + PD_RANGE_MARGIN_MV) * (int64_t)PD_RANGE_GAIN_DEN +
            (int64_t)PD_RANGE_GAIN_ACTIVE_NUM - 1) /
           (int64_t)PD_RANGE_GAIN_ACTIVE_NUM;
    if (need > 30000)
        return 0;
    return (int32_t)need;
}
/* Whole-millivolt view of the ramp, used by the protection arithmetic. The ramp
 * is always >= 0, so rounding to nearest is well defined. */
static int32_t ramp_int(void)
{
    return (int32_t)(p.ramp_mv + 0.5f);
}
/* Threshold that tracks the reference actually being executed (0.3.20).
 *
 *   guard_ref_mv    = max(target_mv, ramp_mv)
 *   tracking_ovp_mv = max(guard * 115/100, guard + 1500)
 *   ovp_limit_mv    = min(tracking_ovp_mv, PD_OVP_ABS_MAX_MV)
 *
 * The max() is the whole point: while the ramp is still above the new (lower)
 * target the guard IS the ramp, so the threshold descends with the reference
 * instead of collapsing to the new target's threshold one tick after the
 * command, while the output is still at the old voltage. The absolute ceiling is
 * exactly the highest threshold this firmware already used, so nothing is
 * weakened: the reference can only narrow the threshold, never widen it beyond
 * the specified output range. */
static int32_t ovp_for(int32_t target_mv, int32_t ramp_mv)
{
    int32_t guard = target_mv > ramp_mv ? target_mv : ramp_mv;
    int32_t tracking;
    if (guard < 0)
        guard = 0;
    tracking = guard * PD_OVP_RATIO_NUM / PD_OVP_RATIO_DEN;
    if (guard + PD_OVP_MARGIN_MV > tracking)
        tracking = guard + PD_OVP_MARGIN_MV;
    if (tracking > PD_OVP_ABS_MAX_MV)
        tracking = PD_OVP_ABS_MAX_MV;
    return tracking;
}
/* How long a descent may take before the stage is stopped: the ramp time for the
 * span that actually has to be traversed plus a settling allowance, floored.
 * Never a flat 10 ms - the output bank (3x220 uF) and the load cannot follow an
 * arbitrary step, and with no load the caps hold their charge for tens of
 * seconds. */
static uint32_t down_budget_ms(int32_t span_mv)
{
    uint32_t ms;
    if (span_mv < 0)
        span_mv = 0;
    ms = (uint32_t)((float)span_mv / (float)PD_RUN_RAMP_DOWN_MV_MS) + PD_DOWN_SETTLE_MS;
    if (ms < PD_DOWN_TIMEOUT_MIN_MS)
        ms = PD_DOWN_TIMEOUT_MIN_MS;
    return ms;
}
/* Ramp update: COMMAND-DRIVEN ONLY. Runs before the protection check so the
 * threshold and the control output of a tick use the same reference. No measured
 * value ever raises the ramp; the only place a measurement seeds it is the
 * precharge->PWM transition, and there it is clamped to the target. */
static void update_ramp(void)
{
    if (p.state == PD_STATES_BOOTSTRAP_PRECHARGE)
        return; /* seeded from the measured output when PWM is enabled */
    if (p.state == PD_STATES_STOPPING)
    {
        /* Shutdown rate: a SEPARATE parameter from the regulation down rate, so
           the two can be tuned independently. Both were the same constant
           before 0.3.20 and the value is unchanged. */
        p.ramp_mv -= (float)PD_STOP_RAMP_MV_MS;
        if (p.ramp_mv < 0.0f)
            p.ramp_mv = 0.0f;
        return;
    }
    if (p.ramp_mv < (float)p.target_mv)
    {
        p.ramp_mv += (float)p.target_mv / (float)PD_SS_MS;
        if (p.ramp_mv > (float)p.target_mv)
            p.ramp_mv = (float)p.target_mv;
    }
    else if (p.ramp_mv > (float)p.target_mv)
    {
        p.ramp_mv -= (float)PD_RUN_RAMP_DOWN_MV_MS;
        if (p.ramp_mv < (float)p.target_mv)
            p.ramp_mv = (float)p.target_mv;
    }
}
/* Take the user target and start/stop the descent supervision. An out-of-window
 * value is IGNORED: SET_TARGET already rejects it for the buck channel, and this
 * second guard guarantees no path can push the ramp or the threshold up with a
 * value the power stage is not allowed to produce. */
static void update_target(uint32_t now)
{
    PD_Channel *c = &pd_channels[0];
    int32_t tgt = c->vref_mv;
    int32_t ramp = ramp_int();
    char line[128];
    if (tgt < PD_BUCK_VMIN_MV || tgt > PD_BUCK_VMAX_MV)
    {
        if (now - p.log_at >= 1000)
        {
            p.log_at = now;
            (void)snprintf(line, sizeof(line),
                           "TARGET IGNORED: %ld mV outside %d..%d mV; ref stays %ld mV", (long)tgt,
                           (int)PD_BUCK_VMIN_MV, (int)PD_BUCK_VMAX_MV, (long)ramp);
            PD_Log(1, 1, 24, line);
        }
        return;
    }
    if (tgt == p.target_mv)
        return; /* a repeated command changes nothing: ramp and deadline stay */
    (void)snprintf(line, sizeof(line), "TARGET: %ld -> %ld mV (ref=%ld VOUT=%ld mV)", (long)p.target_mv,
                   (long)tgt, (long)ramp, (long)c->vout_mv);
    PD_Log(1, 0, 24, line);
    p.target_mv = tgt;
    if (tgt >= ramp)
    {
        /* Nothing to bring down. An ascent is not supervised here; the
           regulation-failure backstop in run_loop() covers a stuck output. */
        p.down_active = 0;
        p.down_band_cnt = 0;
        return;
    }
    if (!p.down_active)
    {
        p.down_active = 1;
        p.down_start_at = now ? now : 1u;
        p.down_from_mv = ramp;
        p.down_band_cnt = 0;
    }
    p.down_target_mv = tgt;
    /* Pure function of (episode start, episode start reference, target), so
       asking for the same target again recomputes the SAME deadline and a
       repeated command can never extend it, while a lower target inside one
       episode gets the time its remaining span needs. Either way the deadline is
       bounded by a span that is itself bounded by the allowed output range. */
    p.down_deadline = p.down_start_at + down_budget_ms(p.down_from_mv - tgt);
    (void)snprintf(line, sizeof(line), "DOWN start: target=%ld ref=%ld VOUT=%ld mV budget=%lu ms",
                   (long)tgt, (long)ramp, (long)c->vout_mv,
                   (unsigned long)(p.down_deadline - p.down_start_at));
    PD_Log(1, 0, 24, line);
}
/* Supervise an in-progress descent. Completion needs BOTH the reference at the
 * target AND the feedback inside PD_DOWN_BAND_MV for PD_DOWN_BAND_SAMPLES in a
 * row: with no load the output caps (RC ~= 70 s) hold their charge, so the
 * reference position alone says nothing about the output. Returns 1 when the
 * stage was stopped. */
static uint8_t down_track(uint32_t now)
{
    PD_Channel *c = &pd_channels[0];
    int32_t ramp = ramp_int();
    int32_t left;
    char line[160];
    /* Supervised while the ramp is being executed (soft start included: the
       reference can already be descending if the target was lowered early).
       PRECHARGE has no reference yet and the STOP ramp has its own goal, so a
       descent episode must not survive into either of them. */
    if (p.state != PD_STATES_SOFT_START && p.state != PD_STATES_RUN_CV &&
        p.state != PD_STATES_RUN_CC)
    {
        p.down_active = 0;
        p.down_band_cnt = 0;
        return 0;
    }
    if (!p.down_active)
        return 0;
    if (ramp > p.down_target_mv)
    {
        /* Still ramping down: report once a second so the state is visible. */
        p.down_band_cnt = 0;
        if (now - p.log_at >= 1000)
        {
            left = (int32_t)(p.down_deadline - now);
            p.log_at = now;
            (void)snprintf(line, sizeof(line),
                           "DOWN in progress: target=%ld ref=%ld VOUT=%ld mV %ld ms left",
                           (long)p.down_target_mv, (long)ramp, (long)c->vout_mv, (long)(left > 0 ? left : 0));
            PD_Log(1, 0, 24, line);
        }
        return 0;
    }
    if (c->vout_mv >= p.down_target_mv - PD_DOWN_BAND_MV &&
        c->vout_mv <= p.down_target_mv + PD_DOWN_BAND_MV)
    {
        if (p.down_band_cnt < 255u)
            p.down_band_cnt++;
    }
    else
        p.down_band_cnt = 0;
    if (p.down_band_cnt >= (uint8_t)PD_DOWN_BAND_SAMPLES)
    {
        p.down_active = 0;
        p.down_band_cnt = 0;
        (void)snprintf(line, sizeof(line), "DOWN done: target=%ld ref=%ld VOUT=%ld mV in %lu ms",
                       (long)p.down_target_mv, (long)ramp, (long)c->vout_mv,
                       (unsigned long)(now - p.down_start_at));
        PD_Log(1, 0, 24, line);
        return 0;
    }
    if ((int32_t)(now - p.down_deadline) < 0)
        return 0;
    /* Could not come down inside the budget. Say so and stop: a stage whose
       output sits above the reference it is supposed to follow is not
       regulating. There is NO fast-discharge trick here - compare values are
       never written as 0/1 and no energy is absorbed from the output caps, so a
       residual output voltage after the stop is EXPECTED. The message must stay
       inside the 96-byte log message field (PD_LOG_MESSAGE_OFFSET). */
    (void)snprintf(line, sizeof(line),
                   "DOWN timeout: tgt=%ld ref=%ld VO=%ld mV %lu ms; residual V stays",
                   (long)p.down_target_mv, (long)ramp, (long)c->vout_mv,
                   (unsigned long)(now - p.down_start_at));
    p.down_active = 0;
    p.down_band_cnt = 0;
    PD_LatchFault(1, PD_FAULTS_INTERNAL);
    PD_Log(1, 2, 24, line);
    return 1;
}
void PD_PowerInit(void)
{
    memset(&p, 0, sizeof(p));
    p.state = PD_STATES_OFF;
    gate_index = 0;
    gate_until = 0;
}
uint8_t PD_GateMappingConfirmed(void) { return 1u; }
uint8_t PD_GateMappingCode(void) { return PD_GATE_MAP_DESIGNED; }
uint8_t PD_PowerActive(uint8_t ch)
{
    return (uint8_t)(ch == 1 && p.state != PD_STATES_OFF && p.state != PD_STATES_FAULT_LATCHED);
}
uint16_t PD_PowerDutyIn(void) { return p.state == PD_STATES_OFF ? 0 : p.cmp_in; }
uint16_t PD_PowerDutyOut(void) { return p.state == PD_STATES_OFF ? 0 : p.cmp_out; }
/* Reference bookkeeping for telemetry (0.3.20): the accepted target, the
 * reference being executed, the threshold in force this cycle and whether a
 * descent is being supervised. Reported in the 1 Hz status line. */
void PD_PowerCtrlTerms(PD_CtrlTerms *out)
{
    if (out)
        *out = p.terms;
}
/* The ONE definition of "the actuator has nothing left", in the coordinate the active
 * modulator drives. Judged on q_app_cnt, the q the COMMITTED pair represents, so it is true
 * only when the registers really are at the end of the released range - not when the A bridge
 * happens to be at its own ceiling with the B bridge still holding travel in reserve.
 *
 * The 8-count tolerance matches the register guard elsewhere in this file: a pair within
 * 8 counts of the limit is treated as being on it, because the next tick cannot move further
 * than that without crossing it. */
uint8_t PD_PowerAtRangeEnd(void)
{
    return (uint8_t)(p.terms.q_app_cnt >= PD_CTRL_LIMIT_CNT - 8);
}
/* Which committed high-side falling edge, if any, sits inside the ADC sampling window
 * (0.3.32). Pure arithmetic on the last committed pair - it reads no hardware, changes nothing
 * and is safe to call from anywhere in the 1 kHz task.
 *
 * The regular-group trigger fires at PD_TE_CMP3 in Timer E's 8 us frame, which is 50 % of the
 * PWM period that contains it: PD_SW_PERIOD / 2 counts after that period's rollover. The
 * falling edge of a bridge with duty d is at d * PD_SW_PERIOD. Both are in the same counts, so
 * the distance is a subtraction.
 *
 * Returns 0x1 for the A bridge, 0x2 for the B bridge, 0x3 for both, 0 for neither; *dist gets
 * the SIGNED distance of the closest of them (positive = the edge is after the sample).
 *
 * It is a REPORT: an edge inside the window is necessary for a biased reading but not
 * sufficient, and nothing in the control law consumes this. */
uint8_t PD_PowerAdcEdgeNear(uint16_t cmp_a, uint16_t cmp_b, int32_t *dist)
{
    const int32_t sample = (int32_t)(PD_TE_CMP3 % PD_SW_PERIOD);
    int32_t da = (int32_t)cmp_a - sample;
    int32_t db = (int32_t)cmp_b - sample;
    uint8_t which = 0u;
    int32_t best = 0;
    if (da < 0)
        da = -da;
    if (db < 0)
        db = -db;
    if (da <= (int32_t)PD_ADC_EDGE_WATCH_COUNTS)
    {
        which |= 0x1u;
        best = (int32_t)cmp_a - sample;
    }
    if (db <= (int32_t)PD_ADC_EDGE_WATCH_COUNTS)
    {
        which |= 0x2u;
        if (!(which & 0x1u) || db < da)
            best = (int32_t)cmp_b - sample;
    }
    if (dist)
        *dist = best;
    return which;
}
int32_t PD_PowerTargetMv(void) { return p.state == PD_STATES_OFF ? 0 : p.target_mv; }
int32_t PD_PowerRampMv(void) { return p.state == PD_STATES_OFF ? 0 : ramp_int(); }
int32_t PD_PowerOvpLimitMv(void) { return p.state == PD_STATES_OFF ? 0 : p.ovp_limit_mv; }
uint8_t PD_PowerDownActive(void) { return p.down_active; }
/* 1 while the accepted target fits the range the latest measured VIN allows.
 * Reported in the 1 Hz status line next to the reference triplet, so a refused
 * command or a later sag can be explained from the log alone. */
uint8_t PD_PowerTargetInRange(void)
{
    int32_t vmax;
    if (p.state == PD_STATES_OFF || p.state == PD_STATES_FAULT_LATCHED)
        return 1;
    vmax = PD_PowerAllowedVmaxMv(pd_channels[0].vin_mv);
    return (uint8_t)(vmax > 0 && p.target_mv <= vmax);
}
void PD_PowerAbort(uint8_t ch)
{
    uint32_t key;
    if (ch != 1 && ch != 0)
        return;
    memset(&p, 0, sizeof(p));
    p.state = PD_STATES_OFF;
    p.state_at = PD_PortNow();
    gate_index = 0;
    gate_until = 0;
    key = PD_PortLock();
    pd_channels[0].enabled = 0;
    pd_channels[0].duty_in = 0;
    pd_channels[0].duty_out = 0;
    pd_channels[0].state = pd_channels[0].fault ? PD_STATES_FAULT_LATCHED : PD_STATES_OFF;
    PD_PortUnlock(key);
}
void PD_PowerStop(uint8_t ch)
{
    if (ch != 1 && ch != 0)
        return;
    PD_PortStop(ch);
    PD_PowerAbort(ch);
}
void PD_PowerRequestStop(void)
{
    if (p.state == PD_STATES_OFF || p.state == PD_STATES_FAULT_LATCHED)
    {
        PD_PowerStop(1);
        return;
    }
    if (p.state == PD_STATES_STOPPING)
        return;
    set_state(PD_STATES_STOPPING);
    pd_channels[0].state = PD_STATES_STOPPING;
    PD_Log(1, 0, 14, "STOP: ramping down before disabling gates");
}
static uint8_t precheck(uint8_t *reason, uint8_t verbose)
{
    const PD_Channel *c = &pd_channels[0];
    int32_t vmax;
    char line[128];
    if (c->state != PD_STATES_OFF || c->enabled)
    {
        *reason = PD_ERRORS_NOT_OFF;
        return 0;
    }
    if (c->fault)
    {
        *reason = PD_ERRORS_FAULT;
        return 0;
    }
    if (c->mode != PD_MODES_NORMAL)
    {
        *reason = PD_ERRORS_NOT_OFF;
        return 0;
    }
    if (PD_ZeroBusy() || gate_index)
    {
        *reason = PD_ERRORS_BUSY;
        return 0;
    }
    if (!PD_PortSamplingHealthy())
    {
        *reason = PD_ERRORS_NOT_READY;
        return 0;
    }
    if ((c->valid & (PD_VALID_BITS_VIN | PD_VALID_BITS_VOUT | PD_VALID_BITS_IOUT | PD_VALID_BITS_IIN)) !=
        (PD_VALID_BITS_VIN | PD_VALID_BITS_VOUT | PD_VALID_BITS_IOUT | PD_VALID_BITS_IIN))
    {
        *reason = PD_ERRORS_NOT_READY;
        return 0;
    }
    if (!PD_ZeroApplied(1))
    {
        *reason = PD_ERRORS_NOT_READY;
        if (verbose) PD_Log(1, 2, 14, "START BLOCKED: CH1 IOUT zero not measured; run ZERO action 1");
        return 0;
    }
    if (c->vin_mv < PD_VIN_MIN_MV || c->vin_mv > PD_VIN_MAX_MV)
    {
        *reason = PD_ERRORS_NOT_READY;
        if (verbose) PD_Log(1, 2, 14, "START BLOCKED: VIN outside configured input window");
        return 0;
    }
    if (c->vref_mv < PD_BUCK_VMIN_MV || c->vref_mv > PD_BUCK_VMAX_MV)
    {
        *reason = PD_ERRORS_RANGE;
        return 0;
    }
    /* Available output range for the MEASURED input voltage (0.3.22). This
       replaces the old fixed "target <= 0.9*VIN" rule, which capped the range at
       17.82 V with 19.8 V in and would have blocked 18 V outright. The same
       function is used by the runtime SET_TARGET check and by the input-sag
       supervision in PD_PowerTick, so all three paths agree by construction. */
    vmax = PD_PowerAllowedVmaxMv(c->vin_mv);
    if (vmax <= 0 || c->vref_mv > vmax)
    {
        *reason = PD_ERRORS_RANGE;
        if (verbose)
        {
            /* Worst case 71 characters per tools/check_log_lengths.py (not
               counted by hand), e.g. "START BLOCKED: tgt 18000 > max 16920 mV at
               VIN 20000; needs 21262". The operator needs the allowed maximum AND
               the required input - the shortfall is what has to change. */
            (void)snprintf(line, sizeof(line),
                           "START BLOCKED: tgt %ld > max %ld mV at VIN %ld; needs %ld",
                           (long)c->vref_mv, (long)vmax, (long)c->vin_mv,
                           (long)PD_PowerRequiredVinMv(c->vref_mv));
            PD_Log(1, 2, 14, line);
        }
        return 0;
    }
    if (c->vout_mv > c->vref_mv + 1000)
    {
        *reason = PD_ERRORS_NOT_READY;
        if (verbose) PD_Log(1, 2, 14, "START BLOCKED: output pre-biased above target; discharge first");
        return 0;
    }
    return 1;
}
uint8_t PD_PowerReady(uint8_t ch)
{
    uint8_t reason = 0;
    return (uint8_t)(ch == 1 && precheck(&reason, 0));
}
uint16_t PD_PowerStart(uint8_t ch)
{
    uint8_t reason = 0, cmp;
    uint32_t key;
    if (ch != 1)
        return PD_ERRORS_UNSUPPORTED;
    if (!precheck(&reason, 1))
        return reason;
    /* Output-bridge phase and the static precharge state are configured while
       every gate output is still disabled. Precharge is NOT "compare = 0": the
       high side is held inactive by its set/reset source selection, so no
       illegal HRTIM compare value is ever programmed. */
    PD_PortBridgeMode(1, in_inv(), 1);
    PD_PortBridgeMode(2, out_inv(), 1);
    set_state(PD_STATES_BOOTSTRAP_PRECHARGE);
    PD_CmpSetStage(PD_CMP_STAGE_START);
    cmp = PD_PortSetCompare(1, PD_CMP_MIN, PD_OUT_BRIDGE_CMP);
    if (cmp != PD_CMP_OK || !PD_PortWaitPeriods(2))
    {
        /* The preloaded values are not guaranteed to be in the compare unit:
           enabling the outputs now would run the boot-time 50 % duty. Abort. */
        PD_PortPowerEnable(1, 0);
        PD_PortGatesEnable(1, 0);
        set_state(PD_STATES_OFF);
        commit_failed(cmp, "POWER START ABORTED");
        if (cmp == PD_CMP_OK)
            PD_Log(1, 2, 14, "POWER START ABORTED: period wait timeout; outputs stay off");
        return PD_ERRORS_NOT_READY;
    }
    key = PD_PortLock();
    if (p.state != PD_STATES_BOOTSTRAP_PRECHARGE || pd_channels[0].fault ||
        PD_PortFaultPins(1) || !PD_PortSamplingHealthy())
    {
        PD_PowerStop(1);
        PD_PortUnlock(key);
        PD_Log(1, 2, 14, "POWER START ABORTED: fault or sampling changed during arming");
        return PD_ERRORS_NOT_READY;
    }
    p.pwm_enabled = 0;
    p.ramp_mv = 0.0f;
    p.target_mv = pd_channels[0].vref_mv;
    p.ovp_limit_mv = ovp_for(p.target_mv, 0);
    p.down_active = 0;
    p.down_band_cnt = 0;
    p.down_from_mv = 0;
    p.down_target_mv = p.target_mv;
    p.down_start_at = 0;
    p.down_deadline = 0;
    p.cc_drop_mv = 0.0f;
    p.integ = 0.0f;             /* bidirectional now: seed at zero, not at a floor */
    p.clamped = 0u;
    p.integ = 0.0f;
    p.filt = 0.0f;
    p.oc_at = 0;
    p.reg_at = 0;
    p.range_at = 0;
    p.cmp_in = PD_CMP_MIN;
    p.cmp_out = PD_OUT_BRIDGE_CMP;
    p.state_at = PD_PortNow();
    pd_channels[0].enabled = 1;
    pd_channels[0].state = PD_STATES_BOOTSTRAP_PRECHARGE;
    pd_channels[0].duty_in = p.cmp_in;
    pd_channels[0].duty_out = p.cmp_out;
    /* Publish state and arm capture before EN rises. An IRQ after unlock may
       stop us, but no remaining start code is allowed to re-enable outputs. */
    PD_TraceAutoArm();
    PD_PortStartMark();
    PD_PortPowerEnable(1, 1);
    PD_PortGatesEnable(1, 1);
    PD_PortUnlock(key);
    if (pd_channels[0].fault || !pd_channels[0].enabled)
        return PD_ERRORS_NOT_READY;
    PD_Log(1, 0, 14, "POWER START: EN high, bootstrap precharge, both low sides on");
    return 0;
}
static uint8_t protect(uint32_t now)
{
    PD_Channel *c = &pd_channels[0];
    /* Threshold computed by the reference layer at the top of this tick, from
       (target, ramp) - NOT from the raw user target. See ovp_for(). */
    int32_t ovp = p.ovp_limit_mv;
    int32_t ilim;
    char line[160];
#if PD_OVERRIDE_NO_LATCH
    /* Bench override: a raised, non-negotiable OVP is the only voltage backstop
       that stays hard. Everything else below is logged and the stage keeps
       running. */
    ovp = PD_OVERRIDE_OVP_MV;
#endif
    if (ovp <= 0)
        ovp = ovp_for(p.target_mv, ramp_int()); /* defensive: never a 0 threshold */
    if (c->vout_mv > ovp)
    {
        if (!p.ovp_at)
            p.ovp_at = now ? now : 1u;
        else if (now - p.ovp_at >= PD_OVP_HOLD_MS)
        {
            (void)snprintf(line, sizeof(line),
                           "OVP: tgt=%ld ref=%ld lim=%ld VO=%ld mV %u ms dn=%u; off",
                           (long)p.target_mv, (long)ramp_int(), (long)ovp, (long)c->vout_mv,
                           (unsigned)PD_OVP_HOLD_MS, p.down_active);
            PD_LatchFault(1, PD_FAULTS_OVERVOLTAGE);
            PD_Log(1, 2, 16, line);
            return 1;
        }
    }
    else
    {
        if (p.ovp_at && now - p.ovp_at < PD_OVP_HOLD_MS && now - p.log_at >= 1000)
        {
            /* A one-off excursion is almost always the gate-enable transient:
               report it, do not act on it. */
            p.log_at = now;
            (void)snprintf(line, sizeof(line),
                           "OVP GLITCH: tgt=%ld ref=%ld lim=%ld VO=%ld mV %lu ms",
                           (long)p.target_mv, (long)ramp_int(), (long)ovp, (long)c->vout_mv,
                           (unsigned long)(now - p.ovp_at));
            PD_Log(1, 1, 16, line);
        }
        p.ovp_at = 0;
    }
    if (c->vin_mv < PD_VIN_MIN_MV || c->vin_mv > PD_VIN_MAX_MV)
    {
        if (!p.in_at)
            p.in_at = now ? now : 1u;
        (void)snprintf(line, sizeof(line),
                       "INPUT FAULT: VIN=%ld outside 17000..23000 mV for %lu ms", (long)c->vin_mv,
                       (unsigned long)(now - p.in_at));
#if PD_OVERRIDE_NO_LATCH
        if (now - p.log_at >= 1000)
        {
            p.log_at = now;
            PD_Log(1, 2, 16, line);
        }
#else
        if (now - p.in_at >= PD_INPUT_HOLD_MS)
        {
            PD_LatchFault(1, PD_FAULTS_INPUT);
            PD_Log(1, 2, 16, line);
            return 1;
        }
#endif
    }
    else
        p.in_at = 0;
    /* Input over-current: hard stop in every build. A bridge fault (shoot
       through, damaged FET, bootstrap collapse) otherwise only shows up as the
       bench supply folding into current limit, with no firmware evidence.
       The limit is adaptive: twice the power-balance estimate plus a margin,
       floored at PD_INPUT_OC_MIN_MA. A fixed value cannot work - a 500 mA
       current-limited supply never reaches 600 mA, and 2 V/0.1 A and 15 V/2 A
       differ by two orders of magnitude. */
    {
        int64_t p_out_uw = (int64_t)c->vout_mv * (int64_t)c->iout_ma;
        int32_t limit = PD_INPUT_OC_MIN_MA;
        if (c->vin_mv > 1000)
        {
            int32_t i_exp_ma = (int32_t)((p_out_uw * 10) / ((int64_t)c->vin_mv * 8)); /* eta = 0.8 */
            limit = i_exp_ma * PD_INPUT_OC_RATIO_NUM / PD_INPUT_OC_RATIO_DEN + PD_INPUT_OC_MARGIN_MA;
            if (limit < PD_INPUT_OC_MIN_MA)
                limit = PD_INPUT_OC_MIN_MA;
        }
        if (c->iin_ma > limit)
        {
            if (!p.ioc_at)
                p.ioc_at = now ? now : 1u;
            else if (now - p.ioc_at >= PD_INPUT_OC_MS)
            {
                (void)snprintf(line, sizeof(line),
                               "INPUT OC: IIN=%ld mA > %ld mA for %u ms; outputs disabled",
                               (long)c->iin_ma, (long)limit, (unsigned)PD_INPUT_OC_MS);
                PD_LatchFault(1, PD_FAULTS_OVERCURRENT);
                PD_Log(1, 2, 16, line);
                return 1;
            }
        }
        else
            p.ioc_at = 0;
    }
    ilim = c->iref_ma * PD_OC_LATCH_NUM / PD_OC_LATCH_DEN + PD_OC_LATCH_MARGIN_MA;
    if (c->iout_ma > ilim)
    {
        if (!p.oc_at)
            p.oc_at = now ? now : 1u;
        else if (now - p.oc_at >= PD_OC_LATCH_MS)
        {
            (void)snprintf(line, sizeof(line),
                           "SLOW OC LATCH: IOUT=%ld mA > %ld mA for %u ms (CC did not hold)",
                           (long)c->iout_ma, (long)ilim, (unsigned)PD_OC_LATCH_MS);
#if PD_OVERRIDE_NO_LATCH
            if (now - p.log_at >= 1000)
            {
                p.log_at = now;
                PD_Log(1, 2, 16, line);
            }
#else
            PD_LatchFault(1, PD_FAULTS_OVERCURRENT);
            PD_Log(1, 2, 16, line);
            return 1;
#endif
        }
    }
    else
        p.oc_at = 0;
#if PD_IL_TRIP_MA > 0
    /* Debounced: PD_PortIlOverRun() counts CONSECUTIVE injected IL samples above the threshold, so
       one coupling spike does not stop the stage while a held over-current does. The counter lives
       in pd_board.c because that is where every injected sample is seen; this side only decides. */
    if (PD_PortIlOverRun() >= PD_IL_TRIP_COUNT)
    {
        (void)snprintf(line, sizeof(line),
                       "IL TRIP: IL=%ld mA over %ld mA for %u samples; outputs disabled",
                       (long)c->il_ma, (long)PD_IL_TRIP_MA, (unsigned)PD_IL_TRIP_COUNT);
        PD_LatchFault(1, PD_FAULTS_OVERCURRENT);
        PD_Log(1, 2, 16, line);
        return 1;
    }
#endif
    return 0;
}
static void run_loop(uint32_t now)
{
    PD_Channel *c = &pd_channels[0];
    float vref, ie, e, u_raw, u, integ_prev;
    uint8_t cmp;
    /* Did the ACTUATOR deliver less than the loop asked for (short_up) or more (short_dn)?
       Both are filled by the mode-specific actuator stage below and are the anti-windup's
       only input besides the error direction. */
    uint8_t short_up, short_dn;
    char line[128];
    p.clamped = 0u;                                 /* set by the duty path if it limits */
    /* Slow current limit: fold the effective reference back. A pure integrator
       with a dead band cannot fight the voltage loop and cannot pump on noise.
       Gain sets a ~5 Hz CC loop at 20 ohm (faster on heavier loads, always
       well below the 150 Hz voltage loop). */
    ie = p.iout_f - limit_current_ma();
    if (ie > PD_CC_DEADBAND_MA)
        ie -= PD_CC_DEADBAND_MA;
    else if (ie < -PD_CC_DEADBAND_MA)
        ie += PD_CC_DEADBAND_MA;
    else
        ie = 0.0f;
    p.cc_drop_mv += PD_CC_GAIN_MV_PER_MA_S * ie * PD_LOOP_DT;
    if (p.cc_drop_mv < 0.0f)
        p.cc_drop_mv = 0.0f;
    if (p.cc_drop_mv > p.ramp_mv)
        p.cc_drop_mv = p.ramp_mv;
    vref = p.ramp_mv - p.cc_drop_mv;
    if (vref < 0.0f)
        vref = 0.0f;

    e = (vref - p.vout_f) * 0.001f;
    /* Record the executed reference and the filtered feedback BEFORE the control law runs,
       so the snapshot describes the tick that produced cmd_cnt. */
    p.terms.ref_mv = (int32_t)vref;
    p.terms.vout_f_mv = (int32_t)p.vout_f;

    /* STEP 2 (0.3.26): the integral is BIDIRECTIONAL and bounded symmetrically in the
       actuator's own domain, [-PD_CTRL_D_MAX, +PD_CTRL_D_MAX]. The bench showed every point
       below 10 V pinned at i = 0 with sat = 3: the integral could only fall to zero, so once
       the feed-forward over-asked, the loop had no authority left downward and the residual
       error became structural. A negative integral only removes duty the feed-forward
       over-asked for - the final output is still limited to [0, PD_CTRL_D_MAX] before it
       reaches the registers, so this cannot command negative duty or reverse current.
       PD_CTRL_D_MAX is the controller's full scale, which in linked mode is the q ceiling
       (0.875) rather than the A-bridge ceiling (0.70): see the top of this file. */
    integ_prev = p.integ;
    p.integ += PD_CV_KI * e * PD_LOOP_DT;
    if (p.integ < -PD_CTRL_D_MAX)
        p.integ = -PD_CTRL_D_MAX;
    if (p.integ > PD_CTRL_D_MAX)
        p.integ = PD_CTRL_D_MAX;

    /* u_raw keeps its original meaning (unclamped PI + feed-forward over the full period) so
       the anti-windup sees the same error direction it always did. */
    u_raw = PD_CV_KP * e + p.integ + duty_ff(vref);
    u = u_raw;
    if (u < 0.0f)
        u = 0.0f;
    if (u > PD_CTRL_D_MAX)
        u = PD_CTRL_D_MAX;
    /* Remember whether this stage limited the request, not only the later ones. */
    if (u < u_raw)
        p.clamped = 1u;

    /* The filter runs on the CLAMPED output only, so its state can never be pushed outside the
       legal range and the two limiting stages cannot contaminate each other. */
    p.filt += PD_CV_POLE_ALPHA * (u - p.filt);
    if (p.filt < 0.0f)
        p.filt = 0.0f;
    if (p.filt > PD_CTRL_D_MAX)
        p.filt = PD_CTRL_D_MAX;
    u = p.filt;

    /* Snapshot the control terms in compare counts, before the actuator stage can change
       anything. ff/p/i are the three terms of the law. q_cnt is the coordinate the controller
       asks the modulator for; it replaces the old "fu" field, because after rounding it carries
       the same information (the pre-modulator output) plus the exact integer the distributor
       receives. i_cnt is what the review asked to see: if it sits on a clamp, the loop has lost
       authority in that direction. */
    p.terms.ff_cnt = (int32_t)(duty_ff(vref) * (float)PD_SW_PERIOD + (duty_ff(vref) >= 0.0f ? 0.5f : -0.5f));
    p.terms.p_cnt = (int32_t)(PD_CV_KP * e * (float)PD_SW_PERIOD + (PD_CV_KP * e >= 0.0f ? 0.5f : -0.5f));
    p.terms.i_cnt = (int32_t)(p.integ * (float)PD_SW_PERIOD + (p.integ >= 0.0f ? 0.5f : -0.5f));
    /* The loop asks in q. Rounding to counts HERE, once, is what makes the actuator's report
       (q_applied) directly comparable with the request instead of being off by a count. p.filt
       is clamped to [0, PD_CTRL_D_MAX] and PD_CTRL_D_MAX * PD_SW_PERIOD stays far below
       INT32_MAX, so this cannot overflow and cannot be negative - PD_ModDistribute() can never
       return PD_MOD_LIMIT_INVALID for a value that came from here. */
    p.terms.q_cnt = (int32_t)(p.filt * (float)PD_SW_PERIOD + 0.5f);
    p.terms.mod_limit = 0u;
    p.terms.mod_zone = PD_MOD_ZONE_NONE;
    short_up = 0u;
    short_dn = 0u;

    /* Actuator stage. Both branches end in the same register-write path, and both report the
       SAME two facts the anti-windup needs: did the actuator deliver less than the loop asked
       for, or more.
     *
     * The difference is where "the limit" is measured. Legacy: the A-bridge ceiling, judged on
       the requested duty. Linked: the distributor's own report, judged on the compare pair it
       produced - so a limit coming from the B floor, the A ceiling or the q range reaches the
       integrator without pd_power.c having to know which one it was. */
#if PD_MOD_LINKED
    {
        PD_ModResult mod;
        PD_ModDistribute(p.terms.q_cnt, &mod);
        p.terms.mod_limit = mod.limit;
        p.terms.mod_zone = mod.zone;
        /* The anti-windup's question: can the committed pair go further in the direction the
           error asks for. Judged on what the DISTRIBUTOR produced, before the register-level
           clamp in commit_counts() can move it - the distributor's answer is the one that
           describes the modulator, and commit_counts() can only ever clamp A down. */
        if (mod.q_applied < p.terms.q_cnt)
            short_up = 1u; /* the pair cannot go higher: q_max, the A ceiling or the B floor */
        else if (mod.q_applied > p.terms.q_cnt)
            short_dn = 1u; /* one of the floors raised the request */
        cmp = commit_counts(mod.cmp_a, mod.cmp_b, PD_CMP_STAGE_LOOP);
    }
#else
    if (p.clamped)
        short_up = 1u;
    cmp = commit_duties(u, PD_OUT_BRIDGE_DUTY, PD_CMP_STAGE_LOOP);
#endif
    if (cmp == PD_CMP_ERR_STOPPED)
        return; /* a fault/stop won the race: not ours to report or undo */
    /* Record what actually reached the registers. q_app_cnt inverts the COMMITTED pair, in
       counts: 0.75*A/B, which in legacy mode reduces to A because B is 0.75. Together with
       mod_limit it answers "the loop asked for this, the actuator got that". */
    if (cmp == PD_CMP_OK)
    {
        p.terms.cmd_cnt = (int32_t)p.cmp_in;
        p.terms.cmp_out_cnt = (int32_t)p.cmp_out;
        p.terms.q_app_cnt = PD_ModAppliedQ(p.cmp_in, p.cmp_out);
        /* sat answers "why is the output limited", ceiling first:
             1 - the ceiling is engaged (the filter is at full scale, or the distributor hit
                 the q ceiling, the A ceiling or the B floor);
             2 - the floor is engaged (the request is at zero, or the distributor hit one of
                 the minimums);
             3 - the integrator ITSELF is on its own clamp, so the loop has no authority left
                 in that direction and the remaining error is structural (feed-forward or plant
                 model).
         *
         * 0.3.32: the actuator half of this no longer comes from short_up/short_dn. Those are
         * the ANTI-WINDUP's question ("can the pair go further this way?") and they tolerate a
         * one-count difference on purpose; the 20 V bench run showed PD_ModAppliedQ() returning
         * 17796 for a request of 17795, which lit s=2 permanently while lim stayed 0x00 - a
         * flag that is always on hides the event it exists for. The reason now comes from the
         * distributor's own limit bits, so s and lim cannot disagree: a limit bit that names a
         * ceiling sets 1, one that names a floor sets 2, and nothing else sets either. */
#if PD_MOD_LINKED
        if (p.terms.mod_limit & (PD_MOD_LIMIT_Q_MAX | PD_MOD_LIMIT_A_MAX | PD_MOD_LIMIT_B_MIN) ||
            u >= PD_CTRL_D_MAX)
            p.terms.sat = 1u;
        else if (p.terms.mod_limit & (PD_MOD_LIMIT_Q_MIN | PD_MOD_LIMIT_A_MIN) || u <= 0.0f)
            p.terms.sat = 2u;
#else
        if (u >= PD_CTRL_D_MAX)
            p.terms.sat = 1u;
        else if (u <= 0.0f)
            p.terms.sat = 2u;
#endif
        else if (p.integ <= -PD_CTRL_D_MAX || p.integ >= PD_CTRL_D_MAX)
            p.terms.sat = 3u;
        else
            p.terms.sat = 0u;
    }
    /* STEP 3 (0.3.26): anti-windup keyed on the final limit and the error direction, restoring
       the SAVED integral instead of subtracting one increment. The old form could not restore
       the previous value once the integrator sat on a limit, which is how it stayed pinned.
       e > 0 means the output is below the reference, i.e. the loop wants MORE; short_up/dn is
       the actuator's answer to that request, in either mode. */
    if ((u_raw > PD_CTRL_D_MAX || short_up) && (e > 0.0f))
        p.integ = integ_prev;                    /* wanted more, could not have it */
    else if ((u_raw < 0.0f || short_dn) && (e < 0.0f))
        p.integ = integ_prev;                    /* wanted less, could not have it */
    /* else: the error points back inside, or the output is in range - integrate normally. */
    if (cmp != PD_CMP_OK)
    {
        /* Nothing was written and the bounded retries are exhausted: the loop
           is no longer in control. Stop and say exactly why. */
        commit_failed(cmp, "COMPARE COMMIT FAILED");
        return;
    }

    if (p.state == PD_STATES_RUN_CV || p.state == PD_STATES_RUN_CC)
    {
        p.state = p.cc_drop_mv > PD_CC_ACTIVE_MV ? PD_STATES_RUN_CC : PD_STATES_RUN_CV;
        c->state = p.state;
        /* Regulation-failure backstop: duty pinned high but the output never
           rises (dead gate drive, collapsed bootstrap, open inductor path).
           Compared against the ACCEPTED TARGET, not the ramp: during a descent
           the reference is intentionally above the output, and that case is
           handled by the descent supervisor, not by this backstop.

           0.3.28: "pinned high" means the ACTUATOR IS OUT OF RANGE, which is no
           longer the same as "the A compare value is at its ceiling". In linked
           mode A reaches PD_CMP_A_MAX at q = 0.75 while B still has from 75 % down
           to 60 % of travel left, so testing the A ceiling alone would latch
           PD_FAULTS_INTERNAL exactly in the newly opened region - while the stage
           could still have recovered. The full-range test is the q the committed
           PAIR represents (PD_CTRL_LIMIT_CNT, 0.875 of the period). In legacy mode
           q_app_cnt reduces to the A compare value, so one definition covers both
           modes and neither can drift from the other.

           0.3.29: the test itself, PD_PowerAtRangeEnd(), is now shared with the code-17
           status field, so the backstop and the operator-facing flag cannot disagree
           about what "out of range" means. */
        if (p.vout_f < (float)p.target_mv - PD_REG_FAIL_MARGIN_MV && PD_PowerAtRangeEnd())
        {
            if (!p.reg_at)
                p.reg_at = now ? now : 1u;
            else if (now - p.reg_at >= PD_REG_FAIL_MS)
            {
                /* Enough to explain the stop without another log, and inside the
                   field by construction (tools/check_log_lengths.py):
                 * target, reference, the pinned pair, the measured output, the
                 * input voltage and the range the input allows. S=1 says the
                 * committed pair is at the END OF ITS RANGE (q at the ceiling, or
                 * the A bridge alone at 70 % in the legacy build). */
                (void)snprintf(line, sizeof(line),
                               "REGULATION FAILED: tg=%ld rf=%ld V=%ld I=%ld mx=%ld duty=%u/%u S=%u",
                               (long)p.target_mv, (long)ramp_int(), (long)c->vout_mv, (long)c->vin_mv,
                               (long)PD_PowerAllowedVmaxMv(c->vin_mv),
                               /* Raw register counts, not bounded counters: the
                                  999-cap helper would print 1000 for every realistic
                                  duty. Both bridges, so the log shows WHICH pair was
                                  pinned rather than one number that no longer
                                  identifies the operating point. */
                               p.cmp_in, p.cmp_out,
                               (unsigned)(p.terms.q_app_cnt >= PD_CTRL_LIMIT_CNT - 8));
#if PD_OVERRIDE_NO_LATCH
                if (now - p.log_at >= 1000)
                {
                    p.log_at = now;
                    PD_Log(1, 2, 16, line);
                }
#else
                PD_LatchFault(1, PD_FAULTS_INTERNAL);
                PD_Log(1, 2, 16, line);
#endif
            }
        }
        else
            p.reg_at = 0;
    }
}
void PD_PowerTick(void)
{
    PD_Channel *c = &pd_channels[0];
    uint32_t now = PD_PortNow(), key;
    float vin = (float)c->vin_mv, vout = (float)c->vout_mv, iout = (float)c->iout_ma;
    float seed;
    uint8_t cmp;
    /* Buffer for the sampling-fault report below. protect() has its own; this function did not
       have one, which the -Werror variants caught. 96 bytes: check_log_lengths.py computes the
       worst case of this format (85 incl. NUL) and a hand-guessed 80 did not fit. */
    char samp[96];

    p.vout_f += PD_VOUT_LPF_ALPHA * (vout - p.vout_f);
    p.iout_f += PD_IOUT_LPF_ALPHA * (iout - p.iout_f);
    p.vin_f += 0.2f * (vin - p.vin_f);

    if (p.state == PD_STATES_OFF || p.state == PD_STATES_FAULT_LATCHED)
        return;
    /* Fixed cycle order (0.3.20). Every stage below uses the values produced by
       the stage above it, in this tick - never a mixture of generations:
         1. sampling validity   - a protection decision on stale feedback is void
         2. target snapshot     - the range-checked user target
         3. ramp update         - the reference this tick will execute
         4. protection threshold- derived from (2) and (3)
         5. protection check    - OVP/input/OC/slow-OC/IL with that threshold
         6. descent supervision - may stop on timeout
         7. control output      - PI + feed-forward, committed to HRTIM */
    if (!PD_PortSamplingHealthy() || PD_PortFastAge() > PD_LOOP_STALE_MS ||
        (c->valid & (PD_VALID_BITS_VIN | PD_VALID_BITS_VOUT | PD_VALID_BITS_IOUT | PD_VALID_BITS_IIN)) !=
        (PD_VALID_BITS_VIN | PD_VALID_BITS_VOUT | PD_VALID_BITS_IOUT | PD_VALID_BITS_IIN))
    {
        /* Say WHICH sub-condition failed, and the four ages, on the tick that decides. The 0.3.36
           field fault could not be attributed because this branch reported only that it had fired:
           the 1 Hz diagnostic samples its counters once a second and the deciding tick fell between
           two of them. A verdict that cannot name its reason costs a bench session.
           Ages are clamped: they are bounded only by uptime (uint32 ms = 49 days), and five
           unclamped ages would blow past the 96-byte log field. 999 is the same ceiling
           PD_LOG_VALUE uses and far above any healthy value here (3/5/10 ms). */
        uint32_t fast_age = 0u, slow_age0 = 0u, slow_age1 = 0u, input_age = 0u;
        uint32_t why = PD_PortSampleFailMask(&fast_age, &slow_age0, &slow_age1, &input_age);
        uint32_t vbits = (uint32_t)(c->valid &
                                    (PD_VALID_BITS_VIN | PD_VALID_BITS_VOUT | PD_VALID_BITS_IOUT |
                                     PD_VALID_BITS_IIN));
        uint32_t fast_total = PD_PortFastAge();
        PD_LatchFault(1, PD_FAULTS_ADC);
        (void)snprintf(samp, sizeof(samp), "SAMP BAD why=0x%02lX vld=0x%02lX a=%lu/%lu/%lu/%lu",
                       (unsigned long)why, (unsigned long)vbits,
                       (unsigned long)(fast_age > 999u ? 999u : fast_age),
                       (unsigned long)(slow_age0 > 999u ? 999u : slow_age0),
                       (unsigned long)(slow_age1 > 999u ? 999u : slow_age1),
                       (unsigned long)(input_age > 999u ? 999u : input_age));
        PD_Log(1, 2, 16, samp);
        (void)fast_total;
        return;
    }
    update_target(now);
    update_ramp();
    p.ovp_limit_mv = ovp_for(p.target_mv, ramp_int());
    if (protect(now))
        return;
    /* Input-sag supervision (0.3.22). The A-bridge compare value is already
       clamped by PD_CMP_A_MAX, so a sag cannot over-drive the stage; but if the
       ACCEPTED target stays outside the range the measured VIN allows, the loop
       is saturating by definition and that must be reported instead of silently
       producing less than the setpoint. Independent 10 ms timer; latches the
       INPUT fault, so recovery never resumes automatically. */
    {
        int32_t vmax = PD_PowerAllowedVmaxMv(c->vin_mv);
        if (vmax <= 0 || p.target_mv > vmax)
        {
            if (!p.range_at)
                p.range_at = now ? now : 1u;
            else if (now - p.range_at >= PD_VIN_RANGE_HOLD_MS)
            {
                char line[128];
                (void)snprintf(line, sizeof(line),
                               "INPUT SAG: tgt=%ld mV > max %ld mV at VIN=%ld for %u ms; outputs off"
                               "",
                               (long)p.target_mv, (long)vmax, (long)c->vin_mv,
                               (unsigned)PD_VIN_RANGE_HOLD_MS);
                PD_LatchFault(1, PD_FAULTS_INPUT);
                PD_Log(1, 2, 16, line);
                return;
            }
        }
        else
            p.range_at = 0;
    }
    if (down_track(now))
        return;
    switch (p.state)
    {
    case PD_STATES_BOOTSTRAP_PRECHARGE:
        if (now - p.state_at >= PD_PRECHARGE_MS)
        {
            /* Seed the reference from the measured output, CLAMPED to the
               target: a pre-biased output must not make the loop command a
               higher voltage, and the guard may never start above the setpoint. */
            seed = p.vout_f < 0.0f ? 0.0f : p.vout_f;
            if (seed > (float)p.target_mv)
                seed = (float)p.target_mv;
            p.ramp_mv = seed;
            p.ovp_limit_mv = ovp_for(p.target_mv, ramp_int());
            /* Leave the static precharge state: the compare values are already
               legal and preloaded, so switching the sources to compare-driven
               only produces a normal dead-time-separated edge at the next
               master-period event. */
            PD_CmpSetStage(PD_CMP_STAGE_PWM);
            /* Hand-off pair. Linked mode asks the distributor for the smallest executable q, so
               the precharge->PWM edges are produced by the same mapping the loop uses one tick
               later; legacy keeps the original A = PD_DUTY_MIN / B = 75 %. In linked mode both
               give the SAME pair (q_min = 0.05 -> A = 1088, B = 16320), which is the point: this
               change must not alter the start sequence. */
#if PD_MOD_LINKED
            {
                PD_ModResult mod;
                PD_ModDistribute((int32_t)PD_MOD_Q_MIN_COUNTS, &mod);
                cmp = commit_counts(mod.cmp_a, mod.cmp_b, PD_CMP_STAGE_PWM);
            }
#else
            cmp = PD_PortSetCompare(1, ratio_cmp(PD_DUTY_MIN), PD_OUT_BRIDGE_CMP);
#endif
            if (cmp != PD_CMP_OK || !PD_PortWaitPeriods(2))
            {
                commit_failed(cmp, "PWM ENABLE FAILED");
                if (cmp == PD_CMP_OK)
                    PD_Log(1, 2, 14, "PWM ENABLE FAILED: period wait timeout; outputs disabled");
                break;
            }
            key = PD_PortLock();
            if (p.state != PD_STATES_BOOTSTRAP_PRECHARGE || c->fault || !c->enabled)
            {
                PD_PortUnlock(key);
                return;
            }
            PD_PortBridgeMode(2, out_inv(), 0);
            PD_PortBridgeMode(1, in_inv(), 0);
            p.pwm_enabled = 1;
            /* The pair in force after the hand-off. The legacy branch needs these because it
               wrote the registers directly; linked mode published the same pair inside
               commit_counts() (the distributor at q_min gives A = 1088, B = 16320, exactly these
               two values), so re-stating them is a no-op there and keeps both modes ending in
               the same published state. */
            p.cmp_in = ratio_cmp(PD_DUTY_MIN);
            p.cmp_out = PD_OUT_BRIDGE_CMP;
            set_state(PD_STATES_SOFT_START);
            c->state = PD_STATES_SOFT_START;
            PD_PortUnlock(key);
            PD_Log(1, 0, 14, "PRECHARGE done; PWM enabled, soft start");
        }
        break;
    case PD_STATES_SOFT_START:
        /* The up-ramp already ran in update_ramp(). */
        if (p.ramp_mv >= (float)p.target_mv)
        {
            p.ramp_mv = (float)p.target_mv;
            set_state(PD_STATES_RUN_CV);
            PD_Log(1, 0, 14, "SOFT START complete; RUN_CV");
        }
        run_loop(now);
        break;
    case PD_STATES_RUN_CV:
    case PD_STATES_RUN_CC:
        /* Both ramps already ran in update_ramp(). */
        run_loop(now);
        break;
    case PD_STATES_STOPPING:
        /* The stop ramp already ran in update_ramp(), before the threshold was
           computed, so the OVP tracked it during this tick too. */
        if (p.ramp_mv > 0.0f)
            run_loop(now);
        if (p.ramp_mv <= 0.0f)
            PD_PowerStop(1);
        break;
    default:
        break;
    }
    key = PD_PortLock();
    if (p.state != PD_STATES_OFF && !c->fault && c->enabled)
    {
        c->state = p.state;
        c->duty_in = p.cmp_in;
        c->duty_out = p.cmp_out;
    }
    PD_PortUnlock(key);
}
uint16_t PD_GateTest(uint8_t action, const uint8_t *payload, uint16_t n, uint8_t *reply,
                     uint16_t *out)
{
    uint32_t now = PD_PortNow();
    *out = 0;
    if (action == 0)
    {
        if (n != PD_GATE_TEST_REQUEST_SIZE)
            return PD_ERRORS_BAD_LENGTH;
        *out = gate_status(reply, now);
        return 0;
    }
    if (action == 1)
    {
        uint8_t index;
        char line[96];
        uint16_t ms;
        uint32_t key;
        if (n != PD_GATE_TEST_REQUEST_SIZE)
            return PD_ERRORS_BAD_LENGTH;
        index = payload[1];
        ms = (uint16_t)(payload[2] | ((uint16_t)payload[3] << 8));
        if (index < 1 || index > 4)
            return PD_ERRORS_RANGE;
        if (ms < 100 || ms > 2000)
            return PD_ERRORS_RANGE;
        if (PD_ZeroBusy())
            return PD_ERRORS_BUSY;
        if (pd_channels[0].fault)
            return PD_ERRORS_FAULT;
        if (pd_channels[0].state != PD_STATES_OFF || pd_channels[0].enabled || gate_index)
            return PD_ERRORS_NOT_OFF;
        if (!PD_PortSamplingHealthy())
            return PD_ERRORS_NOT_READY;
        /* Interlock: the gate test is only allowed with the power input off. */
        if (pd_channels[0].vin_mv > PD_GATE_TEST_VIN_MAX_MV ||
            pd_channels[0].vout_mv > PD_GATE_TEST_VOUT_MAX_MV)
        {
            PD_Log(1, 2, 15, "GATE TEST refused: discharge VIN (<5 V) and VOUT (<1 V) first");
            return PD_ERRORS_NOT_READY;
        }
        if (!PD_PortGateTestStart(index))
        {
            PD_Log(1, 2, 15, "GATE TEST ABORTED: timer timeout or CH1 fault; outputs off");
            return PD_ERRORS_NOT_READY;
        }
        key = PD_PortLock();
        if (pd_channels[0].fault || PD_PortFaultPins(1))
        {
            PD_PortGateTestStop();
            PD_PortUnlock(key);
            return PD_ERRORS_FAULT;
        }
        gate_index = index;
        gate_until = PD_PortNow() + ms;
        PD_PortUnlock(key);
        (void)snprintf(line, sizeof(line),
                       "GATE TEST: PA%d gets the 20%% pulse; its bridge mate runs 80%% to charge the bootstrap",
                       index == 1 ? 8 : index == 2 ? 9 : index == 3 ? 10 : 11);
        PD_Log(1, 1, 15, line);
        *out = gate_status(reply, now);
        return 0;
    }
    if (action == 2)
    {
        if (n != PD_GATE_TEST_REQUEST_SIZE)
            return PD_ERRORS_BAD_LENGTH;
        if (payload[1] > 3)
            return PD_ERRORS_RANGE;
        if (gate_index)
            return PD_ERRORS_BUSY;
        /* Compatibility with the old PC restore command: designed mapping is
           a no-op. Runtime inversion is no longer an operator setting. */
        if (payload[1] != PD_GATE_MAP_DESIGNED) return PD_ERRORS_UNSUPPORTED;
        PD_Log(1, 0, 15, "GATE MAP fixed to verified wiring; manual confirmation not required");
        *out = gate_status(reply, now);
        return 0;
    }
    if (action == 3)
    {
        if (n != PD_GATE_TEST_REQUEST_SIZE)
            return PD_ERRORS_BAD_LENGTH;
        if (gate_index)
            return PD_ERRORS_BUSY;
        PD_Log(1, 0, 15, "GATE MAP fixed; legacy clear does not change wiring or startup readiness");
        *out = gate_status(reply, now);
        return 0;
    }
    return PD_ERRORS_RANGE;
}
void PD_GateTestTick(void)
{
    if (!gate_index)
        return;
    if ((int32_t)(PD_PortNow() - gate_until) < 0)
        return;
    PD_PortGateTestStop();
    PD_Log(1, 0, 15, "GATE TEST finished; outputs disabled, EN low");
    gate_index = 0;
    gate_until = 0;
}
