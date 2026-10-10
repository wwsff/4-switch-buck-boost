#ifndef PD_POWER_H
#define PD_POWER_H
#include <stdint.h>

/* Restricted CH1 buck power stage.
 *
 * Only CH1 has a populated power board (Q1..Q4, U3/U4 UCC27301ADR, L=10 uH,
 * Cout ~= 700 uF). CH2 has no power board and no gate path.
 *
 * Control structure (all of it runs in the 1 kHz task, never in an ISR):
 *   target -> soft-start ramp -> voltage PI (+duty feed-forward)
 *                                |              ^
 *                                |        filtered VOUT (20-sample boxcar)
 *   zeroed IOUT -> slow CC fold-back (lowers the effective reference)
 *   hardware OC/ALERT -> HRTIM fault (independent of this module)
 *
 * The task also owns the HRTIM compare writes, so both bridges always commit
 * their new duty at the same master-period update event.
 */
void PD_PowerInit(void);
/* 1 kHz control tick. Reads pd_channels[0]; must run in task context. */
void PD_PowerTick(void);
/* OUTPUT=1 request. Returns 0 when the start sequence was accepted. */
uint16_t PD_PowerStart(uint8_t ch);
/* Full stop: safe port state first, then control state reset. */
void PD_PowerStop(uint8_t ch);
/* Operator stop: ramp the reference down, then disable gates and EN. */
void PD_PowerRequestStop(void);
/* ISR-safe state reset without touching the port (fault latch path). */
void PD_PowerAbort(uint8_t ch);
/* 1 when a start request would pass every precondition right now. */
uint8_t PD_PowerReady(uint8_t ch);
/* 1 while the state machine is not OFF/FAULT_LATCHED. */
uint8_t PD_PowerActive(uint8_t ch);
/* Duty actually committed to HRTIM (compare counts), 0 when inactive. */
uint16_t PD_PowerDutyIn(void);
uint16_t PD_PowerDutyOut(void);
/* Reference bookkeeping (0.3.20), all in millivolts, 0 while inactive:
 *   PD_PowerTargetMv     last ACCEPTED user target
 *   PD_PowerRampMv       reference the loop executes right now
 *   PD_PowerOvpLimitMv   OVP threshold in force (tracks the reference down, and
 *                        is capped by the absolute ceiling)
 *   PD_PowerDownActive   1 while a commanded descent is being supervised */
/* Voltage-loop internal terms for diagnostics (0.3.25). Read-only snapshot of what the
 * controller computed on the last 1 kHz tick, in COMPARE COUNTS so it can be compared with
 * the duty actually committed. i_cnt is signed on purpose: whether the integral is pinned
 * at its clamp is the question this exists to answer.
 *
 * 0.3.28 adds the two ends of the modulation chain, because the actuator is no longer a
 * single duty:
 *   q_cnt         the coordinate the controller asks for, in counts (replaces the old
 *                 filt_cnt: after rounding, it IS the pre-modulator output)
 *   q_app_cnt     the coordinate the COMMITTED pair actually represents, 0.75*A/B in counts.
 *                 In legacy mode this reduces to A, because B is 0.75.
 *   cmp_out_cnt   the B-bridge compare value that was written
 *   mod_limit     PD_MOD_LIMIT_* bits from the distributor; structurally 0 in legacy mode,
 *                 where no distributor runs. A limit is the only honest explanation of
 *                 q_app_cnt != q_cnt, which is why the two are reported together. */
typedef struct
{
    int32_t ref_mv;      /* executed reference (ramp - CC fold-back) */
    int32_t vout_f_mv;   /* filtered feedback the loop used */
    int32_t ff_cnt, p_cnt, i_cnt, cmd_cnt;
    int32_t q_cnt, q_app_cnt, cmp_out_cnt;
    uint8_t mod_limit;   /* PD_MOD_LIMIT_* */
    uint8_t mod_zone;    /* PD_MOD_ZONE_* */
    uint8_t sat;         /* 0 none, 1 ceiling engaged, 2 floor engaged, 3 integrator clamped */
} PD_CtrlTerms;
void PD_PowerCtrlTerms(PD_CtrlTerms *out);
/* 1 when the committed pair is at the END OF ITS USABLE RANGE, in the coordinate the active
 * modulator drives: q at the released ceiling (linked) or the A compare at PD_CMP_A_MAX
 * (legacy). ONE definition, used by the regulation-failure backstop and by the code-17
 * status field.
 *
 * Why it is not "the A bridge is at 70 %" any more (0.3.29): with both bridges scheduled, A
 * reaches its ceiling at q = 0.75 while B still has 15 percentage points of travel in hand -
 * the bench run at 18 V out of 19.82 V in sat exactly there (A = 15232, B = 15247, q = 16311
 * of 19040, nothing limited). A flag that called that "saturated" told the operator the
 * opposite of the truth. */
uint8_t PD_PowerAtRangeEnd(void);
/* Which committed high-side falling edge lies inside the ADC sampling window, if any
 * (0x1 A, 0x2 B, 0x3 both, 0 neither); *dist gets that edge's signed distance to the sampling
 * instant in counts. Report only - the control law never reads it. */
uint8_t PD_PowerAdcEdgeNear(uint16_t cmp_a, uint16_t cmp_b, int32_t *dist);
int32_t PD_PowerTargetMv(void);
int32_t PD_PowerRampMv(void);
int32_t PD_PowerOvpLimitMv(void);
uint8_t PD_PowerDownActive(void);
/* Available output range for a given input voltage (0.3.23, measured model).
 * ONE source of truth for the start precondition, the runtime SET_TARGET check
 * and the input-sag supervision, so a target cannot be accepted by one path and
 * refused by another. Pure integer function: same inputs, same result; no state,
 * no I/O. Returns 0 when no usable range exists (VIN outside the input window, or
 * no output reachable at all) - callers must treat 0 as "refuse", never as "clamp
 * to the minimum". */
int32_t PD_PowerAllowedVmaxMv(int32_t vin_mv);
/* Smallest input voltage at which that target becomes settable, for messages
 * only (inverse of the measured line plus the margin). 0 = needs more than 30 V. */
int32_t PD_PowerRequiredVinMv(int32_t target_mv);
/* 1 while the accepted target is inside the range for the latest VIN. */
uint8_t PD_PowerTargetInRange(void);

/* Gate-path verification. Raw HRTIM output indices 1..4 = PA8, PA9, PA10, PA11.
 * The test drives the whole half-bridge that owns the selected output: the
 * selected output gets a 20 % narrow pulse and its complementary mate 80 %, so
 * the switch node is pulled low every cycle and the UCC27301A internal
 * bootstrap diode can charge C10/C15.  Without that window the floating
 * high-side output has no supply and its gate cannot move - which is exactly
 * what the 0.3.0/0.3.1 single-output test could not see.
 *
 * UCC27301ADR (D/SOIC-8) is 1 VDD, 2 HB, 3 HO, 4 HS, 5 HI, 6 LI, 7 VSS, 8 LO,
 * and the power-board netlist wires HO to the high-side gate and LO to the
 * low-side gate for both bridges. The bench test therefore confirms the
 * assembled board matches the schematic. The operator has now confirmed the
 * wiring; mapping is fixed at PD_GATE_MAP_DESIGNED, without a per-boot manual
 * confirmation. Legacy query/confirm/clear packets remain compatible no-ops. */
#define PD_GATE_MAP_DESIGNED 0u /* TA1->input HS, TA2->input LS, TB1->output HS, TB2->output LS */
#define PD_GATE_MAP_IN_SWAP 1u  /* measured: TA1 reaches the input low side */
#define PD_GATE_MAP_OUT_SWAP 2u /* measured: TB1 reaches the output low side */
#define PD_GATE_MAP_BOTH_SWAP 3u
uint16_t PD_GateTest(uint8_t action, const uint8_t *payload, uint16_t n, uint8_t *reply,
                     uint16_t *out);
void PD_GateTestTick(void);
uint8_t PD_GateMappingConfirmed(void);
uint8_t PD_GateMappingCode(void);
#endif
