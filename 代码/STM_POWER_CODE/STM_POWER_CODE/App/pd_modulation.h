/* CH1 dual-bridge duty distributor (0.3.28, plan phase A).
 *
 * Pure, integer-in / integer-out, no hardware and no globals, so the whole of it is testable on
 * the host. It is the ONLY place that decides the A/B compare pair; the control loop produces one
 * control quantity q and this function splits it.
 *
 * q is the EQUIVALENT A-bridge duty: in the low region q = DA with DB fixed, and in the high
 * region q exceeds the A-bridge ceiling while remaining a controller coordinate (it is never
 * written to HRTIM and never means a MOSFET above 100 %). When nothing is limited the distributor
 * preserves  DA/DB = q/0.75, i.e. the continuation of the existing behaviour.
 *
 * Zones (all values are high-side COMMAND duties, not measured conduction ratios):
 *   low         q <= PD_MOD_Q_LOW             DA = q,                DB = 0.75
 *   transition  PD_MOD_Q_LOW < q < PD_MOD_Q_HIGH
 *                                             DA = 0.65 + 0.10*t - 0.05*t^2, t = (q-0.65)/0.10
 *                                             DB = 0.75*DA/q
 *   high        q >= PD_MOD_Q_HIGH            DA = 0.70,            DB = 0.525/q
 * The transition formula is continuous in value and first derivative at both ends, and it is
 * exactly the ratio relation DA/DB = q/0.75 throughout.
 *
 * What this file does NOT do, on purpose: it does not claim the delivered VOUT/VIN follows
 * q/0.75 (the plan calls k_v an evidence-limited correction, not a constant), it does not decide
 * whether the power stage may run at a given q, and it does not touch the protections. Callers
 * must treat an executed q below the requested q as a real limit and feed it to the anti-windup.
 */
#ifndef PD_MODULATION_H
#define PD_MODULATION_H
#include <stdint.h>

/* Why the executed q differs from the requested one - also the anti-windup input. */
#define PD_MOD_LIMIT_NONE 0x00u
#define PD_MOD_LIMIT_Q_MAX 0x01u     /* q above the released experimental ceiling */
#define PD_MOD_LIMIT_Q_MIN 0x02u     /* q below the executable floor (not a zero request) */
#define PD_MOD_LIMIT_A_MAX 0x04u     /* A at its ceiling, so q cannot rise further */
#define PD_MOD_LIMIT_B_MIN 0x08u     /* B at its experimental floor, so q cannot rise further */
#define PD_MOD_LIMIT_A_MIN 0x10u     /* a non-zero request floored to the minimum pulse. Cannot fire
                                      * while PD_MOD_A_MIN_COUNTS <= PD_MOD_Q_MIN_COUNTS, which
                                      * pd_config.h asserts: the q floor gets there first. Kept as
                                      * the register-level net against a zero compare value. */
#define PD_MOD_LIMIT_ZERO 0x20u      /* zero request: the power state machine owns the output */
#define PD_MOD_LIMIT_INVALID 0x40u   /* non-finite or malformed input: nothing is written */

/* Distribution zones, for diagnostics only - the control law must not branch on them. */
#define PD_MOD_ZONE_LOW 0u
#define PD_MOD_ZONE_TRANSITION 1u
#define PD_MOD_ZONE_HIGH 2u
#define PD_MOD_ZONE_NONE 3u

typedef struct
{
    int32_t q_request;    /* requested q, in counts of the period */
    int32_t q_applied;    /* q implied by the compare pair actually produced */
    uint16_t cmp_a;       /* A-bridge compare value, counts */
    uint16_t cmp_b;       /* B-bridge compare value, counts */
    uint8_t zone;         /* PD_MOD_ZONE_* */
    uint8_t limit;        /* PD_MOD_LIMIT_* bit mask, 0 when nothing was limited */
} PD_ModResult;

/* Split q into an A/B compare pair. `q_counts` is q in period counts, i.e. q*PD_SW_PERIOD.
 *
 * Bounds come from pd_config.h so this file has a single source of truth:
 *   [PD_MOD_Q_MIN_COUNTS, PD_MOD_Q_MAX_COUNTS]  released experimental q range
 *   PD_CMP_A_MAX                                A bridge ceiling (70 %)
 *   PD_MOD_B_MIN_COUNTS                         experimental B floor (60 % for phase B)
 *   PD_OUT_BRIDGE_CMP                           B bridge ceiling, still 75 %
 *   PD_CMP_MIN                                  HRTIM high-resolution floor
 * A zero request is reported (PD_MOD_LIMIT_ZERO) and produces no duty: the power state machine,
 * not the modulator, decides what "off" means. Invalid input produces PD_MOD_LIMIT_INVALID with
 * both compare values at PD_CMP_MIN, and the caller must not commit them as a running setpoint.
 */
void PD_ModDistribute(int32_t q_counts, PD_ModResult *out);

/* q implied by an A/B pair that was actually committed: 0.75 * A / B, in counts.
 * Used by the anti-windup so "the actuator cannot go further" is judged on what the registers
 * received rather than on what the control law wanted. */
int32_t PD_ModAppliedQ(uint16_t cmp_a, uint16_t cmp_b);

#endif
