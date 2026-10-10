#include "pd_modulation.h"
#include "pd_config.h"

/* Integer-only, everything in period counts so the boundary arithmetic cannot drift with the float
 * conversion at the call site and so the host tests exercise exactly what the firmware runs.
 *
 * The ratio relation DA/DB = q/0.75 is preserved structurally: every zone computes A first and then
 * derives B from
 *     B = A * (ceiling / q)   with ceiling = 1.5 * PD_SW_PERIOD,
 * which is the plan's high-zone formula DB = 0.525/q written in counts. Substituting A = 0.75*q
 * gives B = ceiling exactly - the fixed 75 % the low region needs.
 *
 * A is rounded to nearest and B is then derived from the ROUNDED A, so the pair cannot drift away
 * from the ratio and PD_ModAppliedQ(cmp_a, cmp_b) reproduces q within one count. Deriving B from A
 * is deliberate: rounding A and B independently would let them disagree with the ratio.
 */
/* K in  b_counts = a_counts * K / q_counts, derived from DA/DB = q/0.75:
 *     b_counts = 0.75 * P * a_counts / q_counts * P / P = (0.75 * P^2) * a_counts / (q_counts * P)
 * With a_counts = 0.75*q_counts this returns P (the fixed 75 % the low region needs); with
 * a_counts = 0.70P it returns 0.525*P^2/q_counts, which is the plan's DB = 0.525/q. */
#define PD_MOD_QB_K ((int64_t)PD_SW_PERIOD * PD_SW_PERIOD * 3 / 4)   /* 1.5 * period */

static int32_t div_round(int64_t numerator, int64_t denominator)
{
    if (denominator <= 0)
        return 0;
    if (numerator >= 0)
        return (int32_t)((numerator + denominator / 2) / denominator);
    return (int32_t)(-((-numerator + denominator / 2) / denominator));
}

static int32_t b_from_a(int32_t a_counts, int32_t q_counts)
{
    if (q_counts <= 0)
        return (int32_t)PD_OUT_BRIDGE_CMP;
    return div_round((int64_t)a_counts * PD_MOD_QB_K, (int64_t)q_counts * PD_SW_PERIOD);
}

/* Transition-zone A, in counts:
 *     DA = (A0 + A1*offset/span - A2*offset^2/span^2) * period
 * with A0 = 0.65, A1 = 0.10, A2 = 0.05 and offset, span both in counts. The coefficients carry
 * their own scaling (100000 = 1.0), so every term is consistent by construction and there is no
 * second, implicit use of "0.10". The zone width is asserted to be 0.10*period in pd_config.h, so
 * A1 is the same number as the span - which is why this is easier to get right with the
 * coefficients written out than with a pre-multiplied numerator. */
static int32_t transition_a(int32_t offset, int32_t span)
{
    const int64_t full = 100000;                       /* 1.0 */
    const int64_t a0 = 65000;                          /* 0.65 */
    const int64_t a1 = 10000;                          /* 0.10 */
    const int64_t a2 = 5000;                           /* 0.05 */
    int64_t numerator = a0 * span * span + a1 * offset * span -
                        (a2 * offset * offset * span) / span;
    return div_round(numerator * PD_SW_PERIOD, full * span * span);
}

void PD_ModDistribute(int32_t q_counts, PD_ModResult *out)
{
    int32_t q, a_counts, b_counts;
    uint8_t limit = PD_MOD_LIMIT_NONE;

    if (out == 0)
        return;
    out->q_request = q_counts;
    out->q_applied = 0;
    out->cmp_a = (uint16_t)PD_CMP_MIN;
    out->cmp_b = (uint16_t)PD_OUT_BRIDGE_CMP;
    out->zone = PD_MOD_ZONE_NONE;
    out->limit = PD_MOD_LIMIT_INVALID;

    /* A zero request is a power-state-machine decision, not a modulation one: report it and leave
     * the output alone. The caller must not treat the returned pair as a running setpoint. */
    if (q_counts == 0)
    {
        out->limit = PD_MOD_LIMIT_ZERO;
        return;
    }
    if (q_counts < 0)
        return;                                     /* stays INVALID */

    /* 1. limit q itself, before distributing (plan 4.4). */
    q = q_counts;
    if (q > (int32_t)PD_MOD_Q_MAX_COUNTS)
    {
        q = (int32_t)PD_MOD_Q_MAX_COUNTS;
        limit |= PD_MOD_LIMIT_Q_MAX;
    }
    if (q < (int32_t)PD_MOD_Q_MIN_COUNTS)
    {
        q = (int32_t)PD_MOD_Q_MIN_COUNTS;
        limit |= PD_MOD_LIMIT_Q_MIN;
    }

    /* 2. distribute. */
    if (q <= (int32_t)PD_MOD_Q_LOW_COUNTS)
    {
        out->zone = PD_MOD_ZONE_LOW;
        a_counts = q;
        b_counts = (int32_t)PD_OUT_BRIDGE_CMP;
    }
    else if (q >= (int32_t)PD_MOD_Q_HIGH_COUNTS)
    {
        out->zone = PD_MOD_ZONE_HIGH;
        a_counts = (int32_t)PD_CMP_A_MAX;
        b_counts = b_from_a(a_counts, q);
    }
    else
    {
        int32_t span = (int32_t)PD_MOD_Q_HIGH_COUNTS - (int32_t)PD_MOD_Q_LOW_COUNTS;
        out->zone = PD_MOD_ZONE_TRANSITION;
        a_counts = transition_a(q - (int32_t)PD_MOD_Q_LOW_COUNTS, span);
        b_counts = b_from_a(a_counts, q);
    }

    /* 3. physical bounds, on the INTEGER compare values (plan 4.4). */
    if (a_counts > (int32_t)PD_CMP_A_MAX)
    {
        a_counts = (int32_t)PD_CMP_A_MAX;
        limit |= PD_MOD_LIMIT_A_MAX;
    }
    /* Defensive, and honest about it: with A_min <= q_min (asserted in pd_config.h) the q floor
     * above already guarantees A >= 1088 in every zone - the low zone has A = q >= q_min, the
     * transition zone starts at 0.65*period and the high zone is pinned at 0.70*period. So this
     * branch cannot fire today and the reported limit is PD_MOD_LIMIT_Q_MIN, not A_MIN. It is kept
     * because the alternative is a compare register that could go to zero if someone retunes the
     * zones, and a zero A compare is exactly what the board rules forbid. */
    if (a_counts > 0 && a_counts < (int32_t)PD_MOD_A_MIN_COUNTS)
    {
        a_counts = (int32_t)PD_MOD_A_MIN_COUNTS;
        limit |= PD_MOD_LIMIT_A_MIN;
    }
    if (b_counts < (int32_t)PD_MOD_B_MIN_COUNTS)
    {
        b_counts = (int32_t)PD_MOD_B_MIN_COUNTS;
        limit |= PD_MOD_LIMIT_B_MIN;
    }
    if (b_counts > (int32_t)PD_OUT_BRIDGE_CMP)
        b_counts = (int32_t)PD_OUT_BRIDGE_CMP;
    if (a_counts < (int32_t)PD_CMP_MIN)
        a_counts = (int32_t)PD_CMP_MIN;
    if (b_counts < (int32_t)PD_CMP_MIN)
        b_counts = (int32_t)PD_CMP_MIN;

    /* 4. No cross-bridge refusal here, on purpose. A > B is the intended high-zone operation of a
     *    four-switch non-inverting buck-boost (the plan's table reaches DA/DB = 1.2 at q = 0.90),
     *    and plan 5.1 requires the new mode to allow A and B to be equal, to approach each other
     *    and to reorder. Whether a pair is safe is a TIMING question - per-bridge complementary
     *    dead time, the measured A/B edge separation and body-diode freewheeling - which the plan
     *    assigns to the register checks and to bench evidence, not to a blanket clamp here.
     *    An earlier version clamped A down to B above q = 0.75, which silently made the stage
     *    deliver less than the input exactly where the controller asked for more. */

    out->cmp_a = (uint16_t)a_counts;
    out->cmp_b = (uint16_t)b_counts;
    out->q_applied = PD_ModAppliedQ(out->cmp_a, out->cmp_b);
    out->limit = limit;
}

int32_t PD_ModAppliedQ(uint16_t cmp_a, uint16_t cmp_b)
{
    if (cmp_b == 0)
        return 0;
    return div_round((int64_t)cmp_a * (int64_t)PD_OUT_BRIDGE_CMP, (int64_t)cmp_b);
}
