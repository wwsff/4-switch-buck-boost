/* The two rotating diagnostic frames (0.3.23), in a module the host suite can build.
 *
 * Why they live here instead of inline in pd_board.c: pd_board.c is bound to the STM32
 * HAL and is not part of the host DLL, so the 1 Hz rotation could not be rendered in a
 * test at all. That is how 0.3.23 lost the target/reference pair - splitting code-17
 * dropped it, the format-length gate cannot see a MISSING line, and nothing could execute
 * the rotation.
 *
 * Keeping the formatting here means the exact bytes the operator reads are produced by
 * code the host suite links, so a dropped frame or a wrong field is a test failure
 * instead of a bench surprise. */
#include "pd_diag.h"
#include <stdio.h>

uint16_t PD_DiagStatusLine(char *out, uint16_t n, uint8_t state, uint32_t fault,
                           uint16_t duty_in, uint16_t duty_out, uint8_t saturated,
                           int32_t iout_ma, int32_t vout_mv, int32_t vin_mv,
                           int32_t allowed_vmax_mv)
{
    int written = snprintf(out, n,
                           "s=%u f=%02lX D=%u/%u S=%u IO=%ld VO=%ld VI=%ld vm=%ld",
                           (unsigned)state, (unsigned long)fault, (unsigned)duty_in,
                           (unsigned)duty_out, (unsigned)saturated, (long)iout_ma,
                           (long)vout_mv, (long)vin_mv, (long)allowed_vmax_mv);
    if (written < 0)
        return 0;
    return (uint16_t)(written < (int)n ? written : (int)n - 1);
}

uint16_t PD_DiagTargetLine(char *out, uint16_t n, int32_t target_mv, int32_t ramp_mv,
                           int32_t ovp_mv, uint8_t down)
{
    int written = snprintf(out, n, "tgt=%ld ref=%ld ovp=%ld dn=%u",
                           (long)target_mv, (long)ramp_mv, (long)ovp_mv, (unsigned)down);
    if (written < 0)
        return 0;
    return (uint16_t)(written < (int)n ? written : (int)n - 1);
}

/* The 1 Hz control trace. Every field is in compare counts except ref/vo (mV), sat and the
 * limit mask. The shape changed once in 0.3.28, when the actuator stopped being a single duty:
 * "fu" was dropped because q_cnt carries the same information (the pre-modulator output) plus
 * the exact integer the distributor receives, and q/b/lim were added so the trace can answer
 * "the loop asked for this, the actuator got that, and here is what stopped it".
 *
 *   r  executed reference, mV        a   A-bridge compare value committed
 *   v  filtered feedback, mV         qr  q the loop ASKED the modulator for, in counts
 *   f  feed-forward term             q   q the committed pair represents (0.75*A/B)
 *   p  proportional term             b   B-bridge compare value committed
 *   i  integral term                 lim PD_MOD_LIMIT_* mask, 0 = nothing was limited
 *   s  saturation reason (0 none, 1 ceiling engaged, 2 floor engaged,
 *      3 = the integrator itself is on its clamp)
 * qr vs q is the pair that matters: when they differ, lim says which bound did it, so the bench
 * no longer has to infer the actuator's state from a single duty number.
 * Worst case is asserted by tools/check_log_lengths.py, not by eye. */
uint16_t PD_DiagControlLine(char *out, uint16_t n, int32_t ref_mv, int32_t vout_f_mv, int32_t ff_cnt,
                            int32_t p_cnt, int32_t i_cnt, int32_t q_cnt, int32_t cmd_cnt,
                            int32_t q_app_cnt, int32_t cmp_out_cnt, uint8_t mod_limit, uint8_t sat)
{
    int written = snprintf(out, n,
                           "CT r=%ld v=%ld f=%ld p=%ld i=%ld a=%ld qr=%ld q=%ld b=%ld lim=0x%02X s=%u",
                           (long)ref_mv, (long)vout_f_mv, (long)ff_cnt, (long)p_cnt, (long)i_cnt,
                           (long)cmd_cnt, (long)q_cnt, (long)q_app_cnt, (long)cmp_out_cnt,
                           (unsigned)mod_limit, (unsigned)sat);
    if (written < 0)
        return 0;
    return (uint16_t)(written < (int)n ? written : (int)n - 1);
}

/* ADC edge-proximity line (0.3.32). Reported, never acted on: the fields are the two committed
 * compare values, which of them is inside the sampling window, and how close it is in ADC
 * counts. "e=" is 0x1 for the A bridge and 0x2 for the B bridge.
 *
 *   a  A-bridge compare value (counts)        d  signed distance to the sampling instant
 *   b  B-bridge compare value (counts)        e  0x1 A edge near, 0x2 B edge near
 *   s  sampling instant in counts
 * Worst case is asserted by tools/check_log_lengths.py, not by eye. */
uint16_t PD_DiagSamplingLine(char *out, uint16_t n, uint16_t cmp_a, uint16_t cmp_b,
                             uint16_t sample_cnt, int32_t dist_cnt, uint8_t which)
{
    int written = snprintf(out, n, "ADCEDGE a=%u b=%u s=%u d=%ld e=0x%02X",
                           (unsigned)cmp_a, (unsigned)cmp_b, (unsigned)sample_cnt,
                           (long)dist_cnt, (unsigned)which);
    if (written < 0)
        return 0;
    return (uint16_t)(written < (int)n ? written : (int)n - 1);
}
