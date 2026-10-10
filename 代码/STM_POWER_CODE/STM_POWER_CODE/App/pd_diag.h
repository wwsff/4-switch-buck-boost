#ifndef PD_DIAG_H
#define PD_DIAG_H
#include <stdint.h>

/* The two rotating diagnostic frames (0.3.23). Formatted here rather than inline in
 * pd_board.c so the host suite can render exactly what the operator will read: a frame
 * that stops being emitted, or prints the wrong field, is then a test failure.
 *
 * Field meanings (also in the release notes - the operator reads these to explain a
 * refused or clipped target):
 *   s         channel state            f   fault bits
 *   D         A/B compare values as RAW register counts (0..PD_SW_PERIOD = 21760;
 *             PD_CMP_A_MAX = 15232 is the 70 % ceiling, PD_OUT_BRIDGE_CMP = 16320 the
 *             75 % output-bridge duty). Deliberately NOT passed through PD_LOG_VALUE:
 *             its 999 cap would print "1000" for every realistic duty and the line would
 *             explain nothing.
 *   S         1 = the A-bridge compare sits on PD_CMP_A_MAX (saturated)
 *   IO/VO     measured output current (mA) and voltage (mV)
 *   VI        measured input voltage (mV)
 *   vm        maximum output the MEASURED VIN allows (mV) - the number that explains a
 *             refusal, and NOT always PD_BUCK_VMAX_MV
 *   tgt/ref   accepted target / reference the loop is executing (mV)
 *   ovp       OVP threshold in force (mV)      dn  1 while a descent is supervised
 */
uint16_t PD_DiagStatusLine(char *out, uint16_t n, uint8_t state, uint32_t fault,
                           uint16_t duty_in, uint16_t duty_out, uint8_t saturated,
                           int32_t iout_ma, int32_t vout_mv, int32_t vin_mv,
                           int32_t allowed_vmax_mv);
uint16_t PD_DiagTargetLine(char *out, uint16_t n, int32_t target_mv, int32_t ramp_mv,
                           int32_t ovp_mv, uint8_t down);
uint16_t PD_DiagSamplingLine(char *out, uint16_t n, uint16_t cmp_a, uint16_t cmp_b,
                             uint16_t sample_cnt, int32_t dist_cnt, uint8_t which);
uint16_t PD_DiagControlLine(char *out, uint16_t n, int32_t ref_mv, int32_t vout_f_mv, int32_t ff_cnt,
                            int32_t p_cnt, int32_t i_cnt, int32_t q_cnt, int32_t cmd_cnt,
                            int32_t q_app_cnt, int32_t cmp_out_cnt, uint8_t mod_limit, uint8_t sat);

#endif
