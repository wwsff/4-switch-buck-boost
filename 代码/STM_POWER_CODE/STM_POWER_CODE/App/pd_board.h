#ifndef PD_BOARD_H
#define PD_BOARD_H
#include <stdint.h>
void PD_BoardInit(void);
void PD_BoardPoll(void);
void PD_BoardADCIRQ(void);
void PD_BoardFaultIRQ(void);
void PD_BoardEmergencyStop(void);
/* Release the ADC-fault debounce so CLEAR_FAULT can succeed. Called from CLEAR_FAULT
   BEFORE the fault bit is cleared; without it the command answers NOT_READY for ever. */
void PD_BoardClearAdcFault(void);
uint32_t PD_PortIlOverRun(void);
/* Release the IL over-run counter so CLEAR_FAULT can succeed after an IL trip. */
void PD_BoardClearIlRun(void);
/* Which sub-condition failed, and the four ages, on the tick that decides.
   Bit 1 not initialised, 2 too few pairs, 4 injected stale, 8 regular stale,
   16 input rank stale, 32 debounced ADC error streak. 0 = healthy. */
uint32_t PD_PortSampleFailMask(uint32_t *fast_age, uint32_t *slow_age0,
                               uint32_t *slow_age1, uint32_t *input_age);
/* Power-stage port primitives. Only the 1 kHz task calls the first three;
   the gate-test pair is called from the protocol dispatch. */
void PD_PortPowerEnable(uint8_t ch, uint8_t on);
void PD_PortGatesEnable(uint8_t ch, uint8_t on);
/* Commit both bridges' CMP1 in one master update event. Returns PD_CMP_OK or a
   PD_CMP_ERR_* reason (see pd_cmp.h). A window that closes before the masked
   write is retried on the next period (nothing was written); the bounded retry
   limit and the wait guard both surface as PD_CMP_ERR_TIMEOUT, and the caller
   must then keep the outputs disabled. Never re-enables anything. */
uint8_t PD_PortSetCompare(uint8_t ch, uint16_t cmp_in, uint16_t cmp_out);
/* Wait for n master-period rollovers. Returns 0 on timeout. */
uint8_t PD_PortWaitPeriods(uint8_t n);
/* Bridge drive mode.
 *   high_on_output2: the CONFIRMED gate map drives the physical high side from
 *                    OUT2 instead of OUT1 (inverted phase).
 *   static_low_side: hold the physical high side inactive and its complementary
 *                    output active, with NO compare source involved.
 * The static state exists because a compare value cannot express "high side
 * permanently off": the smallest legal value (PD_CMP_MIN) still produces a
 * pulse, and values below it are outside the documented HRTIM range and may not
 * generate a compare event at all (which leaves the high side on for the whole
 * period). See pd_config.h PD_CMP_MIN. */
void PD_PortBridgeMode(uint8_t ch, uint8_t high_on_output2, uint8_t static_low_side);
/* Returns 0 with EN/gates off on timeout or a latched/live CH1 fault. */
uint8_t PD_PortGateTestStart(uint8_t index);
void PD_PortGateTestStop(void);
#endif
