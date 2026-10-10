#ifndef PD_OC_H
#define PD_OC_H
#include <stdint.h>
/* OC_N (TMCS1133 over-current) run-length monitor.
 *
 * The pin is sampled at 100 kHz from the injected ADC ISR. A single switching
 * transient must not stop the stage, but a genuinely held over-current must, so
 * the decision is made on the LENGTH of the low run, not on one sample.
 *
 * PD_OcFeed() returns 1 exactly once, when a run that is outside the blank
 * window reaches PD_DIAG_OC_LOW_SAMPLES (1 ms). The blank window is armed at
 * gate enable: the 0.3.16 bench showed a ~160 us disturbance there.
 *
 * This module is deliberately pure (no HAL, no registers) so the host test
 * suite can drive it with synthetic sample sequences.
 */
void PD_OcInit(void);
/* Arm a blank window starting now_ms. */
void PD_OcArmBlank(uint32_t now_ms, uint32_t ms);
/* Feed one 10 us sample. low != 0 means OC_N is asserted (active low). */
uint8_t PD_OcFeed(uint8_t low, uint32_t now_ms);
uint32_t PD_OcEvents(void);
uint32_t PD_OcBlankedSamples(void);
uint32_t PD_OcLowSamples(void);
uint32_t PD_OcRuns(void);
uint32_t PD_OcMaxRun(void);
uint32_t PD_OcTrips(void);
#endif
