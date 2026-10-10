#ifndef PD_NTC_H
#define PD_NTC_H
#include <stdint.h>
#include <math.h>

/* Power-board R38/R39: HNTC0603-103F3450FA, R25=10k, B25/85=3450 K.
 * R33/R34 are 10k to the same 3.3V rail as VDDA; NTC goes to GND.
 * Rntc/R25 = ADC/(4095-ADC). Supply cancels; do not apply voltage calibration.
 * This is the beta approximation, not a measured R/T calibration or junction
 * temperature. Call in task context at 10 Hz, never in the fast ADC ISR. */
#define PD_NTC_INVALID (-32768)
#define PD_NTC_PERIOD_MS 100u
/* Conservative integer ADC bounds of the -40..125 C beta model. */
#define PD_NTC_RAW_MIN 213u
#define PD_NTC_RAW_MAX 3938u
static inline int16_t PD_NtcConvert(uint16_t raw)
{
    float ratio, c;
    if (raw < PD_NTC_RAW_MIN || raw > PD_NTC_RAW_MAX) return PD_NTC_INVALID;
    ratio = (float)raw / (4095.0f - (float)raw);
    c = 1.0f / (1.0f / 298.15f + logf(ratio) / 3450.0f) - 273.15f;
    if (!isfinite(c) || c < -40.0f || c > 125.0f) return PD_NTC_INVALID;
    return (int16_t)(c * 100.0f + (c >= 0.0f ? 0.5f : -0.5f));
}
#endif
