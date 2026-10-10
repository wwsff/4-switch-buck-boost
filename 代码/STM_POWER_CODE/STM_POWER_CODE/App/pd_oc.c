/* OC_N run-length monitor. Pure state machine: no HAL, no registers, so the
 * native test suite can feed it synthetic 10 us samples. */
#include "pd_oc.h"
#include "pd_config.h"

static uint32_t blank_until, low_run, run_max, runs, low_samples, blanked, trips, events;
static uint8_t tripped;

void PD_OcInit(void)
{
    blank_until = low_run = run_max = runs = low_samples = blanked = trips = events = 0;
    tripped = 0;
}
void PD_OcArmBlank(uint32_t now_ms, uint32_t ms)
{
    blank_until = now_ms + ms;
}
uint8_t PD_OcFeed(uint8_t low, uint32_t now_ms)
{
    if (!low)
    {
        if (low_run)
            low_run = 0;
        return 0;
    }
    if (!low_run)
        runs++;
    low_run++;
    low_samples++;
    if (low_run > run_max)
        run_max = low_run; /* a run still in progress counts as its current length */
    if ((int32_t)(now_ms - blank_until) < 0)
    {
        /* Inside the gate-enable window: count it, never act on it. A run that
           continues past the window is re-evaluated below with its full length,
           so a genuinely held OC still stops the stage the moment the window
           closes (the old "== threshold" form silently missed exactly that). */
        blanked++;
        return 0;
    }
    events++;
    if (!tripped && low_run >= PD_DIAG_OC_LOW_SAMPLES)
    {
        tripped = 1;
        trips++;
        return 1;
    }
    return 0;
}
uint32_t PD_OcEvents(void) { return events; }
uint32_t PD_OcBlankedSamples(void) { return blanked; }
uint32_t PD_OcLowSamples(void) { return low_samples; }
uint32_t PD_OcRuns(void) { return runs; }
uint32_t PD_OcMaxRun(void) { return run_max; }
uint32_t PD_OcTrips(void) { return trips; }
