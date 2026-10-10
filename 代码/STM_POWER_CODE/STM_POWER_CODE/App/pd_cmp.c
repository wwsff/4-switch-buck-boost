/* Compare-commit policy + statistics. Pure module: the host test suite links it
 * and drives the same window/retry decisions with a simulated counter. */
#include "pd_cmp.h"
#include "pd_config.h"

static volatile uint32_t commits, retries, timeouts, stopped, param_errors;
static volatile uint32_t guard_expired, wait_timeouts, crit_max, crit_last;
static volatile uint16_t last_mcntr;
static volatile uint8_t last_reason, stage;

void PD_CmpInit(void)
{
    commits = retries = timeouts = stopped = param_errors = 0;
    guard_expired = wait_timeouts = crit_max = crit_last = 0;
    last_mcntr = 0;
    last_reason = PD_CMP_OK;
    stage = PD_CMP_STAGE_NONE;
}
uint8_t PD_CmpWindowAccepts(uint16_t mcntr)
{
    /* Both writes must finish before the master update event that transfers the
       preload registers. PD_CMP_LATEST leaves PD_CMP_COMMIT_MARGIN counts of
       slack for the masked section (measured at runtime as crit_cycles). */
    if (PD_CMP_LATEST == 0u)
        return 0;
    return (uint8_t)(mcntr <= (uint16_t)PD_CMP_LATEST);
}
uint8_t PD_CmpRetryAllowed(uint32_t attempts)
{
    return (uint8_t)(attempts < PD_CMP_MAX_RETRIES);
}
void PD_CmpSetStage(uint8_t s) { stage = s; }
void PD_CmpNoteCommit(uint16_t mcntr, uint32_t crit_cycles)
{
    commits++;
    last_mcntr = mcntr;
    last_reason = PD_CMP_OK;
    crit_last = crit_cycles;
    if (crit_cycles > crit_max)
        crit_max = crit_cycles;
}
void PD_CmpNoteRetry(uint16_t mcntr, uint32_t crit_cycles)
{
    /* A retry is NOT a failure: it deliberately leaves last_reason alone so the
       1 Hz log still shows the last real outcome (ok/timeout/stopped/param). */
    retries++;
    last_mcntr = mcntr;
    crit_last = crit_cycles;
    if (crit_cycles > crit_max)
        crit_max = crit_cycles;
}
void PD_CmpNoteTimeout(uint16_t mcntr, uint8_t guard)
{
    timeouts++;
    if (guard)
        guard_expired++;
    last_mcntr = mcntr;
    last_reason = PD_CMP_ERR_TIMEOUT;
}
void PD_CmpNoteStopped(uint16_t mcntr)
{
    stopped++;
    last_mcntr = mcntr;
    last_reason = PD_CMP_ERR_STOPPED;
}
void PD_CmpNoteParam(void)
{
    param_errors++;
    last_reason = PD_CMP_ERR_PARAM;
}
void PD_CmpNoteWaitTimeout(void) { wait_timeouts++; }
uint32_t PD_CmpCommits(void) { return commits; }
uint32_t PD_CmpRetries(void) { return retries; }
uint32_t PD_CmpTimeouts(void) { return timeouts; }
uint32_t PD_CmpStopped(void) { return stopped; }
uint32_t PD_CmpParamErrors(void) { return param_errors; }
uint32_t PD_CmpGuardExpired(void) { return guard_expired; }
uint32_t PD_CmpWaitTimeouts(void) { return wait_timeouts; }
uint32_t PD_CmpCritMax(void) { return crit_max; }
uint32_t PD_CmpCritLast(void) { return crit_last; }
uint16_t PD_CmpLastCounter(void) { return last_mcntr; }
uint8_t PD_CmpLastReason(void) { return last_reason; }
uint8_t PD_CmpStage(void) { return stage; }
