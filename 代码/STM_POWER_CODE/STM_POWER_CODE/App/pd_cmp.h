#ifndef PD_CMP_H
#define PD_CMP_H
#include <stdint.h>
/* Compare-commit policy and statistics (portable: no HAL, no registers, so the
 * host suite can drive the same decisions with a simulated HRTIM counter).
 *
 * Why a window at all: CMP1xR of TIMER A and TIMER B are preloaded and both are
 * transferred to their active registers by the MASTER timer update event (both
 * timers are configured PreloadEnable=ENABLED, UpdateTrigger1=MASTER,
 * ResetUpdate=DISABLED). Writing the pair is only coherent if BOTH writes finish
 * before that same update event; otherwise the first value lands on one boundary
 * and the second on the next one, and the two bridges run a period with
 * mismatched duty. Masking interrupts does not stop the HRTIM counter, so the
 * only sound guarantee is an explicit time margin - see PD_CMP_COMMIT_MARGIN.
 */
#define PD_CMP_OK 0u
#define PD_CMP_ERR_PARAM 1u   /* wrong channel or HRTIM clock gated */
#define PD_CMP_ERR_STOPPED 2u /* fault/stop won the race: nothing was written */
#define PD_CMP_ERR_TIMEOUT 3u /* bounded retries or the wait guard expired */

/* Commit stage, recorded with the statistics so a failure says where it came
 * from: power-up precharge, the precharge->PWM transition, or the 1 kHz loop. */
#define PD_CMP_STAGE_NONE 0u
#define PD_CMP_STAGE_START 1u
#define PD_CMP_STAGE_PWM 2u
#define PD_CMP_STAGE_LOOP 3u

void PD_CmpInit(void);
/* Rate-limit: may the two writes start now? mcntr is the master counter value. */
uint8_t PD_CmpWindowAccepts(uint16_t mcntr);
/* Rate-limit: is another retry allowed after `attempts` refused attempts? */
uint8_t PD_CmpRetryAllowed(uint32_t attempts);
void PD_CmpSetStage(uint8_t stage);
void PD_CmpNoteCommit(uint16_t mcntr, uint32_t crit_cycles);
void PD_CmpNoteRetry(uint16_t mcntr, uint32_t crit_cycles);
void PD_CmpNoteTimeout(uint16_t mcntr, uint8_t guard_expired);
void PD_CmpNoteStopped(uint16_t mcntr);
void PD_CmpNoteParam(void);
void PD_CmpNoteWaitTimeout(void);
uint32_t PD_CmpCommits(void);
uint32_t PD_CmpRetries(void);
uint32_t PD_CmpTimeouts(void);
uint32_t PD_CmpStopped(void);
uint32_t PD_CmpParamErrors(void);
uint32_t PD_CmpGuardExpired(void);
uint32_t PD_CmpWaitTimeouts(void);
uint32_t PD_CmpCritMax(void);
uint32_t PD_CmpCritLast(void);
uint16_t PD_CmpLastCounter(void);
uint8_t PD_CmpLastReason(void);
uint8_t PD_CmpStage(void);
#endif
