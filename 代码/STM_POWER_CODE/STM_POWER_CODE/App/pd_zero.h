#ifndef PD_ZERO_H
#define PD_ZERO_H
#include <stdint.h>
/* CH1 only. Sensor 1=IOUT, 2=IL. No automatic input-current zeroing. */
typedef struct {
    uint8_t state, sensor, applied, reason;
    uint16_t samples;
    uint32_t iout_uv, il_uv, pp_uv, span_uv;
} PD_ZeroStatus;
void PD_ZeroInit(void);
uint8_t PD_ZeroBusy(void);
void PD_ZeroStart(uint32_t now, uint8_t sensor);
void PD_ZeroReset(uint8_t sensor);
void PD_ZeroCancel(void);
void PD_ZeroFeed(uint32_t now, uint8_t allowed, uint8_t fresh, uint16_t raw);
void PD_ZeroGet(PD_ZeroStatus *out);
int32_t PD_ZeroCurrent(uint8_t sensor, uint16_t raw);
/* 1 when a measured (not nominal) zero is applied for that sensor. */
uint8_t PD_ZeroApplied(uint8_t sensor);
enum { PD_ZERO_BOOT_WAIT, PD_ZERO_BOOT_RUN, PD_ZERO_BOOT_READY, PD_ZERO_BOOT_FAILED };
enum { PD_ZERO_BOOT_NONE, PD_ZERO_BOOT_WAIT_EVENT, PD_ZERO_BOOT_START_EVENT,
       PD_ZERO_BOOT_READY_EVENT, PD_ZERO_BOOT_FAILED_EVENT };
/* Board supplies the existing OFF/VIN/VOUT/sampling interlock. One startup
 * calibration; temporary condition loss/cancellation returns to WAIT. */
uint8_t PD_ZeroStartupTick(uint32_t now, uint8_t allowed);
uint8_t PD_ZeroStartupState(void);
void PD_ZeroStartupRetry(void); /* explicit CLEAR while OFF; never erases a valid zero */
#endif
