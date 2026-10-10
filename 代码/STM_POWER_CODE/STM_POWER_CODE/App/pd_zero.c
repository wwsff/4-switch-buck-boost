#include "pd_zero.h"
#include "pd_config.h"
#include "pd_core.h"
#include <string.h>
#define NOMINAL_ZERO (1650u * 4095u)
#define ZERO_SAMPLES 4096u
static PD_ZeroStatus z;
/* ADC-code * VREF_mV units: preserves sub-LSB mean without 64-bit fast-path math. */
static volatile uint32_t baseline[2];
static uint32_t started, last_sample, sum, block_sum, block_min, block_max;
static uint16_t raw_min, raw_max;
static volatile uint8_t cancelled;
static uint8_t boot_state, boot_announced, qualified;
static uint32_t qualified_at;
static void fail(uint8_t reason) { z.state = 4; z.reason = reason; }
uint8_t PD_ZeroBusy(void) { return !cancelled && (z.state == 1 || z.state == 2); }
void PD_ZeroInit(void) {
    memset(&z, 0, sizeof(z));
    baseline[0] = baseline[1] = NOMINAL_ZERO;
    cancelled = 0;
    boot_state = PD_ZERO_BOOT_WAIT;
    boot_announced = qualified = 0;
}
void PD_ZeroStart(uint32_t now, uint8_t sensor) {
    if (sensor < 1 || sensor > 2 || PD_ZeroBusy()) return;
    z.state = 1; z.sensor = sensor; z.reason = 0; z.samples = 0;
    cancelled = 0;
    z.pp_uv = z.span_uv = 0;
    started = last_sample = now;
    sum = block_sum = block_max = 0; block_min = UINT32_MAX;
    raw_min = 4095; raw_max = 0;
}
void PD_ZeroReset(uint8_t sensor) {
    if (sensor < 1 || sensor > 2 || PD_ZeroBusy()) return;
    baseline[sensor - 1] = NOMINAL_ZERO;
    z.applied &= (uint8_t)~(1u << (sensor - 1));
    z.sensor = sensor; z.state = 0; z.reason = 0; z.samples = 0;
    cancelled = 0;
    z.pp_uv = z.span_uv = 0;
}
void PD_ZeroCancel(void) { if (PD_ZeroBusy()) cancelled = 1; }
void PD_ZeroFeed(uint32_t now, uint8_t allowed, uint8_t fresh, uint16_t raw) {
    uint32_t mean, block, key;
    if (!PD_ZeroBusy()) return;
    if (!allowed) {
        /* reason 1 covered two different things and the log could not tell them apart: the board
           gate refusing (now reported by zero_allowed with its own reason) and the IL validity bit
           being absent for the IL sensor. Reason 6 is the second one. */
        fail(z.sensor == 2 ? 6 : 1);
        return;
    }
    if (now - started > 8000u) { fail(4); return; }
    if (now - started < 500u) return;
    if (!fresh || now == last_sample) return;
    if (raw <= 8 || raw >= 4087) { fail(2); return; }
    last_sample = now; z.state = 2;
    sum += raw; block_sum += raw; z.samples++;
    if (raw < raw_min) raw_min = raw;
    if (raw > raw_max) raw_max = raw;
    if ((z.samples & 511u) == 0) {
        block = (uint32_t)(((uint64_t)block_sum * PD_VREF_MV) / 512u);
        if (block < block_min) block_min = block;
        if (block > block_max) block_max = block;
        block_sum = 0;
    }
    if (z.samples != ZERO_SAMPLES) return;
    mean = (uint32_t)(((uint64_t)sum * PD_VREF_MV + ZERO_SAMPLES / 2) / ZERO_SAMPLES);
    z.pp_uv = (uint32_t)(((uint64_t)(raw_max - raw_min) * PD_VREF_MV * 1000u) / 4095u);
    z.span_uv = (uint32_t)(((uint64_t)(block_max - block_min) * 1000u) / 4095u);
    /* Bench qualification limits, not sensor datasheet specifications.
       A failed/repeated calibration preserves the previously applied zero. */
    if (mean < 1450u * 4095u || mean > 1850u * 4095u) { fail(2); return; }
    if (z.pp_uv > (z.sensor == 1 ? 100000u : 300000u) ||
        z.span_uv > (z.sensor == 1 ? 5000u : 10000u)) { fail(3); return; }
    key = PD_PortLock();
    if (!cancelled) {
        baseline[z.sensor - 1] = mean;
        z.applied |= (uint8_t)(1u << (z.sensor - 1));
        z.state = 3;
    }
    PD_PortUnlock(key);
}
void PD_ZeroGet(PD_ZeroStatus *out) {
    *out = z;
    if (cancelled) { out->state = 4; out->reason = 5; }
    out->iout_uv = (uint32_t)(((uint64_t)baseline[0] * 1000u) / 4095u);
    out->il_uv = (uint32_t)(((uint64_t)baseline[1] * 1000u) / 4095u);
}
int32_t PD_ZeroCurrent(uint8_t sensor, uint16_t raw) {
    int32_t scale = sensor == 1 ? 5 : 10;
    if (sensor < 1 || sensor > 2) return 0;
    return (((int32_t)raw * PD_VREF_MV - (int32_t)baseline[sensor - 1]) * scale) / 4095;
}
uint8_t PD_ZeroApplied(uint8_t sensor) {
    if (sensor < 1 || sensor > 2) return 0;
    return (uint8_t)((z.applied >> (sensor - 1)) & 1u);
}
uint8_t PD_ZeroStartupState(void) { return boot_state; }
void PD_ZeroStartupRetry(void) {
    if (!PD_ZeroApplied(1) && !PD_ZeroBusy()) {
        boot_state = PD_ZERO_BOOT_WAIT;
        boot_announced = qualified = 0;
    }
}
uint8_t PD_ZeroStartupTick(uint32_t now, uint8_t allowed) {
    PD_ZeroStatus status;
    if (!boot_announced) {
        boot_announced = 1;
        return PD_ZERO_BOOT_WAIT_EVENT;
    }
    if (PD_ZeroApplied(1)) {
        if (boot_state != PD_ZERO_BOOT_READY) {
            boot_state = PD_ZERO_BOOT_READY;
            return PD_ZERO_BOOT_READY_EVENT;
        }
        return PD_ZERO_BOOT_NONE;
    }
    if (boot_state == PD_ZERO_BOOT_READY || boot_state == PD_ZERO_BOOT_FAILED)
        return PD_ZERO_BOOT_NONE;
    if (PD_ZeroBusy()) {
        PD_ZeroGet(&status);
        if (status.sensor == 1) boot_state = PD_ZERO_BOOT_RUN;
        qualified = 0;
        return PD_ZERO_BOOT_NONE;
    }
    if (boot_state == PD_ZERO_BOOT_RUN) {
        PD_ZeroGet(&status);
        qualified = 0;
        if (status.state == 4 && status.reason != 1 && status.reason != 5) {
            boot_state = PD_ZERO_BOOT_FAILED; /* bad zero/noise/timeout: keep locked */
            return PD_ZERO_BOOT_FAILED_EVENT;
        }
        boot_state = PD_ZERO_BOOT_WAIT;
        return PD_ZERO_BOOT_WAIT_EVENT;
    }
    if (!allowed) { qualified = 0; return PD_ZERO_BOOT_NONE; }
    if (!qualified) { qualified_at = now; qualified = 1; return PD_ZERO_BOOT_NONE; }
    if ((uint32_t)(now - qualified_at) < 500u) return PD_ZERO_BOOT_NONE;
    qualified = 0;
    PD_ZeroStart(now, 1u);
    boot_state = PD_ZERO_BOOT_RUN;
    return PD_ZERO_BOOT_START_EVENT;
}
