#ifndef PD_CORE_H
#define PD_CORE_H
#include "pd_pid.h"
#include "powerdesk_protocol.h"
#include <stdint.h>
typedef struct
{
    uint32_t uptime_ms, fault;
    uint16_t valid;
    int32_t vin_mv, iin_ma, vout_mv, iout_ma, il_ma, vref_mv, iref_ma, ilref_ma;
    int16_t tmos_cdeg, tl_cdeg;
    uint16_t duty_in, duty_out;
    uint8_t state, mode, enabled, ready;
} PD_Channel;
extern PD_Channel pd_channels[2];
void PD_Init(void);
void PD_Feed(const uint8_t *bytes, uint16_t size);
void PD_ResetReceiver(void);
void PD_Poll(void);
void PD_LatchFault(uint8_t channel, uint32_t fault);
/* ---- CAN transport entry points ---------------------------------------------
   PD_CanRequest() runs the SAME dispatch() as the UART path, so range checks,
   the VIN reachability test and every fault-latch rule are shared; the CAN layer
   only translates bytes. PD_CanReply points at the reply payload of the most
   recent PD_CanRequest() call (read only, valid until the next call).
   PD_CanHeartbeat() refreshes the shared communication watchdog. */
uint16_t PD_CanRequest(uint8_t op, uint8_t ch, const uint8_t *p, uint16_t n);
extern const uint8_t *const PD_CanReply;
void PD_CanHeartbeat(void);
/* Record 256 inductor-current samples at 100 kSPS from the next sample on; the
   1 kHz task logs first/min/max/last when it completes. Used to capture the
   power-stage start transient. */
void PD_TraceAutoArm(void);
void PD_FastSample(uint8_t channel, int32_t il_ma, int32_t vout_mv, int32_t vin_mv, int32_t iout_ma,
                   uint8_t valid);
void PD_Log(uint8_t channel, uint8_t level, uint16_t code, const char *text);
/* Task-only; code 22 uses the priority queue. Return 0 to retry later. */
int PD_TryLog(uint8_t channel, uint8_t level, uint16_t code, const char *text);
/* Voltage-channel conversion (0.3.22), pure and unit-testable: raw ADC code ->
 * millivolts through the VIN/VOUT divider, the measured reference
 * (PD_VOLTAGE_SCALE_REF_MV) and the per-channel gain/offset hook. Control,
 * telemetry, start checks and OVP all consume the RESULT, so no caller may
 * apply a second correction. Current channels do NOT use these (they keep
 * PD_VREF_MV and the pd_zero.c zero store). */
int32_t PD_ConvertVinMv(uint16_t raw);
int32_t PD_ConvertVoutMv(uint16_t raw);
/* Board boundary: TX copies before returning; all ISR-facing functions bounded. */
uint32_t PD_PortNow(void);
uint32_t PD_PortLock(void);
void PD_PortUnlock(uint32_t key);
int PD_PortTransmit(const uint8_t *data, uint16_t length, uint8_t priority);
void PD_PortStop(uint8_t channel);
/* Bounded first-fault evidence, called with interrupts masked before stop. */
void PD_PortFaultSnapshot(uint8_t channel, uint32_t fault);
void PD_PortStartMark(void);
uint32_t PD_PortFaultPins(uint8_t channel);
uint8_t PD_PortSamplingHealthy(void);
/* Milliseconds since the last completed 100 kHz injected pair. The 1 kHz loop
   stops the stage above PD_LOOP_STALE_MS instead of using stale feedback. */
uint32_t PD_PortFastAge(void);
void PD_PortDeviceID(char out[24]);
uint16_t PD_PortZero(uint8_t action);
#endif
