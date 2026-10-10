#ifndef PD_CAN_PORT_H
#define PD_CAN_PORT_H
/* FDCAN2 glue for the power board CAN transport. Hardware-facing: this file is
 * NOT covered by the host compile gate (it needs the HAL). The protocol codec it
 * uses, App/pd_can.c, is host-compilable and IS covered. */
#include <stdint.h>

/* Configure the filter, start FDCAN2. Returns 0 on success. Call once after
   MX_FDCAN2_Init() and after the RTOS is running is fine; it does not block. */
int PD_CanPortInit(void);

/* Drain the receive FIFO, answer commands, send periodic telemetry and fault
   reports. Call from the same task that calls PD_BoardPoll(). Never blocks. */
void PD_CanPortPoll(void);

/* Send the capability frames for both channels once (used at start-up). */
void PD_CanPortAnnounce(void);

#endif
