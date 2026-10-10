/**
  ******************************************************************************
  * @file    can_proto.h
  * @brief   STM_POWER CAN protocol v1 endpoint for the H750 HMI board.
  ******************************************************************************
  */
#ifndef APP_CAN_PROTO_H
#define APP_CAN_PROTO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "ui_model.h"

/** @brief  Start CAN and put the model into "waiting for the power board". */
int can_proto_init(void);

/** @brief  Run only from GuiTask; it owns protocol state and the UI model. */
void can_proto_poll(uint32_t timeout_ms);

/* Command requests. Each one is turned into a 0x110 frame with a fresh
 * sequence number; the display is NOT updated until the G474 answers. */
void can_proto_request_set(uint16_t v_mv, uint16_t i_ma);
void can_proto_request_start(void);
void can_proto_request_stop(void);
void can_proto_request_clear_fault(void);
void can_proto_request_capability(void);
/** @brief  Consume a setpoint ACK result: +1 accepted, -1 failed, 0 pending. */
int can_proto_take_set_result(uint16_t *v_mv, uint16_t *i_ma);

/** @brief  True while a command is waiting for its acknowledgement. */
uint8_t can_proto_command_pending(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_CAN_PROTO_H */
