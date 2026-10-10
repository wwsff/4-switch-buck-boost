/**
  ******************************************************************************
  * @file    can_if.h
  * @brief   Thin CAN transport: a queue of raw frames plus the HAL glue.
  *
  *          Keeps FreeRTOS and the STM32 HAL out of the protocol logic so the
  *          packing/unpacking can be reviewed on its own.
  ******************************************************************************
  */
#ifndef APP_CAN_IF_H
#define APP_CAN_IF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define CANIF_MAX_DLC 8u

typedef struct
{
  uint32_t id;                        /* 11-bit standard identifier */
  uint8_t  dlc;                       /* 0..8 */
  uint8_t  data[CANIF_MAX_DLC];
} canif_frame_t;

/** @brief  Start FDCAN2 and install the receive callback. */
int canif_init(void);

/** @brief  Block up to timeout_ms for a received frame. 1 = got one, 0 = none. */
int canif_receive(canif_frame_t *out, uint32_t timeout_ms);

/** @brief  Queue a frame for transmission (blocks up to timeout_ms if full). */
int canif_send(uint32_t id, const uint8_t *data, uint8_t dlc, uint32_t timeout_ms);

/** @brief  Called from the FDCAN RX interrupt; pushes into the receive queue. */
void canif_rx_from_isr(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_CAN_IF_H */
