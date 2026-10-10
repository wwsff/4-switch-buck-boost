/**
  ******************************************************************************
  * @file    touch.h
  * @brief   FT6336U capacitive touch driver interface (I2C1, PB8/PB9).
  ******************************************************************************
  */
#ifndef BSP_TOUCH_H
#define BSP_TOUCH_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "bsp_config.h"

#define TOUCH_MAX_POINTS  2u

typedef struct
{
  uint8_t count;                          /* 0 = no touch                    */
  uint16_t x[TOUCH_MAX_POINTS];
  uint16_t y[TOUCH_MAX_POINTS];
} touch_state_t;

/**
  * @brief  Reset the controller and verify it answers on I2C.
  * @retval HAL_OK when the module answers; keep the demo's power-on defaults.
  * @note   Unknown full-byte IDs are logged for bench verification. On failure
  *         check address/configuration, FPC1 and the I2C pull-ups; touch_scan_bus()
  *         reports which addresses answer.
  */
HAL_StatusTypeDef touch_init(void);

/** @brief  Read the device id register (FT6336_ID_*). */
HAL_StatusTypeDef touch_read_id(uint8_t *id);

/** @brief  Read the current touch state. count == 0 means nothing pressed. */
HAL_StatusTypeDef touch_read(touch_state_t *state);

/* Diagnostic data from the same read used for hit testing (no extra poll). */
uint16_t touch_address(void); /* HAL address, shift right once for 7-bit */
uint32_t touch_last_error(void);
HAL_StatusTypeDef touch_last_status(void);
uint8_t touch_last_count(void); /* 0xff before status has been read */

/** @brief  True when the INT line is asserted (active low). */
uint8_t touch_irq_asserted(void);

/**
  * @brief  Probe every 7-bit address on I2C1 and log the ones that answer.
  * @note   Diagnostic for first bring-up; call it once from the GUI task when
  *         touch_init() fails.
  */
void touch_scan_bus(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_TOUCH_H */
