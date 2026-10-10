/**
  ******************************************************************************
  * @file    app_bringup.h
  * @brief   Per-peripheral acceptance test for first bring-up.
  *
  *          Run this BEFORE any UI work. It answers, in order:
  *            1. does the panel light up, and are colours/orientation right?
  *            2. does the FT6336 answer on I2C, and where does it report a touch?
  *            3. does the encoder count and in which direction?
  *            4. are the keys readable and debounced?
  *            5. does CAN come up at 500 kbit/s?
  *
  *          Each step is reported on the USART2 log so the result is visible
  *          even when the display is the thing under test.
  ******************************************************************************
  */
#ifndef APP_BRINGUP_H
#define APP_BRINGUP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
  * @brief  Run the whole acceptance sequence once.
  * @note   Blocking; call from the GUI task before entering its loop.
  */
void bringup_run(void);

/** @brief  1 ms tick: keep the log timestamp and poll the encoder/keys. */
void bringup_tick_1ms(void);

/** @brief  Latest encoder position (free running, wraps at int16). */
int16_t bringup_encoder(void);

/** @brief  Counts seen in the last 1 ms - verifies direction and speed. */
int16_t bringup_encoder_delta(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_BRINGUP_H */
