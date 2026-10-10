/**
  ******************************************************************************
  * @file    ui_main.h
  * @brief   Main screen: measured values, setpoints, output state, faults.
  ******************************************************************************
  */
#ifndef UI_MAIN_H
#define UI_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** @brief  Register the input handler (encoder, keys). Call once at start-up. */
void ui_main_init(void);

/** @brief  Draw the whole screen from the current model state. */
void ui_main_draw_all(void);

/** @brief  Redraw only what changed since the previous call. */
void ui_main_update(void);

#ifdef __cplusplus
}
#endif

#endif /* UI_MAIN_H */
