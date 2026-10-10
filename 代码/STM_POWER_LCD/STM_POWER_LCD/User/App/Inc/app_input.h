/**
  ******************************************************************************
  * @file    app_input.h
  * @brief   Debounced physical input events (encoder, keys) for the UI layer.
  *
  *          app_bringup owns the 1 ms polling and debounce; it publishes
  *          events here. The GUI task registers one handler. This keeps the
  *          input layer free of any UI knowledge.
  ******************************************************************************
  */
#ifndef APP_INPUT_H
#define APP_INPUT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
  IN_EVT_ENCODER = 0,   /* value = signed detent delta */
  IN_EVT_KEY1,
  IN_EVT_KEY2,
  IN_EVT_ENC_SW
} input_event_t;

typedef enum
{
  IN_ACT_RELEASED = 0,
  IN_ACT_PRESSED,
  IN_ACT_LONG_PRESS
} input_action_t;

typedef void (*input_handler_t)(input_event_t evt, input_action_t act, int16_t value);

/** @brief  Register the handler. Pass NULL to unregister. */
void input_set_handler(input_handler_t handler);

/** @brief  Enqueue from the 1 ms key task; never calls UI code there. */
void input_publish(input_event_t evt, input_action_t act, int16_t value);
/** @brief  Dispatch queued events in GuiTask. */
void input_drain(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_INPUT_H */
