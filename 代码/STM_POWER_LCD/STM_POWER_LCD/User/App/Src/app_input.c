/**
  ******************************************************************************
  * @file    app_input.c
  * @brief   Event plumbing for physical inputs.
  ******************************************************************************
  */
#include "app_input.h"
#include "FreeRTOS.h"
#include "queue.h"

#include <stddef.h>

static input_handler_t s_handler;
typedef struct {
  input_event_t evt;
  input_action_t act;
  int16_t value;
} input_msg_t;
static QueueHandle_t s_input_queue;

void input_set_handler(input_handler_t handler)
{
  if (s_input_queue == NULL) {
    s_input_queue = xQueueCreate(32u, sizeof(input_msg_t));
  }
  s_handler = handler;
}

void input_publish(input_event_t evt, input_action_t act, int16_t value)
{
  input_msg_t msg = { evt, act, value };
  if (s_input_queue != NULL) {
    (void)xQueueSendToBack(s_input_queue, &msg, 0u);
  }
}

void input_drain(void)
{
  input_msg_t msg;
  if (s_input_queue == NULL) { return; }
  while (xQueueReceive(s_input_queue, &msg, 0u) == pdTRUE) {
    if (s_handler != NULL) { s_handler(msg.evt, msg.act, msg.value); }
  }
}
