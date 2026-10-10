/**
  ******************************************************************************
  * @file    ui_model.c
  * @brief   Display data model and mock data source.
  ******************************************************************************
  */
#include "ui_model.h"
#include "can_protocol.h"

#include <string.h>

static ui_model_t s_model;

ui_model_t *ui_model(void)
{
  return &s_model;
}

void ui_model_init(void)
{
  (void)memset(&s_model, 0, sizeof(s_model));

  s_model.source   = UI_SRC_CAN;
  s_model.link_up  = 0u;
  s_model.seq_out  = 1u;

  ui_channel_t *c = &s_model.ch1;
  c->installed = 0u; /* unknown/absent until the power board reports CH1 */
  c->output_on = 0u;
  c->cv_mode   = 1u;

  /* Conservative standalone limits. The real ones arrive in 0x210 and the UI
   * must always prefer those - never a compiled-in constant. */
  c->v_min_mv  = 0u;
  c->v_max_mv  = 30000u;
  c->i_max_ma  = 5000u;

  c->v_set_mv   = 5000u;
  c->i_limit_ma = 1000u;
  c->v_out_mv   = 0u;
  c->i_out_ma   = 0u;
}

void ui_model_step_mock(void)
{
  if (s_model.source != UI_SRC_MOCK) {
    return;
  }
  ui_channel_t *c = &s_model.ch1;

  if (!c->output_on) {
    c->v_out_mv = 0u;
    c->i_out_ma = 0u;
    c->cv_mode  = 1u;
    return;
  }

  /* Called once per GUI loop. Steps are per-call, not per-millisecond, so the
   * ramp rate does not silently depend on the task period. */
  const uint16_t v_step = 40u;    /* mV per call */
  const uint16_t i_step = 15u;    /* mA per call */

  if (c->v_out_mv < c->v_set_mv) {
    uint16_t d = (uint16_t)(c->v_set_mv - c->v_out_mv);
    c->v_out_mv = (uint16_t)(c->v_out_mv + ((d > v_step) ? v_step : d));
  } else if (c->v_out_mv > c->v_set_mv) {
    uint16_t d = (uint16_t)(c->v_out_mv - c->v_set_mv);
    c->v_out_mv = (uint16_t)(c->v_out_mv - ((d > v_step) ? v_step : d));
  }

  /* Current rises until the programmed limit, which is CC. */
  if (c->i_out_ma < c->i_limit_ma) {
    uint16_t d = (uint16_t)(c->i_limit_ma - c->i_out_ma);
    c->i_out_ma = (uint16_t)(c->i_out_ma + ((d > i_step) ? i_step : d));
  }
  c->cv_mode = (c->i_out_ma >= c->i_limit_ma) ? 0u : 1u;
}

const char *ui_fault_text(uint32_t fault_bits)
{
  if (fault_bits == 0u) {
    return "OK";
  }
  /* Ordered by what the operator must act on first. The bit names below are the
   * HMI's; the power board maps its own PD_FAULTS_* onto them in pd_can.c
   * (pd_can_map_fault). Two of them are worth spelling out because the names
   * differ between the two projects:
   *   power board PD_FAULTS_SENSOR_ALERT -> CANP_FAULT_ALERT_N  (hardware pin)
   *   power board PD_FAULTS_ADC          -> CANP_FAULT_SENSOR   (sampler)
   *   power board PD_FAULTS_INTERNAL     -> CANP_FAULT_INTERNAL */
  if ((fault_bits & CANP_FAULT_OTP) != 0u)         { return "OVER TEMP"; }
  if ((fault_bits & CANP_FAULT_OVP) != 0u)         { return "OVER VOLT"; }
  if ((fault_bits & CANP_FAULT_OCP) != 0u)         { return "OVER CURR"; }
  if ((fault_bits & CANP_FAULT_OPP) != 0u)         { return "OVER POWR"; }
  if ((fault_bits & CANP_FAULT_UVLO) != 0u)        { return "LOW INPUT"; }
  if ((fault_bits & CANP_FAULT_REVERSE) != 0u)     { return "REVERSED"; }
  if ((fault_bits & CANP_FAULT_ALERT_N) != 0u)     { return "HW ALERT"; }
  if ((fault_bits & CANP_FAULT_CAN_TIMEOUT) != 0u) { return "LINK LOST"; }
  if ((fault_bits & CANP_FAULT_SENSOR) != 0u)      { return "SENSOR"; }
  if ((fault_bits & CANP_FAULT_INTERNAL) != 0u)    { return "INTERNAL"; }
  return "FAULT";
}
