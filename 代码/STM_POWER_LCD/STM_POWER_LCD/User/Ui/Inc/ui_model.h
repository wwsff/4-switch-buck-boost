/**
  ******************************************************************************
  * @file    ui_model.h
  * @brief   Shared display data model.
  *
  *          THE CONTRACT: exactly one task (GuiTask) ever touches the panel.
  *          The 1 ms key task queues input. GuiTask owns CAN protocol state,
  *          this model and all display transactions.
  *
  *          "Output ON" is NEVER set by the HMI on its own: it is only shown
  *          once the G474 has acknowledged and reported the real state.
  ******************************************************************************
  */
#ifndef UI_MODEL_H
#define UI_MODEL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
  UI_SRC_MOCK = 0,      /* simulated data, no power board attached */
  UI_SRC_CAN            /* live data from the G474 */
} ui_source_t;

typedef struct
{
  uint8_t      installed;         /* channel exists on the power board */
  uint8_t      output_on;         /* as REPORTED by the G474, not requested */
  uint8_t      cv_mode;           /* 1 = CV, 0 = CC */
  uint8_t      ready, ready_known, run_state; /* reported by G474, never guessed */
  uint8_t      startup_failed;
  uint8_t      fault_latched;
  uint32_t     fault_bits;        /* CANP_FAULT_* */

  uint16_t     v_out_mv;          /* measured output voltage */
  uint16_t     i_out_ma;          /* measured output current */

  int16_t      tmos_cdeg, tl_cdeg;
  uint8_t      temp_valid, temp_seen;
  uint32_t     temp_rx_ms;         /* separate from output-state freshness */

  uint16_t     v_set_mv;          /* setpoint being edited / sent */
  uint16_t     i_limit_ma;

  /* Limits come from the G474 capability message (0x210) when connected,
   * otherwise these safe defaults are used so the UI still works standalone. */
  uint16_t     v_min_mv;
  uint16_t     v_max_mv;
  uint16_t     i_max_ma;
} ui_channel_t;

typedef struct
{
  ui_source_t  source;
  uint8_t      link_up;           /* receiving 0x200/0x201 from the G474 */
  uint32_t     last_rx_ms;
  uint16_t     seq_out;           /* sequence for the next command */

  ui_channel_t ch1;               /* CH1 is the only channel in this phase */
  uint8_t      ch2_present;       /* from capability flags; normally 0 */
} ui_model_t;

/** @brief  The one global instance. */
ui_model_t *ui_model(void);

/** @brief  Reset to an offline state with conservative editing limits. */
void ui_model_init(void);

/** @brief  Fill the model with plausible simulated values (no CAN). */
void ui_model_step_mock(void);

/** @brief  Human-readable fault string for the status line (never NULL). */
const char *ui_fault_text(uint32_t fault_bits);

#ifdef __cplusplus
}
#endif

#endif /* UI_MODEL_H */
