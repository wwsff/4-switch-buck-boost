/**
  ******************************************************************************
  * @file    ui_main.c
  * @brief   Main screen renderer.
  *
  *          Screen layout, 320x240 landscape:
  *
  *            POWER      CAN / NO DEVICE           <- one output only
  *            -----------------------------------------
  *            VOLTAGE              CURRENT
  *            12.34 V              1.00 A              <- bitmap font
  *            SET VOLTAGE          SET CURRENT
  *            12.00 V              1.00 A
  *            OUT ON   CV      status line
  *            <keys hint>
  *
  *          Only changed rows are re-sent, and only from the GUI task.
  ******************************************************************************
  */
#include "ui_main.h"
#include "ui_model.h"
#include "gfx_text.h"
#include "lcd.h"

#include <string.h>
#include "app_input.h"
#include "can_proto.h"
#include "can_protocol.h"
#include "touch.h"
#include "bsp_log.h"


/* --------------------------------------------------------------------------
 * Colours
 * -------------------------------------------------------------------------- */
#define COL_BG      LCD_BLACK
#define COL_FG      LCD_WHITE
#define COL_DIM     0x8410u   /* grey   */
#define COL_OK      LCD_GREEN
#define COL_WARN    LCD_YELLOW
#define COL_ERR     LCD_RED
#define COL_CYAN    0x07FFu

/* --------------------------------------------------------------------------
 * Layout (pixels)
 * -------------------------------------------------------------------------- */
#define ROW_HEAD_Y   4u
#define ROW_LABEL_Y  32u
#define ROW_VALUE_Y  54u
#define ROW_SET_Y    100u
#define ROW_SET_VALUE_Y 120u
#define ROW_STATUS_Y 158u
#define ROW_HINT_Y   220u
#define COL_VOLT_X   8u
#define COL_AMP_X    168u
#define FIELD_W      144u

/* Three distinct hit rectangles, with gaps matching their visual bounds. */
#define BTN_Y        182u
#define BTN_W        96u
#define BTN_H        32u
#define BTN_X0       8u
#define BTN_X1       112u
#define BTN_X2       216u

#define SCALE_BIG    2u
#define SCALE_LABEL  1u
#define SCALE_SMALL  1u

/* The encoder edits one decimal place at a time. KEY2 selects V/A, KEY1 sends
 * the staged pair, and only an accepted ACK updates the confirmed baseline.
 * s_conf_v / s_conf_i restore the last acknowledged values after failure. */
static uint8_t  s_setpoint_pending;
static uint8_t  s_waiting_ack;
static uint16_t s_conf_v, s_conf_i;

/* --------------------------------------------------------------------------
 * Setpoint editing
 * -------------------------------------------------------------------------- */
typedef enum { FIELD_VOLTAGE = 0, FIELD_CURRENT } edit_field_t;

static edit_field_t s_field = FIELD_VOLTAGE;
static uint8_t s_digit; /* 0: 0.01; 1: 0.1; 2: 1; 3: 10 */
static const uint16_t s_v_steps_mv[4] = {10u, 100u, 1000u, 10000u};
static const uint16_t s_i_steps_ma[3] = {10u, 100u, 1000u};

/* forward declarations (definitions further down) */
static void draw_row(uint16_t y, const char *s, uint8_t scale, uint16_t fg, uint16_t bg);
static void draw_field(uint16_t x, uint16_t y, uint16_t width, const char *s,
                       uint8_t scale, uint16_t fg, uint16_t bg);
static void fmt_fixed(uint32_t value, uint8_t decimals, char *out, uint32_t out_sz);
static uint8_t s_temp_page;
static uint8_t s_setpoint_dirty;
static uint8_t s_frame_dirty;
static uint8_t s_full_redraw;
static uint8_t s_touch_down;
static uint32_t s_touch_error_at;
static uint8_t s_touch_error_logged;
static uint32_t s_marker_error_at;
static uint8_t s_marker_error_logged;
static uint8_t s_touch_health; /* 0=read OK, 1=I2C/parse error, 2=coordinate range */
static uint8_t s_prev_touch_health;

/* Temporary overlay, owned by GuiTask. Each press keeps its own 5-second
 * lifetime. If all slots are occupied, replace the oldest mark. */
#define TOUCH_MARK_COUNT 64u
#define TOUCH_MARK_SIZE  5u
#define TOUCH_MARK_MS    5000u
typedef struct {
  uint16_t x, y; /* actual centre; kept 2 pixels away from the edges */
  uint32_t pressed_at;
  uint8_t active;
} touch_mark_t;
static touch_mark_t s_marks[TOUCH_MARK_COUNT];
static HAL_StatusTypeDef restore_touch_mark(const touch_mark_t *mark);

static void expire_touch_marks(void)
{
  uint32_t now = HAL_GetTick();
  for (uint32_t i = 0u; i < TOUCH_MARK_COUNT; i++) {
    if (s_marks[i].active && (uint32_t)(now - s_marks[i].pressed_at) >= TOUCH_MARK_MS) {
      if (restore_touch_mark(&s_marks[i]) == HAL_OK) {
        s_marks[i].active = 0u;
      }
      s_frame_dirty = 1u; /* reapply surviving overlapping marks */
    }
  }
}

static void add_touch_mark(uint16_t x, uint16_t y, uint16_t *cx, uint16_t *cy)
{
  uint32_t now = HAL_GetTick();
  uint32_t slot = 0u, oldest = 0u;
  for (uint32_t i = 0u; i < TOUCH_MARK_COUNT; i++) {
    if (!s_marks[i].active) { slot = i; break; }
    if ((uint32_t)(now - s_marks[i].pressed_at) >= oldest) {
      oldest = (uint32_t)(now - s_marks[i].pressed_at);
      slot = i;
    }
  }
  if (s_marks[slot].active && restore_touch_mark(&s_marks[slot]) != HAL_OK) {
    *cx = s_marks[slot].x; *cy = s_marks[slot].y;
    return; /* keep the old slot for retry instead of leaving a stale red point */
  }
  *cx = (x < 2u) ? 2u : ((x > LCD_W - 3u) ? LCD_W - 3u : x);
  *cy = (y < 2u) ? 2u : ((y > LCD_H - 3u) ? LCD_H - 3u : y);
  s_marks[slot].x = *cx;
  s_marks[slot].y = *cy;
  s_marks[slot].pressed_at = now;
  s_marks[slot].active = 1u;
  s_frame_dirty = 1u;
}

static void draw_touch_marks(void)
{
  if (!s_frame_dirty) { return; }
  s_frame_dirty = 0u;
  for (uint32_t i = 0u; i < TOUCH_MARK_COUNT; i++) {
    if (s_marks[i].active &&
        lcd_fill_rect((uint16_t)(s_marks[i].x - 2u), (uint16_t)(s_marks[i].y - 2u),
                      TOUCH_MARK_SIZE, TOUCH_MARK_SIZE, LCD_RED) != HAL_OK) {
      s_frame_dirty = 1u; /* retry the overlay on the next GUI tick */
    }
  }
  if (s_frame_dirty && (!s_marker_error_logged ||
      (uint32_t)(HAL_GetTick() - s_marker_error_at) >= 1000u)) {
    LOGE("LCD ", "touch mark write_failed; retrying");
    s_marker_error_at = HAL_GetTick();
    s_marker_error_logged = 1u;
  }
}

static uint8_t map_touch(uint16_t raw_x, uint16_t raw_y, uint16_t *x, uint16_t *y)
{
  *x = TOUCH_SWAP_XY ? raw_y : raw_x;
  *y = TOUCH_SWAP_XY ? raw_x : raw_y;
  if (*x >= LCD_W || *y >= LCD_H) { return 0u; }
  if (TOUCH_MIRROR_X) { *x = (uint16_t)(LCD_W - 1u - *x); }
  if (TOUCH_MIRROR_Y) { *y = (uint16_t)(LCD_H - 1u - *y); }
  return 1u;
}

/**
  * @brief  Keep the setpoint inside what the power board says it can do.
  * @note   Limits arrive in the 0x210 capability message, so a power-board
  *         change never requires an HMI reflash.
  */
static void clamp_setpoints(ui_channel_t *c)
{
  if (c->v_set_mv < c->v_min_mv)   { c->v_set_mv = c->v_min_mv; }
  if (c->v_set_mv > c->v_max_mv)   { c->v_set_mv = c->v_max_mv; }
  if (c->i_limit_ma > c->i_max_ma) { c->i_limit_ma = c->i_max_ma; }
  if (c->i_limit_ma < 10u)         { c->i_limit_ma = 10u; }
}

/* Mark the row dirty; GuiTask paints it on its next update. */
static void redraw_setpoint(ui_channel_t *c)
{
  (void)c;
  s_setpoint_dirty = 1u;
}

static void on_input(input_event_t evt, input_action_t act, int16_t value)
{
  ui_model_t   *m = ui_model();
  ui_channel_t *c = &m->ch1;
  uint8_t live = (m->link_up && c->installed) ? 1u : 0u;

  if (s_temp_page) { return; } /* read-only page: no invisible setpoint edits */

  switch (evt)
  {
    case IN_EVT_ENCODER:
      if (act != IN_ACT_PRESSED) { return; }
      if (s_waiting_ack) { return; }
      if (s_field == FIELD_VOLTAGE) {
        int32_t v = (int32_t)c->v_set_mv + ((int32_t)value * s_v_steps_mv[s_digit]);
        if (v < (int32_t)c->v_min_mv) { v = (int32_t)c->v_min_mv; }
        if (v > (int32_t)c->v_max_mv) { v = (int32_t)c->v_max_mv; }
        c->v_set_mv = (uint16_t)v;
      } else {
        int32_t v = (int32_t)c->i_limit_ma + ((int32_t)value * s_i_steps_ma[s_digit]);
        if (v < 10) { v = 10; }
        if (v > (int32_t)c->i_max_ma) { v = (int32_t)c->i_max_ma; }
        c->i_limit_ma = (uint16_t)v;
      }
      clamp_setpoints(c);
      s_setpoint_pending = 1u;      /* edited, not yet committed */
      redraw_setpoint(c);
      break;

    case IN_EVT_ENC_SW:
      if (act == IN_ACT_PRESSED) {
        s_digit = (uint8_t)((s_digit + 1u) %
                 ((s_field == FIELD_VOLTAGE) ? 4u : 3u));
        redraw_setpoint(c);
      }
      break;

    case IN_EVT_KEY1:
      if (act == IN_ACT_PRESSED) {
        if (live && !s_waiting_ack) {
          s_waiting_ack = 1u;
          can_proto_request_set(c->v_set_mv, c->i_limit_ma);
          LOGI("CH1 ", "setpoint_sent v=%umV limit=%umA",
               (unsigned)c->v_set_mv, (unsigned)c->i_limit_ma);
        } else if (!live) {
          LOGW("CH1 ", "setpoint_not_sent: CAN offline");
        }
        redraw_setpoint(c);
      }
      break;

    case IN_EVT_KEY2:
      if (act == IN_ACT_PRESSED) {
        s_field = (s_field == FIELD_VOLTAGE) ? FIELD_CURRENT : FIELD_VOLTAGE;
        s_digit = 0u;
        redraw_setpoint(c);
      }
      break;

    default:
      break;
  }
}

/* --------------------------------------------------------------------------
 * Touch buttons
 *
 * Raw portrait coordinates rotate with the LCD into 320x240. VERIFY on the
 * real module using the red mark/log. Axis swap/mirroring belong in the
 * TOUCH_* configuration, so drawing and button hit testing stay consistent.
 * -------------------------------------------------------------------------- */
static void draw_button(uint16_t x, const char *label, uint16_t fg, uint16_t bg)
{
  s_frame_dirty = 1u;
  (void)lcd_fill_rect(x, BTN_Y, BTN_W, BTN_H, bg);
  uint16_t tw = gfx_text_width((uint32_t)strlen(label), 1u);
  uint16_t tx = (uint16_t)(x + ((BTN_W > tw) ? ((BTN_W - tw) / 2u) : 0u));
  uint16_t ty = (uint16_t)(BTN_Y + ((BTN_H - gfx_text_height(1u)) / 2u));
  (void)gfx_text_draw(tx, ty, label, 1u, fg, bg);
}

static void draw_buttons(void)
{
  ui_model_t *m = ui_model();
  ui_channel_t *c = &m->ch1;

  draw_button(BTN_X0, c->output_on ? "STOP" : "START",
              c->output_on ? COL_ERR : ((m->link_up && c->installed && !c->fault_latched && !c->startup_failed &&
                   (!c->ready_known || c->ready)) ? COL_OK : COL_DIM), COL_BG);
  draw_button(BTN_X1, "CLR", COL_WARN, COL_BG);
  draw_button(BTN_X2, s_temp_page ? "BACK" : "SET", COL_CYAN, COL_BG);

}

/* Returns 0 = START/STOP, 1 = CLEAR, 2 = SEND SETPOINT, 0xFF = outside */
static uint8_t touch_to_button(uint16_t x, uint16_t y)
{
  if ((y < BTN_Y) || (y >= (BTN_Y + BTN_H))) {
    return 0xFFu;
  }
  if (x >= BTN_X0 && x < BTN_X0 + BTN_W) { return 0u; }
  if (x >= BTN_X1 && x < BTN_X1 + BTN_W) { return 1u; }
  if (x >= BTN_X2 && x < BTN_X2 + BTN_W) { return 2u; }
  return 0xFFu;
}

static void handle_touch(void)
{
  uint16_t down_x, down_y, cx, cy;
  touch_state_t ts;

  if (touch_read(&ts) != HAL_OK) {
    s_touch_health = 1u;
    uint32_t now = HAL_GetTick();
    if (!s_touch_error_logged || (uint32_t)(now - s_touch_error_at) >= 1000u) {
      LOGW("TOUCH", "read_failed a7=0x%02X st=%u err=0x%08lX cnt=0x%02X",
           (unsigned)(touch_address() >> 1), (unsigned)touch_last_status(),
           (unsigned long)touch_last_error(), (unsigned)touch_last_count());
      s_touch_error_at = now;
      s_touch_error_logged = 1u;
    }
    return;
  }

  if (ts.count == 0u) {
    s_touch_health = 0u;
    if (s_touch_down) { LOGI("TOUCH", "released"); }
    s_touch_down = 0u;
    return;
  }
  if (s_touch_down != 0u) {
    return;                       /* already handled this press */
  }
  s_touch_down = 1u;
  if (!map_touch(ts.x[0], ts.y[0], &down_x, &down_y)) {
    s_touch_health = 2u;
    LOGW("TOUCH", "press raw=(%u,%u) outside LCD; ignored",
         (unsigned)ts.x[0], (unsigned)ts.y[0]);
    return;
  }
  s_touch_health = 0u;
  add_touch_mark(down_x, down_y, &cx, &cy);

  ui_model_t   *m = ui_model();
  ui_channel_t *c = &m->ch1;
  uint8_t live = (m->link_up && c->installed) ? 1u : 0u;
  uint8_t navigation = (down_y < 24u && down_x < 104u);
  uint8_t back = (s_temp_page && touch_to_button(down_x, down_y) == 2u);
  const char *area = (navigation || back) ? (s_temp_page ? "POWER_PAGE" : "TEMP_PAGE") : "NONE";
  uint8_t value_area = (!s_temp_page && ((down_y >= 28u && down_y < 96u) ||
                        (down_y >= 98u && down_y < 154u)));
  uint8_t voltage_area = (value_area && down_x < LCD_W / 2u);
  uint8_t current_area = (value_area && down_x >= LCD_W / 2u);
  uint8_t button = touch_to_button(down_x, down_y);
  if (voltage_area) { area = "VOLTAGE"; }
  else if (current_area) { area = "CURRENT"; }
  else if (button == 0u) { area = c->output_on ? "STOP" : "START"; }
  else if (button == 1u) { area = "CLEAR"; }
  else if (button == 2u && !s_temp_page) { area = "SET"; }
  LOGI("TOUCH", "press raw=(%u,%u) lcd=(%u,%u) center=(%u,%u) area=%s",
       (unsigned)ts.x[0], (unsigned)ts.y[0], (unsigned)down_x, (unsigned)down_y,
       (unsigned)cx, (unsigned)cy, area);

  if (navigation || back) {
    s_temp_page = (uint8_t)!s_temp_page;
    (void)memset(s_marks, 0, sizeof(s_marks)); /* page owns its overlay */
    add_touch_mark(down_x, down_y, &cx, &cy);
    s_full_redraw = 1u;
    LOGI("UI  ", "page=%s", s_temp_page ? "TEMPERATURE" : "POWER");
    return;
  }

  /* Touch either measurement area or its half of the SET row to choose field. */
  if (voltage_area) {
    s_field = FIELD_VOLTAGE;
    s_digit = 0u;
    redraw_setpoint(c);
    return;
  }
  if (current_area) {
    s_field = FIELD_CURRENT;
    s_digit = 0u;
    redraw_setpoint(c);
    return;
  }

  switch (button)
  {
    case 0u:
      if (live != 0u) {
        if (c->output_on != 0u) { can_proto_request_stop(); }
        else                    { can_proto_request_start(); }
      }
      break;

    case 1u:
      if (live != 0u) { can_proto_request_clear_fault(); }
      break;

    case 2u:
      /* The on-screen SET button is itself a deliberate confirmation. */
      if (live != 0u && !s_waiting_ack) {
        s_waiting_ack = 1u;
        can_proto_request_set(c->v_set_mv, c->i_limit_ma);
      }
      redraw_setpoint(c);
      break;

    default:
      break;
  }
}

void ui_main_init(void)
{
  ui_channel_t *c = &ui_model()->ch1;
  /* Baseline the confirmation values on the starting setpoints. Without this a
   * cancel before the first confirm would restore zeros. */
  s_conf_v = c->v_set_mv;
  s_conf_i = c->i_limit_ma;
  s_setpoint_pending = 0u;
  s_waiting_ack = 0u;
  s_digit = 0u;
  s_temp_page = 0u;
  s_field = FIELD_VOLTAGE;
  s_touch_down = 0u;
  s_touch_error_logged = 0u;
  s_marker_error_logged = 0u;
  s_touch_health = 0u;
  s_prev_touch_health = 0xFFu;
  (void)memset(s_marks, 0, sizeof(s_marks));
  s_full_redraw = 1u;
  s_frame_dirty = 0u;
  input_set_handler(on_input);
}

/* --------------------------------------------------------------------------
 * Integer formatting without printf (keeps ~5 KB of C library out of flash)
 * -------------------------------------------------------------------------- */

/* "12.34" for 12340 with 2 decimals */
static void fmt_fixed(uint32_t value, uint8_t decimals, char *out, uint32_t out_sz)
{
  char  tmp[12];
  int   n = 0;
  uint32_t div = 1u;

  for (uint8_t i = 0u; i < decimals; i++) {
    div *= 10u;
  }

  uint32_t ip = value / div;
  uint32_t fp = value % div;

  /* integer part, reversed */
  char ipbuf[12];
  int  m = 0;
  if (ip == 0u) {
    ipbuf[m++] = '0';
  }
  while ((ip > 0u) && (m < 11)) {
    ipbuf[m++] = (char)('0' + (ip % 10u));
    ip /= 10u;
  }
  for (int i = m - 1; i >= 0 && n < 11; i--) {
    tmp[n++] = ipbuf[i];
  }

  if (decimals > 0u) {
    if (n < 11) { tmp[n++] = '.'; }
    /* fractional part, zero padded, most significant first */
    for (int i = (int)decimals - 1; i >= 0; i--) {
      uint32_t p = 1u;
      for (int k = 0; k < i; k++) { p *= 10u; }
      if (n < 11) { tmp[n++] = (char)('0' + ((fp / p) % 10u)); }
    }
  }

  uint32_t copy = ((uint32_t)n < (out_sz - 1u)) ? (uint32_t)n : (out_sz - 1u);
  (void)memcpy(out, tmp, copy);
  out[copy] = '\0';
}

/* --------------------------------------------------------------------------
 * Cached strings so we only redraw changed rows
 * -------------------------------------------------------------------------- */
static char s_prev_volt[16];
static char s_prev_amp[16];
static char s_prev_set[2][24];
static char s_prev_status[24];
static char s_prev_head[24];
static uint16_t s_prev_status_col;
static uint8_t s_prev_out, s_prev_available;
static char s_prev_step[48];

static void sample_text(uint16_t x, uint16_t y, uint16_t tx, uint16_t ty,
                         const char *text, uint8_t scale, uint16_t fg, uint16_t *colour)
{
  if (x >= tx && y >= ty) {
    (void)gfx_text_pixel((uint16_t)(x - tx), (uint16_t)(y - ty), text,
                         scale, fg, COL_BG, colour);
  }
}

/* Reconstruct the last PAINTED scene from its caches, not from new telemetry.
 * The normal update below then paints any newer values. Only the 25 pixels
 * covered by a mark are sent to the panel; no full framebuffer/readback. */
static uint16_t scene_pixel(uint16_t x, uint16_t y)
{
  uint16_t colour = COL_BG;
  char text[32];
  if ((y == 24u && x >= 8u && x < LCD_W - 8u) ||
      (x == LCD_W / 2u && y >= 28u && y < 154u)) { colour = COL_DIM; }
  sample_text(x, y, COL_VOLT_X, ROW_LABEL_Y, s_temp_page ? "MOS TEMP" : "VOLTAGE", SCALE_LABEL, COL_DIM, &colour);
  sample_text(x, y, COL_AMP_X, ROW_LABEL_Y, s_temp_page ? "INDUCTOR" : "CURRENT", SCALE_LABEL, COL_DIM, &colour);
  (void)strcpy(text, s_temp_page ? "TEMP   " : "POWER> ");
  (void)strncat(text, s_prev_head, sizeof(text) - strlen(text) - 1u);
  uint16_t head_col = (strcmp(s_prev_head, "CAN") == 0) ? COL_OK :
                      ((strcmp(s_prev_head, "MOCK") == 0) ? COL_WARN : COL_ERR);
  sample_text(x, y, 8u, ROW_HEAD_Y, text, SCALE_LABEL, head_col, &colour);
  (void)strcpy(text, s_prev_volt); (void)strcat(text, s_temp_page ? "C" : "V");
  sample_text(x, y, COL_VOLT_X, ROW_VALUE_Y, text, SCALE_BIG, COL_FG, &colour);
  (void)strcpy(text, s_prev_amp); (void)strcat(text, s_temp_page ? "C" : "A");
  sample_text(x, y, COL_AMP_X, ROW_VALUE_Y, text, SCALE_BIG, COL_CYAN, &colour);
  for (uint8_t field = 0u; field < 2u; field++) {
    uint16_t sx = (field == 0u) ? COL_VOLT_X : COL_AMP_X;
    uint16_t col = s_temp_page ? COL_DIM : ((s_prev_set[field][0] == '>') ? COL_FG : COL_DIM);
    sample_text(x, y, sx, ROW_SET_Y, s_temp_page ? ((field == 0u) ? "MOS SENSOR" : "IND SENSOR") :
                 ((field == 0u) ? "SET VOLTAGE" : "SET CURRENT"),
                 SCALE_LABEL, col, &colour);
    if (s_temp_page) col = (strcmp(s_prev_set[field], "OK") == 0) ? COL_OK :
                           ((strcmp(s_prev_set[field], "SENSOR ERR") == 0) ? COL_ERR : COL_DIM);
    sample_text(x, y, sx, ROW_SET_VALUE_Y, s_prev_set[field], s_temp_page ? 1u : SCALE_BIG, col, &colour);
  }
  sample_text(x, y, 8u, ROW_STATUS_Y, s_prev_status, SCALE_SMALL, s_prev_status_col, &colour);
  sample_text(x, y, 8u, ROW_HINT_Y, s_prev_step, SCALE_SMALL, COL_DIM, &colour);
  const char *labels[3] = { s_prev_out ? "STOP" : "START", "CLR", s_temp_page ? "BACK" : "SET" };
  const uint16_t positions[3] = { BTN_X0, BTN_X1, BTN_X2 };
  const uint16_t colours[3] = {
    s_prev_out ? COL_ERR : (s_prev_available ? COL_OK : COL_DIM), COL_WARN, COL_CYAN
  };
  for (uint8_t button = 0u; button < 3u; button++) {
    uint16_t tw = gfx_text_width((uint32_t)strlen(labels[button]), 1u);
    uint16_t tx = (uint16_t)(positions[button] + (BTN_W - tw) / 2u);
    sample_text(x, y, tx, BTN_Y + (BTN_H - GFX_GLYPH_H) / 2u,
                 labels[button], 1u, colours[button], &colour);
  }
  sample_text(x, y, 208u, ROW_HEAD_Y,
               (s_prev_touch_health == 1u) ? "TOUCH ERR" :
               ((s_prev_touch_health == 2u) ? "TOUCH MAP" : ""), 1u, COL_WARN, &colour);
  return colour;
}

static HAL_StatusTypeDef restore_touch_mark(const touch_mark_t *mark)
{
  uint16_t pixels[TOUCH_MARK_SIZE * TOUCH_MARK_SIZE];
  uint16_t x = (uint16_t)(mark->x - 2u), y = (uint16_t)(mark->y - 2u);
  for (uint16_t row = 0u; row < TOUCH_MARK_SIZE; row++) {
    for (uint16_t col = 0u; col < TOUCH_MARK_SIZE; col++) {
      pixels[row * TOUCH_MARK_SIZE + col] = scene_pixel((uint16_t)(x + col), (uint16_t)(y + row));
    }
  }
  return lcd_flush_rgb565(x, y, TOUCH_MARK_SIZE, TOUCH_MARK_SIZE, pixels);
}

static void draw_field(uint16_t x, uint16_t y, uint16_t width, const char *s,
                       uint8_t scale, uint16_t fg, uint16_t bg)
{
  s_frame_dirty = 1u;
  uint16_t h = gfx_text_height(scale);
  /* Clear this field only; its neighbour has its own cache and pixels. */
  (void)lcd_fill_rect(x, y, width, h, bg);
  (void)gfx_text_draw(x, y, s, scale, fg, bg);
}

static void draw_row(uint16_t y, const char *s, uint8_t scale, uint16_t fg, uint16_t bg)
{
  draw_field(8u, y, LCD_W - 16u, s, scale, fg, bg);
}

static uint8_t row_changed(char *cache, uint32_t cache_sz, const char *s)
{
  if (strncmp(cache, s, cache_sz) == 0) {
    return 0u;
  }  (void)strncpy(cache, s, cache_sz - 1u);
  cache[cache_sz - 1u] = '\0';
  return 1u;
}

/* --------------------------------------------------------------------------
 * Drawing
 * -------------------------------------------------------------------------- */
static void draw_base(void)
{
  (void)lcd_fill(COL_BG);

  s_prev_volt[0]   = '\0';
  s_prev_amp[0]    = '\0';
  s_prev_set[0][0] = '\0';
  s_prev_set[1][0] = '\0';
  s_prev_status[0] = '\0';
  s_prev_head[0]   = '\0';
  s_prev_status_col = 0u;
  s_prev_out = 0xFFu;
  s_prev_available = 0xFFu;
  s_prev_step[0] = '\0';
  s_prev_touch_health = 0xFFu;

  draw_field(COL_VOLT_X, ROW_LABEL_Y, FIELD_W, s_temp_page ? "MOS TEMP" : "VOLTAGE", SCALE_LABEL, COL_DIM, COL_BG);
  draw_field(COL_AMP_X, ROW_LABEL_Y, FIELD_W, s_temp_page ? "INDUCTOR" : "CURRENT", SCALE_LABEL, COL_DIM, COL_BG);
  (void)lcd_fill_rect(8u, 24u, LCD_W - 16u, 1u, COL_DIM);
  (void)lcd_fill_rect(LCD_W / 2u, 28u, 1u, 126u, COL_DIM);

  draw_buttons();

}

/* Temperature pages share the scene caches, so a 5x5 mark can be restored
 * without reading the panel or redrawing the screen. Negative values are signed. */
static void temperature_text(int16_t cdeg, uint8_t valid, char *out)
{
  char number[16];
  if (!valid) { (void)strcpy(out, "--.-"); return; }
  int32_t v = cdeg;
  uint32_t mag = (uint32_t)(v < 0 ? -v : v);
  fmt_fixed((mag + 5u) / 10u, 1u, number, sizeof(number));
  out[0] = '\0';
  if (v < 0) (void)strcpy(out, "-");
  (void)strcat(out, number);
}

static void update_temperature(uint8_t available, uint8_t startable)
{
  ui_channel_t *c = &ui_model()->ch1;
  uint8_t fresh = (uint8_t)(available && c->temp_seen &&
                           (uint32_t)(HAL_GetTick() - c->temp_rx_ms) <= CANP_TEMP_STALE_MS);
  const char *source = available ? "CAN" : "NO DEVICE";
  if (row_changed(s_prev_head, sizeof(s_prev_head), source)) {
    char head[24] = "TEMP   ";
    (void)strcat(head, source);
    draw_row(ROW_HEAD_Y, head, 1u, available ? COL_OK : COL_ERR, COL_BG);
    s_prev_touch_health = 0xFFu;
  }
  for (uint8_t sensor = 0u; sensor < 2u; sensor++) {
    uint8_t valid = (uint8_t)(fresh && (c->temp_valid & (1u << sensor)));
    int16_t value = sensor ? c->tl_cdeg : c->tmos_cdeg;
    char number[16], line[20];
    temperature_text(value, valid, number);
    char *cache = sensor ? s_prev_amp : s_prev_volt;
    uint16_t x = sensor ? COL_AMP_X : COL_VOLT_X;
    if (row_changed(cache, sizeof(s_prev_volt), number)) {
      (void)strcpy(line, number); (void)strcat(line, "C");
      draw_field(x, ROW_VALUE_Y, FIELD_W, line, 2u, sensor ? COL_CYAN : COL_FG, COL_BG);
    }
    const char *state = !available ? "NO DEVICE" : !c->temp_seen ? "WAIT DATA" :
                        !fresh ? "STALE" : valid ? "OK" : "SENSOR ERR";
    uint16_t sensor_col = (strcmp(state, "OK") == 0) ? COL_OK :
                          ((strcmp(state, "SENSOR ERR") == 0) ? COL_ERR : COL_DIM);
    if (row_changed(s_prev_set[sensor], sizeof(s_prev_set[sensor]), state)) {
      draw_field(x, ROW_SET_Y, FIELD_W, sensor ? "IND SENSOR" : "MOS SENSOR", 1u, COL_DIM, COL_BG);
      draw_field(x, ROW_SET_VALUE_Y, FIELD_W, state, 1u, sensor_col, COL_BG);
    }
  }
  const char *state = !available ? "NO DEVICE" : c->fault_latched ? ui_fault_text(c->fault_bits) :
                     c->run_state == CANP_STATE_STARTING ? "STARTING" :
                     c->run_state == CANP_STATE_STOPPING ? "STOPPING" :
                     c->output_on ? "OUT ON" : "OUT OFF";
  uint16_t col = (!available || c->fault_latched) ? COL_ERR : COL_DIM;
  if (row_changed(s_prev_status, sizeof(s_prev_status), state) || col != s_prev_status_col) {
    s_prev_status_col = col;
    draw_row(ROW_STATUS_Y, state, 1u, col, COL_BG);
  }
  if (s_prev_out != c->output_on || s_prev_available != startable) {
    s_prev_out = c->output_on; s_prev_available = startable;
    draw_buttons();
  }
  if (row_changed(s_prev_step, sizeof(s_prev_step), "TOP=POWER  NTC TEMPERATURE"))
    draw_row(ROW_HINT_Y, s_prev_step, 1u, COL_DIM, COL_BG);
}

static void touch_health_and_marks(void)
{
  handle_touch();
  if (s_touch_health != s_prev_touch_health) {
    s_prev_touch_health = s_touch_health;
    draw_field(208u, ROW_HEAD_Y, 104u,
               (s_touch_health == 1u) ? "TOUCH ERR" : ((s_touch_health == 2u) ? "TOUCH MAP" : ""),
               1u, COL_WARN, COL_BG);
  }
  draw_touch_marks();
}

void ui_main_draw_all(void)
{
  s_full_redraw = 1u;
  ui_main_update();
}

void ui_main_update(void)
{
  ui_model_t *m = ui_model();
  ui_channel_t *c = &m->ch1;
  char buf[24];   /* reused by each row below */
  uint8_t available = (m->link_up && c->installed) ? 1u : 0u;
  uint8_t startable = (available && !c->fault_latched && !c->startup_failed &&
                       (!c->ready_known || c->ready)) ? 1u : 0u;
  expire_touch_marks();
  if (s_full_redraw) {
    s_full_redraw = 0u;
    draw_base();
  }
  uint16_t applied_v = 0u, applied_i = 0u;
  int set_result = can_proto_take_set_result(&applied_v, &applied_i);
  if (set_result > 0) {
    c->v_set_mv = applied_v;
    c->i_limit_ma = applied_i;
    s_conf_v = applied_v;
    s_conf_i = applied_i;
    s_setpoint_pending = 0u;
    s_waiting_ack = 0u;
    redraw_setpoint(c);
  } else if (set_result < 0) {
    c->v_set_mv = s_conf_v;
    c->i_limit_ma = s_conf_i;
    clamp_setpoints(c);
    s_setpoint_pending = 0u;
    s_waiting_ack = 0u;
    redraw_setpoint(c);
    LOGW("CH1 ", "setpoint_not_applied; restored confirmed value");
  }

  if (s_temp_page) {
    update_temperature(available, startable);
    touch_health_and_marks();
    return;
  }

  /* ---- header: channel + data source. Never pretend mock data is live. ---- */
  const char *src;
  uint16_t src_col;
  if (m->source == UI_SRC_MOCK) {
    src = "MOCK";
    src_col = COL_WARN;
  } else if (available) {
    src = "CAN";
    src_col = COL_OK;
  } else {
    src = "NO DEVICE";
    src_col = COL_ERR;
  }
  if (row_changed(s_prev_head, sizeof(s_prev_head), src)) {
    char head[24];
    (void)strcpy(head, "POWER> ");
    uint32_t i = 7u;
    for (const char *p = src; (*p != '\0') && (i < sizeof(head) - 1u); p++) {
      head[i++] = *p;
    }
    head[i] = '\0';
    /* clear whole header band then draw */
    draw_row(ROW_HEAD_Y, head, SCALE_LABEL, src_col, COL_BG);
    s_prev_touch_health = 0xFFu;
  }

  /* ---- big voltage ---- */
  fmt_fixed(available ? ((uint32_t)c->v_out_mv + 5u) / 10u : 0u, 2u, buf, sizeof(buf));
  if (row_changed(s_prev_volt, sizeof(s_prev_volt), buf)) {
    char line[20];
    uint32_t i = 0u;
    for (const char *p = buf; (*p != '\0') && (i < sizeof(line) - 3u); p++) {
      line[i++] = *p;
    }
    line[i++] = 'V';
    line[i] = '\0';
    draw_field(COL_VOLT_X, ROW_VALUE_Y, FIELD_W, line, SCALE_BIG, COL_FG, COL_BG);
  }

  /* ---- big current ---- */
  fmt_fixed(available ? ((uint32_t)c->i_out_ma + 5u) / 10u : 0u, 2u, buf, sizeof(buf));
  if (row_changed(s_prev_amp, sizeof(s_prev_amp), buf)) {
    char line[20];
    uint32_t i = 0u;
    for (const char *p = buf; (*p != '\0') && (i < sizeof(line) - 3u); p++) {
      line[i++] = *p;
    }
    line[i++] = 'A';
    line[i] = '\0';
    draw_field(COL_AMP_X, ROW_VALUE_Y, FIELD_W, line, SCALE_BIG, COL_CYAN, COL_BG);
  }

  /* ---- setpoints ---- */
  {
    for (uint32_t field = 0u; field < 2u; field++) {
      char value[12], line[24];
      uint32_t i = 0u;
      uint16_t x = (field == 0u) ? COL_VOLT_X : COL_AMP_X;
      uint16_t val = (field == 0u) ? c->v_set_mv : c->i_limit_ma;
      uint16_t col = (s_field == (edit_field_t)field) ? COL_FG : COL_DIM;
      fmt_fixed(((uint32_t)val + 5u) / 10u, 2u, value, sizeof(value));
      line[i++] = (s_field == (edit_field_t)field) ? '>' : ' ';
      for (const char *p = value; *p != '\0' && i < sizeof(line) - 4u; p++) { line[i++] = *p; }
      line[i++] = (field == 0u) ? 'V' : 'A';
      if (s_setpoint_pending || s_waiting_ack) { line[i++] = '*'; }
      line[i] = '\0';
      if (s_setpoint_dirty || row_changed(s_prev_set[field], sizeof(s_prev_set[field]), line)) {
        (void)strncpy(s_prev_set[field], line, sizeof(s_prev_set[field]) - 1u);
        s_prev_set[field][sizeof(s_prev_set[field]) - 1u] = '\0';
        draw_field(x, ROW_SET_Y, FIELD_W, (field == 0u) ? "SET VOLTAGE" : "SET CURRENT",
                   SCALE_LABEL, col, COL_BG);
        draw_field(x, ROW_SET_VALUE_Y, FIELD_W, line, SCALE_BIG, col, COL_BG);
      }
    }
    s_setpoint_dirty = 0u;
  }

  /* ---- status: OUT / CV-CC / fault ---- */
  {
    char line[24];
    uint16_t col;
    uint32_t i = 0u;

    if (!available) {
      const char *t = "NO DEVICE";
      for (const char *p = t; *p != '\0'; p++) { line[i++] = *p; }
      col = COL_ERR;
    } else if (c->fault_latched) {
      line[i++] = '!'; line[i++] = ' ';
      const char *t = ui_fault_text(c->fault_bits);
      for (const char *p = t; (*p != '\0') && (i < sizeof(line) - 1u); p++) { line[i++] = *p; }
      col = COL_ERR;
    } else if (c->run_state == CANP_STATE_STARTING || c->run_state == CANP_STATE_STOPPING) {
      const char *t = (c->run_state == CANP_STATE_STARTING) ? "STARTING" : "STOPPING";
      for (const char *p = t; *p != '\0'; p++) { line[i++] = *p; }
      col = COL_WARN;
    } else if (!c->output_on && c->startup_failed) {
      const char *t = "INIT ERROR";
      for (const char *p = t; *p != '\0'; p++) { line[i++] = *p; }
      col = COL_ERR;
    } else if (!c->output_on && c->ready_known && !c->ready) {
      const char *t = "PREPARING";
      for (const char *p = t; *p != '\0'; p++) { line[i++] = *p; }
      col = COL_WARN;
    } else {
      const char *t = c->output_on ? "OUT ON " : "OUT OFF";
      for (const char *p = t; *p != '\0'; p++) { line[i++] = *p; }
      if (c->output_on) {
        line[i++] = ' ';
        const char *m2 = c->cv_mode ? "CV" : "CC";
        for (const char *p = m2; *p != '\0'; p++) { line[i++] = *p; }
      }
      col = c->output_on ? COL_OK : COL_DIM;
    }
    line[i] = '\0';

    if (row_changed(s_prev_status, sizeof(s_prev_status), line) ||
        (col != s_prev_status_col)) {
      s_prev_status_col = col;
      draw_row(ROW_STATUS_Y, line, SCALE_SMALL, col, COL_BG);
    }
  }

  /* ---- touch buttons: label follows the output state ---- */
  {
    if ((uint8_t)c->output_on != s_prev_out || startable != s_prev_available) {
      s_prev_out = (uint8_t)c->output_on;
      s_prev_available = startable;
      draw_button(BTN_X0, c->output_on ? "STOP" : "START",
                  c->output_on ? COL_ERR : (startable ? COL_OK : COL_DIM), COL_BG);
    }
  }

  /* ---- hints (static) ---- */
  {
    static const char *const v_step[] = {"0.01V", "0.10V", "1.00V", "10.00V"};
    static const char *const i_step[] = {"0.01A", "0.10A", "1.00A"};
    char step[48];
    const char *label = (s_field == FIELD_VOLTAGE) ? v_step[s_digit] : i_step[s_digit];
    step[0] = (s_field == FIELD_VOLTAGE) ? 'V' : 'I';
    step[1] = ' ';
    step[2] = '\0';
    (void)strncat(step, label, sizeof(step) - strlen(step) - 1u);
    (void)strncat(step, " K2=V/A K1=SET TOP=TEMP", sizeof(step) - strlen(step) - 1u);
    if (row_changed(s_prev_step, sizeof(s_prev_step), step)) {
      draw_row(ROW_HINT_Y, step, SCALE_SMALL, COL_DIM, COL_BG);
    }
  }

  /* Inputs handled last so the frame reflects the newest state. */
  handle_touch();
  if (s_touch_health != s_prev_touch_health) {
    s_prev_touch_health = s_touch_health;
    draw_field(208u, ROW_HEAD_Y, 104u,
               (s_touch_health == 1u) ? "TOUCH ERR" : ((s_touch_health == 2u) ? "TOUCH MAP" : ""),
               1u, COL_WARN, COL_BG);
  }
  /* Always paint the overlay AFTER changed rows/buttons. */
  draw_touch_marks();
}
