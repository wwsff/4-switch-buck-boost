/**
  ******************************************************************************
  * @file    app_bringup.c
  * @brief   Per-peripheral acceptance test, step by step, with logging.
  ******************************************************************************
  */
#include "app_bringup.h"

#include "bsp_config.h"
#include "bsp_log.h"
#include "lcd.h"
#include "touch.h"
#include "app_input.h"

#include "tim.h"
#include "fdcan.h"
#include "i2c.h"
#include "usart.h"

/* --------------------------------------------------------------------------
 * Encoder
 *
 * TIM3 runs in encoder mode TI12 (x4 decoding) with ARR = 65535, so the
 * hardware counter is 16-bit. We keep a 32-bit position by tracking wraps.
 * -------------------------------------------------------------------------- */
static int32_t  s_enc_position;
static uint16_t s_enc_last;
static int16_t  s_enc_delta_1ms;
static int32_t  s_enc_accum;

static void encoder_poll(void)
{
  uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
  int16_t  diff = (int16_t)(now - s_enc_last);   /* wraps correctly */
  s_enc_last = now;
  if (diff == 0) {
    return;
  }
  s_enc_delta_1ms = diff;
  s_enc_position += diff;

  /* Publish one event per detent by accumulating raw counts. A typical EC11
   * gives 4 counts per detent in TI12 (x4) mode; adjust DETENT_COUNTS if the
   * knob is a different type. */
  s_enc_accum += diff;
  #define DETENT_COUNTS 4
  while (s_enc_accum >= DETENT_COUNTS) {
    s_enc_accum -= DETENT_COUNTS;
    input_publish(IN_EVT_ENCODER, IN_ACT_PRESSED, +1);
  }
  while (s_enc_accum <= -DETENT_COUNTS) {
    s_enc_accum += DETENT_COUNTS;
    input_publish(IN_EVT_ENCODER, IN_ACT_PRESSED, -1);
  }
}

int16_t bringup_encoder(void)
{
  return (int16_t)s_enc_position;
}

/** @brief  Counts seen in the last 1 ms - use it to verify direction. */
int16_t bringup_encoder_delta(void)
{
  return s_enc_delta_1ms;
}

/* --------------------------------------------------------------------------
 * Keys, with simple debounce
 * -------------------------------------------------------------------------- */
#define DEBOUNCE_MS 20u

typedef struct
{
  uint8_t  stable;        /* last debounced state, 1 = pressed */
  uint8_t  candidate;     /* debounce integrator */
  uint8_t  candidate_raw; /* raw level sampled this tick */
  uint16_t count;
} debounce_t;

static debounce_t s_key1, s_key2, s_encsw;

/* hold-time tracking, for long press */
#define LONG_PRESS_MS 700u
static uint16_t s_key1_hold, s_key2_hold, s_encsw_hold;

static uint8_t debounce(debounce_t *d, uint8_t raw_pressed)
{
  (void)raw_pressed;   /* the raw level is already in d->candidate_raw */
  uint8_t raw = d->candidate_raw;
  if (raw != d->candidate) {
    d->candidate = raw;
    d->count = 0u;
    return 0u;
  }
  if (d->count < DEBOUNCE_MS) {
    d->count++;
  }
  if ((d->count >= DEBOUNCE_MS) && (d->stable != raw)) {
    d->stable = raw;
    return 1u;            /* state changed */
  }
  return 0u;
}

/* One key: debounce, then publish press / release / long-press-once. */
static void key_poll(debounce_t *d, uint16_t *hold, input_event_t evt, const char *name)
{
  if (debounce(d, d->candidate_raw) != 0u) {
    if (d->stable != 0u) {
      *hold = 0u;
      input_publish(evt, IN_ACT_PRESSED, 0);
      LOGI("KEY ", "%s down", name);
    } else {
      input_publish(evt, IN_ACT_RELEASED, 0);
      LOGI("KEY ", "%s up", name);
    }
    return;
  }

  /* held? */
  if (d->stable != 0u) {
    if (*hold < LONG_PRESS_MS) {
      (*hold)++;
      if (*hold == LONG_PRESS_MS) {
        input_publish(evt, IN_ACT_LONG_PRESS, 0);
        LOGI("KEY ", "%s long press", name);
      }
    }
  }
}

static void keys_poll(void)
{
  s_key1.candidate_raw  = KEY1_PRESSED()   ? 1u : 0u;
  s_key2.candidate_raw  = KEY2_PRESSED()   ? 1u : 0u;
  s_encsw.candidate_raw = ENC_SW_PRESSED() ? 1u : 0u;

  key_poll(&s_key1,  &s_key1_hold,  IN_EVT_KEY1,   "KEY1");
  key_poll(&s_key2,  &s_key2_hold,  IN_EVT_KEY2,   "KEY2");
  key_poll(&s_encsw, &s_encsw_hold, IN_EVT_ENC_SW, "ENCSW");
}

/* --------------------------------------------------------------------------
 * Touch
 * -------------------------------------------------------------------------- */
/* --------------------------------------------------------------------------
 * Steps
 * -------------------------------------------------------------------------- */
static void step_lcd(void)
{
  if (lcd_init() != HAL_OK) {
    LOGE("LCD ", "init FAILED (spi/gpio)");
    return;
  }
  LOGI("LCD ", "init ok %dx%d", LCD_W, LCD_H);

  if (lcd_self_test() != HAL_OK) {
    LOGE("LCD ", "self test FAILED");
    return;
  }
  LOGI("LCD ", "self_test: bars R,G,B,W,K + markers TL=R TR=G BL=B BR=Y");
}

static void step_touch(void)
{
  uint8_t id = 0u;
  if (touch_init() != HAL_OK) {
    LOGE("TOUCH", "init FAILED - scanning bus");
    touch_scan_bus();
    return;
  }
  if (touch_read_id(&id) != HAL_OK) {
    LOGE("TOUCH", "id read FAILED");
    return;
  }
  LOGI("TOUCH", "init ok id=0x%02X", (unsigned)id);

  if (touch_irq_asserted()) {
    LOGI("TOUCH", "INT already asserted (finger on panel?)");
  }
}

static void step_encoder(void)
{
  /* CubeMX initialises TIM3 but does not start its encoder counter. */
  if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK) {
    LOGE("ENC ", "encoder start FAILED");
    return;
  }
  s_enc_last = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
  LOGI("ENC ", "encoder started, counter=%u", (unsigned)s_enc_last);
}

void bringup_run(void)
{
  log_raw("\r\n=== STM_POWER_LCD bring-up ===\r\n");
  LOGI("BOOT", "firmware=%s", HMI_FW_VERSION);
  log_raw("UART TEST 115200 8N1: 0123456789 ABCDEF UUUUUUUU\r\n");
  LOGI("UART", "pclk1=%lu brr=%lu baud=%lu 8N1",
       (unsigned long)HAL_RCC_GetPCLK1Freq(), (unsigned long)huart2.Instance->BRR,
       (unsigned long)huart2.Init.BaudRate);
  LOGI("BOOT", "sysclk=%lu Hz", (unsigned long)HAL_RCC_GetSysClockFreq());
  LOGI("BOOT", "hclk=%lu Hz pclk1=%lu Hz pclk2=%lu Hz",
       (unsigned long)HAL_RCC_GetHCLKFreq(),
       (unsigned long)HAL_RCC_GetPCLK1Freq(),
       (unsigned long)HAL_RCC_GetPCLK2Freq());

  step_lcd();
  step_touch();
  step_encoder();

  LOGI("BOOT", "bring-up done");
}

void bringup_tick_1ms(void)
{
  log_tick_1ms();
  encoder_poll();
  keys_poll();
}
