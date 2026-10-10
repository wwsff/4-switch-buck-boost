/**
  ******************************************************************************
  * @file    bsp_log.c
  * @brief   Maintenance log over USART2, blocking transmit.
  ******************************************************************************
  */
#include "bsp_log.h"
#include "usart.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "semphr.h"

#define LOG_LINE_MAX 128u

static char     s_line[LOG_LINE_MAX];
static SemaphoreHandle_t s_log_mutex;

void log_init(void)
{
  if (s_log_mutex == NULL) { s_log_mutex = xSemaphoreCreateMutex(); }
}

uint32_t log_ticks(void)
{
  return HAL_GetTick(); /* same millisecond clock as touch mark expiry */
}

/* Retained for the housekeeping caller; no task-iteration timestamp drift. */
void log_tick_1ms(void)
{
}

static const char *level_str(log_level_t level)
{
  switch (level)
  {
    case LOG_WARN:  return "WARN ";
    case LOG_ERROR: return "ERROR";
    case LOG_INFO:
    default:        return "INFO ";
  }
}

static void log_emit(const char *s, uint32_t len)
{
  if ((s == NULL) || (len == 0u)) {
    return;
  }
  /* Blocking transmit: log volume is low and ordering matters more than
   * throughput here. Switch to DMA only if this ever shows up in a profile. */
  (void)HAL_UART_Transmit(&huart2, (const uint8_t *)s, (uint16_t)len, 100u);
}

void log_raw(const char *s)
{
  if (s != NULL && s_log_mutex != NULL &&
      xSemaphoreTake(s_log_mutex, pdMS_TO_TICKS(5u)) == pdTRUE) {
    log_emit(s, (uint32_t)strlen(s));
    (void)xSemaphoreGive(s_log_mutex);
  }
}

void log_printf(log_level_t level, const char *tag, const char *fmt, ...)
{
  if ((tag == NULL) || (fmt == NULL)) {
    return;
  }
  if (s_log_mutex == NULL ||
      xSemaphoreTake(s_log_mutex, pdMS_TO_TICKS(5u)) != pdTRUE) { return; }

  int n = snprintf(s_line, sizeof(s_line), "[%08lu][%s][%-4s] ",
                   (unsigned long)log_ticks(), level_str(level), tag);
  if ((n < 0) || ((size_t)n >= sizeof(s_line))) {
    (void)xSemaphoreGive(s_log_mutex);
    return;
  }

  va_list ap;
  va_start(ap, fmt);
  int m = vsnprintf(&s_line[n], sizeof(s_line) - (size_t)n, fmt, ap);
  va_end(ap);
  if (m < 0) {
    (void)xSemaphoreGive(s_log_mutex);
    return;
  }

  size_t used = (size_t)n + (size_t)m;
  if (used > sizeof(s_line) - 3u) {
    used = sizeof(s_line) - 3u;
  }
  s_line[used++] = '\r';
  s_line[used++] = '\n';
  s_line[used]   = '\0';

  log_emit(s_line, (uint32_t)used);
  (void)xSemaphoreGive(s_log_mutex);
}

void bsp_log_scan_hit(uint16_t hal_addr)
{
  log_printf(LOG_INFO, "I2C ", "device_ack addr=0x%02X (7bit 0x%02X)",
             (unsigned)hal_addr, (unsigned)(hal_addr >> 1));
}
