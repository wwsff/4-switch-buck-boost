/**
  ******************************************************************************
  * @file    bsp_log.h
  * @brief   Maintenance log over USART2 (CH340K -> USB-C), 115200 8N1.
  *
  *          Format matches the agreed style:
  *            [00123456][INFO ][CAN ] link_up bitrate=500000 proto=1
  *            [00140217][ERROR][CH1 ] fault=0x00000004 vout=5230mV output=off
  *
  *          Normal operation must NOT log per frame - only state changes,
  *          commands, acknowledgements and faults.
  ******************************************************************************
  */
#ifndef BSP_LOG_H
#define BSP_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "bsp_config.h"

typedef enum
{
  LOG_INFO = 0,
  LOG_WARN,
  LOG_ERROR
} log_level_t;

/** @brief  Initialise the shared UART log mutex. */
void log_init(void);

/** @brief  Timestamp source: HAL millisecond tick (TIM6). */
uint32_t log_ticks(void);

/** @brief  Compatibility hook for housekeeping; timestamps use HAL_GetTick. */
void log_tick_1ms(void);

/** @brief  printf-style log with timestamp, level and tag. */
void log_printf(log_level_t level, const char *tag, const char *fmt, ...);

/** @brief  Raw string, no decoration (boot banners). */
void log_raw(const char *s);

/** @brief  Hook used by touch_scan_bus() for I2C scan hits. */
void bsp_log_scan_hit(uint16_t hal_addr);

#define LOGI(tag, ...)  log_printf(LOG_INFO,  tag, __VA_ARGS__)
#define LOGW(tag, ...)  log_printf(LOG_WARN,  tag, __VA_ARGS__)
#define LOGE(tag, ...)  log_printf(LOG_ERROR, tag, __VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif /* BSP_LOG_H */
