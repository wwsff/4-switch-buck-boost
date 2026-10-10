/**
  ******************************************************************************
  * @file    lcd.h
  * @brief   ST7789 SPI panel driver interface.
  *
  *          Bus: SPI1 (PA5 SCK, PA7 MOSI, PA6 MISO), software chip select on
  *          PA4, data/command on PC1, reset on PC0.
  *
  *          Bring-up order (see the plan):
  *            lcd_init()  ->  lcd_fill() solid colours  ->  corner markers
  *          Confirm colour order and orientation BEFORE building any UI.
  ******************************************************************************
  */
#ifndef BSP_LCD_H
#define BSP_LCD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "bsp_config.h"

/* RGB565 helpers. The panel expects big-endian pixel bytes on the wire; this
 * is handled inside lcd_flush_rgb565(), callers just pass host-order u16. */
#define LCD_RGB565(r, g, b)  (uint16_t)((((uint16_t)(r) & 0xF8u) << 8) | \
                                        (((uint16_t)(g) & 0xFCu) << 3) | \
                                        (((uint16_t)(b) & 0xF8u) >> 3))

#define LCD_BLACK   0x0000u
#define LCD_WHITE   0xFFFFu
#define LCD_RED     0xF800u
#define LCD_GREEN   0x07E0u
#define LCD_BLUE    0x001Fu
#define LCD_YELLOW  0xFFE0u
#define LCD_CYAN    0x07FFu
#define LCD_MAGENTA 0xF81Fu
#define LCD_GREY    0x8410u

/**
  * @brief  Hardware reset + ST7789 register initialisation.
  * @retval HAL_OK on success.
  */
HAL_StatusTypeDef lcd_init(void);

/** @brief  Set the address window the following pixel data is written into. */
HAL_StatusTypeDef lcd_set_window(uint16_t x, uint16_t y, uint16_t w, uint16_t h);

/** @brief  Fill the whole panel with one RGB565 colour. */
HAL_StatusTypeDef lcd_fill(uint16_t colour);

/** @brief  Fill a rectangle with one RGB565 colour. */
HAL_StatusTypeDef lcd_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t colour);

/**
  * @brief  Stream w*h RGB565 pixels (host byte order) starting at (x,y).
  * @note   The window is set once and all pixels are sent in a single CS-low
  *         burst - do NOT toggle chip select per byte.
  */
HAL_StatusTypeDef lcd_flush_rgb565(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                                   const uint16_t *pixels);

/** @brief  Draw the four corner markers used to verify orientation. */
HAL_StatusTypeDef lcd_draw_corner_markers(void);

/** @brief  Run the colour/orientation self test (solid bars + markers). */
HAL_StatusTypeDef lcd_self_test(void);

/* Low level, exposed for diagnostics */
HAL_StatusTypeDef lcd_write_cmd(uint8_t cmd);
HAL_StatusTypeDef lcd_write_data(const uint8_t *data, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* BSP_LCD_H */
