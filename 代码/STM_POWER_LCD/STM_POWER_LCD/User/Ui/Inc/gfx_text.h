/**
  ******************************************************************************
  * @file    gfx_text.h
  * @brief   Readable 8x16 ASCII bitmap font from the supplied panel demo.
  *          Printable ASCII 0x20..0x7E; unsupported bytes render '?'.
  *          Scale 1 = 8x16, scale 2 = 16x32. No dynamic allocation.
  ******************************************************************************
  */
#ifndef GFX_TEXT_H
#define GFX_TEXT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Native bitmap cell geometry. */
#define GFX_GLYPH_W   8u
#define GFX_GLYPH_H   16u

/** @brief  Pixel width of n characters at the given scale. */
uint16_t gfx_text_width(uint32_t n, uint8_t scale);

/** @brief  Pixel height of one line at the given scale. */
uint16_t gfx_text_height(uint8_t scale);

/**
  * @brief  Draw text at (x,y) with an RGB565 foreground on an RGB565 background.
  * @param  scale  Integer bitmap enlargement (1..8).
  * @note   Only the changed region is written; the caller decides when.
  */
int gfx_text_draw(uint16_t x, uint16_t y, const char *s, uint8_t scale,
                  uint16_t fg, uint16_t bg);

/* Sample one relative pixel using exactly the same bitmap as the renderer.
 * Returns 1 inside the text rectangle, 0 outside/invalid; no LCD transaction. */
int gfx_text_pixel(uint16_t x, uint16_t y, const char *s, uint8_t scale,
                   uint16_t fg, uint16_t bg, uint16_t *colour);

/** @brief  Character cell advance; the bitmap already contains side bearings. */
uint16_t gfx_char_advance(uint8_t scale);

#ifdef __cplusplus
}
#endif

#endif /* GFX_TEXT_H */
