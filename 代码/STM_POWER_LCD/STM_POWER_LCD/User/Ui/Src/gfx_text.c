/* Readable 8x16 bitmap text using the supplied manufacturer's ASCII font. */
#include "gfx_text.h"
#include "font_8x16.h"
#include "lcd.h"
#include <string.h>

/* One scaled bitmap row. Maximum scale 8: 64x8 RGB565 pixels. */
static uint16_t s_strip[GFX_GLYPH_W * 8u * 8u];

uint16_t gfx_char_advance(uint8_t scale)
{
  return (uint16_t)(GFX_GLYPH_W * scale);
}
uint16_t gfx_text_width(uint32_t n, uint8_t scale)
{
  uint32_t width = n * gfx_char_advance(scale);
  return (uint16_t)((width > 65535u) ? 65535u : width);
}
uint16_t gfx_text_height(uint8_t scale)
{
  return (uint16_t)(GFX_GLYPH_H * scale);
}
int gfx_text_pixel(uint16_t x, uint16_t y, const char *s, uint8_t scale,
                   uint16_t fg, uint16_t bg, uint16_t *colour)
{
  if (s == 0 || colour == 0 || scale == 0u || scale > 8u ||
      y >= gfx_text_height(scale)) { return 0; }
  uint32_t cell = x / gfx_char_advance(scale);
  if (cell >= strlen(s)) { return 0; }
  unsigned char c = (unsigned char)s[cell];
  if (c < 0x20u || c > 0x7Eu) { c = '?'; }
  uint16_t col = (uint16_t)((x / scale) % GFX_GLYPH_W);
  uint16_t row = (uint16_t)(y / scale);
  uint8_t bits = s_font_8x16[c - 0x20u][col * 2u + row / 8u];
  *colour = (bits & (0x80u >> (row % 8u))) ? fg : bg;
  return 1;
}
static int draw_glyph(uint16_t x, uint16_t y, unsigned char c, uint8_t scale,
                      uint16_t fg, uint16_t bg)
{
  if (x >= LCD_W || y >= LCD_H) { return 0; }
  if (c < 0x20u || c > 0x7Eu) { c = '?'; }
  const uint8_t *glyph = s_font_8x16[c - 0x20u];
  uint16_t pixel_w = gfx_char_advance(scale);
  if ((uint32_t)x + pixel_w > LCD_W) { pixel_w = (uint16_t)(LCD_W - x); }
  for (uint16_t r = 0u; r < GFX_GLYPH_H; r++) {
    uint16_t py = (uint16_t)(y + r * scale);
    uint16_t pixel_h = scale;
    if (py >= LCD_H) { break; }
    if ((uint32_t)py + pixel_h > LCD_H) { pixel_h = (uint16_t)(LCD_H - py); }
    for (uint16_t yy = 0u; yy < pixel_h; yy++) {
      for (uint16_t xx = 0u; xx < pixel_w; xx++) {
        uint16_t col = (uint16_t)(xx / scale);
        uint8_t bits = glyph[col * 2u + r / 8u];
        s_strip[yy * pixel_w + xx] = (bits & (0x80u >> (r % 8u))) ? fg : bg;
      }
    }
    if (lcd_flush_rgb565(x, py, pixel_w, pixel_h, s_strip) != HAL_OK) { return -1; }
  }
  return 0;
}
int gfx_text_draw(uint16_t x, uint16_t y, const char *s, uint8_t scale,
                  uint16_t fg, uint16_t bg)
{
  if (s == 0 || scale == 0u || scale > 8u) { return -1; }
  uint32_t cx = x;
  for (const unsigned char *p = (const unsigned char *)s; *p != 0u && cx < LCD_W; p++) {
    if (draw_glyph((uint16_t)cx, y, *p, scale, fg, bg) != 0) { return -1; }
    cx += gfx_char_advance(scale);
  }
  return 0;
}
