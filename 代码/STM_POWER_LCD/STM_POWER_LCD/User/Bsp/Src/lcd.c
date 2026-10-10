/**
  ******************************************************************************
  * @file    lcd.c
  * @brief   ST7789 SPI panel driver (blocking transfers, bring-up version).
  *
  *          Deliberately blocking/simple for first light. Once colours and
  *          orientation are confirmed, switch lcd_flush_rgb565() to SPI DMA:
  *          the buffer must then live in DMA-reachable SRAM and D-Cache
  *          coherency has to be handled (see ST AN4839).
  ******************************************************************************
  */
#include "lcd.h"
#include "spi.h"

/* --------------------------------------------------------------------------
 * ST7789 commands
 * -------------------------------------------------------------------------- */
#define ST7789_NOP        0x00u
#define ST7789_SWRESET    0x01u
#define ST7789_SLPOUT     0x11u
#define ST7789_NORON      0x13u
#define ST7789_INVON      0x21u
#define ST7789_DISPON     0x29u
#define ST7789_CASET      0x2Au
#define ST7789_RASET      0x2Bu
#define ST7789_RAMWR      0x2Cu
#define ST7789_MADCTL     0x36u
#define ST7789_COLMOD     0x3Au
#define ST7789_PORCTRL    0xB2u
#define ST7789_GCTRL      0xB7u
#define ST7789_VCOMS      0xBBu
#define ST7789_LCMCTRL    0xC0u
#define ST7789_VDVVRHEN   0xC2u
#define ST7789_VRHS       0xC3u
#define ST7789_VDVS       0xC4u
#define ST7789_FRCTRL2    0xC6u
#define ST7789_PWCTRL1    0xD0u
#define ST7789_PVGAMCTRL  0xE0u
#define ST7789_NVGAMCTRL  0xE1u

/* MADCTL bits */
#define MADCTL_MY         0x80u
#define MADCTL_MX         0x40u
#define MADCTL_MV         0x20u
#define MADCTL_RGB        0x00u   /* RGB order */
#define MADCTL_BGR        0x08u

/* --------------------------------------------------------------------------
 * Low level
 * -------------------------------------------------------------------------- */
HAL_StatusTypeDef lcd_write_cmd(uint8_t cmd)
{
  LCD_DC_CMD();
  LCD_CS_LOW();
  HAL_StatusTypeDef st = HAL_SPI_Transmit(&hspi1, &cmd, 1u, BSP_SPI_TIMEOUT_MS);
  LCD_CS_HIGH();
  return st;
}

HAL_StatusTypeDef lcd_write_data(const uint8_t *data, uint32_t len)
{
  if ((data == NULL) || (len == 0u)) {
    return HAL_ERROR;
  }
  LCD_DC_DATA();
  LCD_CS_LOW();
  HAL_StatusTypeDef st = HAL_SPI_Transmit(&hspi1, data, (uint16_t)len, BSP_SPI_TIMEOUT_MS);
  LCD_CS_HIGH();
  return st;
}

static HAL_StatusTypeDef lcd_write_data8(uint8_t value)
{
  return lcd_write_data(&value, 1u);
}

/* --------------------------------------------------------------------------
 * Window / pixels
 * -------------------------------------------------------------------------- */
HAL_StatusTypeDef lcd_set_window(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
  if ((w == 0u) || (h == 0u)) {
    return HAL_ERROR;
  }
  if (((uint32_t)x + w > LCD_W) || ((uint32_t)y + h > LCD_H)) {
    return HAL_ERROR;
  }

  uint16_t xs = x;
  uint16_t xe = (uint16_t)(x + w - 1u);
  uint16_t ys = y;
  uint16_t ye = (uint16_t)(y + h - 1u);

  uint8_t caset[4] = { (uint8_t)(xs >> 8), (uint8_t)(xs & 0xFFu),
                       (uint8_t)(xe >> 8), (uint8_t)(xe & 0xFFu) };
  uint8_t raset[4] = { (uint8_t)(ys >> 8), (uint8_t)(ys & 0xFFu),
                       (uint8_t)(ye >> 8), (uint8_t)(ye & 0xFFu) };

  if (lcd_write_cmd(ST7789_CASET) != HAL_OK) { return HAL_ERROR; }
  if (lcd_write_data(caset, 4u)     != HAL_OK) { return HAL_ERROR; }
  if (lcd_write_cmd(ST7789_RASET) != HAL_OK) { return HAL_ERROR; }
  if (lcd_write_data(raset, 4u)     != HAL_OK) { return HAL_ERROR; }
  return lcd_write_cmd(ST7789_RAMWR);
}

/* Stream pixels in chunks. A byte-swapped chunk is used so that the whole
 * transfer down to the panel is one CS-low burst. 256 pixels = 512 bytes. */
#define PIX_CHUNK 256u

static HAL_StatusTypeDef lcd_send_pixels(const uint16_t *pixels, uint32_t count)
{
  static uint8_t buf[PIX_CHUNK * 2u];   /* static: 16 KB stack task would blow */
  uint32_t left = count;

  while (left > 0u) {
    uint32_t n = (left > PIX_CHUNK) ? PIX_CHUNK : left;
    for (uint32_t i = 0u; i < n; i++) {
      uint16_t p = pixels[i];
      buf[i * 2u]      = (uint8_t)(p >> 8);        /* panel wants MSB first */
      buf[i * 2u + 1u] = (uint8_t)(p & 0xFFu);
    }
    if (lcd_write_data(buf, n * 2u) != HAL_OK) {
      return HAL_ERROR;
    }
    pixels += n;
    left   -= n;
  }
  return HAL_OK;
}

HAL_StatusTypeDef lcd_flush_rgb565(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                                   const uint16_t *pixels)
{
  if ((pixels == NULL) || (w == 0u) || (h == 0u)) {
    return HAL_ERROR;
  }
  if (lcd_set_window(x, y, w, h) != HAL_OK) {
    return HAL_ERROR;
  }
  return lcd_send_pixels(pixels, (uint32_t)w * (uint32_t)h);
}

/* --------------------------------------------------------------------------
 * Fills
 * -------------------------------------------------------------------------- */
HAL_StatusTypeDef lcd_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t colour)
{
  if (lcd_set_window(x, y, w, h) != HAL_OK) {
    return HAL_ERROR;
  }
  uint32_t left = (uint32_t)w * (uint32_t)h;
  uint16_t chunk[PIX_CHUNK];
  for (uint32_t i = 0u; i < PIX_CHUNK; i++) {
    chunk[i] = colour;
  }
  while (left > 0u) {
    uint32_t n = (left > PIX_CHUNK) ? PIX_CHUNK : left;
    if (lcd_send_pixels(chunk, n) != HAL_OK) {
      return HAL_ERROR;
    }
    left -= n;
  }
  return HAL_OK;
}

HAL_StatusTypeDef lcd_fill(uint16_t colour)
{
  return lcd_fill_rect(0u, 0u, LCD_W, LCD_H, colour);
}

/* --------------------------------------------------------------------------
 * Initialisation
 * -------------------------------------------------------------------------- */
static HAL_StatusTypeDef st7789_init_regs(void)
{
  /* Module-specific values from LCD_Demo_taobao/HARDWARE/LCD/LCD.C.
   * Do not substitute a generic ST7789 panel's voltage/gamma settings. */
  static const uint8_t porctrl[] = { 0x0Cu, 0x0Cu, 0x00u, 0x33u, 0x33u };
  static const uint8_t gctrl[]   = { 0x35u };
  static const uint8_t vcoms[]   = { 0x28u };
  static const uint8_t lcmctrl[] = { 0x2Cu };
  static const uint8_t vdvrvhen[]= { 0x01u };
  static const uint8_t vrhs[]    = { 0x0Bu };
  static const uint8_t vdvs[]    = { 0x20u };
  static const uint8_t frctrl2[] = { 0x0Fu };
  static const uint8_t pwctrl1[] = { 0xA4u, 0xA1u };
  static const uint8_t pgamma[]  = { 0xD0u, 0x01u, 0x08u, 0x0Fu, 0x11u, 0x2Au,
                                     0x36u, 0x55u, 0x44u, 0x3Au, 0x0Bu, 0x06u,
                                     0x11u, 0x20u };
  static const uint8_t ngamma[]  = { 0xD0u, 0x02u, 0x07u, 0x0Au, 0x0Bu, 0x18u,
                                     0x34u, 0x43u, 0x4Au, 0x2Bu, 0x1Bu, 0x1Cu,
                                     0x22u, 0x1Fu };

  if (lcd_write_cmd(ST7789_SLPOUT) != HAL_OK) { return HAL_ERROR; }
  HAL_Delay(120u);

  if (lcd_write_cmd(ST7789_COLMOD) != HAL_OK) { return HAL_ERROR; }
  if (lcd_write_data8(0x05u) != HAL_OK) { return HAL_ERROR; }   /* demo: RGB565 */

  if (lcd_write_cmd(ST7789_MADCTL) != HAL_OK) { return HAL_ERROR; }
#if (LCD_ROTATION == 1)
  if (lcd_write_data8(MADCTL_MX | MADCTL_MV | MADCTL_RGB) != HAL_OK) { return HAL_ERROR; }
#else
  if (lcd_write_data8(MADCTL_RGB) != HAL_OK) { return HAL_ERROR; }
#endif

  if (lcd_write_cmd(ST7789_PORCTRL) != HAL_OK) { return HAL_ERROR; }
  if (lcd_write_data(porctrl, sizeof(porctrl)) != HAL_OK) { return HAL_ERROR; }
  if (lcd_write_cmd(ST7789_GCTRL)   != HAL_OK) { return HAL_ERROR; }
  if (lcd_write_data(gctrl, sizeof(gctrl)) != HAL_OK) { return HAL_ERROR; }
  if (lcd_write_cmd(ST7789_VCOMS)   != HAL_OK) { return HAL_ERROR; }
  if (lcd_write_data(vcoms, sizeof(vcoms)) != HAL_OK) { return HAL_ERROR; }
  if (lcd_write_cmd(ST7789_LCMCTRL) != HAL_OK) { return HAL_ERROR; }
  if (lcd_write_data(lcmctrl, sizeof(lcmctrl)) != HAL_OK) { return HAL_ERROR; }
  if (lcd_write_cmd(ST7789_VDVVRHEN) != HAL_OK) { return HAL_ERROR; }
  if (lcd_write_data(vdvrvhen, sizeof(vdvrvhen)) != HAL_OK) { return HAL_ERROR; }
  if (lcd_write_cmd(ST7789_VRHS)    != HAL_OK) { return HAL_ERROR; }
  if (lcd_write_data(vrhs, sizeof(vrhs)) != HAL_OK) { return HAL_ERROR; }
  if (lcd_write_cmd(ST7789_VDVS)    != HAL_OK) { return HAL_ERROR; }
  if (lcd_write_data(vdvs, sizeof(vdvs)) != HAL_OK) { return HAL_ERROR; }
  if (lcd_write_cmd(ST7789_FRCTRL2) != HAL_OK) { return HAL_ERROR; }
  if (lcd_write_data(frctrl2, sizeof(frctrl2)) != HAL_OK) { return HAL_ERROR; }
  if (lcd_write_cmd(ST7789_PWCTRL1) != HAL_OK) { return HAL_ERROR; }
  if (lcd_write_data(pwctrl1, sizeof(pwctrl1)) != HAL_OK) { return HAL_ERROR; }

  if (lcd_write_cmd(ST7789_PVGAMCTRL) != HAL_OK) { return HAL_ERROR; }
  if (lcd_write_data(pgamma, sizeof(pgamma)) != HAL_OK) { return HAL_ERROR; }
  if (lcd_write_cmd(ST7789_NVGAMCTRL) != HAL_OK) { return HAL_ERROR; }
  if (lcd_write_data(ngamma, sizeof(ngamma)) != HAL_OK) { return HAL_ERROR; }

  if (lcd_write_cmd(ST7789_NORON)  != HAL_OK) { return HAL_ERROR; }
  HAL_Delay(10u);
  if (lcd_write_cmd(ST7789_DISPON) != HAL_OK) { return HAL_ERROR; }
  HAL_Delay(10u);
  return HAL_OK;
}

HAL_StatusTypeDef lcd_init(void)
{
  /* Chip select idle high before anything else. */
  LCD_CS_HIGH();
  LCD_DC_DATA();

  LCD_RST_HIGH();
  HAL_Delay(10u);
  LCD_RST_LOW();
  HAL_Delay(20u);
  LCD_RST_HIGH();
  HAL_Delay(120u);

  if (lcd_write_cmd(ST7789_SWRESET) != HAL_OK) { return HAL_ERROR; }
  HAL_Delay(150u);

  return st7789_init_regs();
}

/* --------------------------------------------------------------------------
 * Bring-up self test
 * -------------------------------------------------------------------------- */
HAL_StatusTypeDef lcd_draw_corner_markers(void)
{
  const uint16_t m = 20u;              /* marker size in pixels */
  const uint16_t colours[4] = { LCD_RED, LCD_GREEN, LCD_BLUE, LCD_YELLOW };

  /* TL red, TR green, BL blue, BR yellow - any swap tells you the mapping */
  if (lcd_fill_rect(0u, 0u, m, m, colours[0]) != HAL_OK) { return HAL_ERROR; }
  if (lcd_fill_rect((uint16_t)(LCD_W - m), 0u, m, m, colours[1]) != HAL_OK) { return HAL_ERROR; }
  if (lcd_fill_rect(0u, (uint16_t)(LCD_H - m), m, m, colours[2]) != HAL_OK) { return HAL_ERROR; }
  if (lcd_fill_rect((uint16_t)(LCD_W - m), (uint16_t)(LCD_H - m), m, m, colours[3]) != HAL_OK) {
    return HAL_ERROR;
  }
  return HAL_OK;
}

HAL_StatusTypeDef lcd_self_test(void)
{
  /* Horizontal bars in a known order: R, G, B, W, K, then corner markers.
   * All eight colours exercised, so both a swapped colour order and a
   * mirrored/flipped axis are immediately visible. */
  const uint16_t bars[5] = { LCD_RED, LCD_GREEN, LCD_BLUE, LCD_WHITE, LCD_BLACK };
  uint16_t band = (uint16_t)(LCD_H / 5u);

  for (uint16_t i = 0u; i < 5u; i++) {
    uint16_t y = (uint16_t)(i * band);
    uint16_t h = (i == 4u) ? (uint16_t)(LCD_H - y) : band;
    if (lcd_fill_rect(0u, y, LCD_W, h, bars[i]) != HAL_OK) {
      return HAL_ERROR;
    }
  }
  return lcd_draw_corner_markers();
}
