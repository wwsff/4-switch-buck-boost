/* Runs real UI, glyph renderer, touch parser and CAN protocol against a fake
 * LCD framebuffer / I2C module. Does not establish hardware correctness. */
#include "ui_main.h"
#include "ui_model.h"
#include "gfx_text.h"
#include "lcd.h"
#include "touch.h"
#include "i2c.h"
#include "app_input.h"
#include "can_if.h"
#include "can_proto.h"
#include "can_protocol.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

static uint32_t now;
static uint8_t regs[256];
static uint16_t fb[LCD_H][LCD_W], baseline[LCD_H][LCD_W];
static int checks, failures, bad_rect, bad_address, read_count;
static unsigned write_count;
static uint32_t lcd_pixels, full_fills;
static int bus_fail, read_fail, tx_commands;
static uint16_t responding_address = 0x70u;
static canif_frame_t rx;
static uint8_t rx_ready;
static input_handler_t handler;
static char logs[128][160];
static unsigned log_n;
I2C_HandleTypeDef hi2c1;
uint32_t HAL_I2C_GetError(I2C_HandleTypeDef *h)
{ (void)h; return read_fail ? 4u : 0u; }

static void check(const char *name, int pass)
{
  checks++;
  if (!pass) { failures++; printf("FAIL: %s\n", name); }
}
uint32_t HAL_GetTick(void) { return now; }
void HAL_Delay(uint32_t ms) { now += ms; }
void HAL_GPIO_WritePin(void *port, uint16_t pin, GPIO_PinState s)
{ (void)port; (void)pin; (void)s; }
GPIO_PinState HAL_GPIO_ReadPin(void *port, uint16_t pin)
{ (void)port; (void)pin; return GPIO_PIN_SET; }
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *h, uint16_t addr,
    uint16_t reg, uint16_t size, uint8_t *out, uint16_t len, uint32_t timeout)
{
  (void)h; (void)size; (void)timeout;
  read_count++;
  if (addr != responding_address) { bad_address++; return HAL_ERROR; }
  if (read_fail || reg + len > sizeof(regs)) { return HAL_ERROR; }
  memcpy(out, regs + reg, len);
  return HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *h, uint16_t addr,
    uint16_t reg, uint16_t size, uint8_t *data, uint16_t len, uint32_t timeout)
{
  (void)h; (void)size; (void)timeout;
  write_count++;
  if (addr != responding_address) { bad_address++; return HAL_ERROR; }
  if (reg + len > sizeof(regs)) { return HAL_ERROR; }
  memcpy(regs + reg, data, len);
  return HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *h, uint16_t addr,
    uint32_t trials, uint32_t timeout)
{ (void)h; (void)trials; (void)timeout; return addr == responding_address ? HAL_OK : HAL_ERROR; }
void bsp_log_scan_hit(uint16_t addr) { (void)addr; }
void host_log_probe(const char *level, const char *tag, const char *fmt, ...)
{
  va_list ap;
  (void)level; (void)tag;
  if (log_n >= 128u) { return; }
  va_start(ap, fmt);
  vsnprintf(logs[log_n++], sizeof(logs[0]), fmt, ap);
  va_end(ap);
}
static int has_log(const char *text)
{
  for (unsigned i = 0u; i < log_n; i++) {
    if (strstr(logs[i], text)) { return 1; }
  }
  return 0;
}
static int in_bounds(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
  if (!w || !h || (uint32_t)x + w > LCD_W || (uint32_t)y + h > LCD_H) {
    bad_rect++; return 0;
  }
  return 1;
}
HAL_StatusTypeDef lcd_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t col)
{
  if (!in_bounds(x, y, w, h)) { return HAL_ERROR; }
  lcd_pixels += (uint32_t)w * h;
  for (unsigned yy = y; yy < y + h; yy++) {
    for (unsigned xx = x; xx < x + w; xx++) { fb[yy][xx] = col; }
  }
  return HAL_OK;
}
HAL_StatusTypeDef lcd_fill(uint16_t col)
{ full_fills++; return lcd_fill_rect(0, 0, LCD_W, LCD_H, col); }
HAL_StatusTypeDef lcd_flush_rgb565(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                                  const uint16_t *pixels)
{
  if (!in_bounds(x, y, w, h)) { return HAL_ERROR; }
  lcd_pixels += (uint32_t)w * h;
  for (unsigned yy = 0; yy < h; yy++) {
    for (unsigned xx = 0; xx < w; xx++) { fb[y + yy][x + xx] = pixels[yy * w + xx]; }
  }
  return HAL_OK;
}
void input_set_handler(input_handler_t fn) { handler = fn; }
int canif_init(void) { return bus_fail ? -1 : 0; }
int canif_send(uint32_t id, const uint8_t *d, uint8_t len, uint32_t timeout)
{ (void)d; (void)len; (void)timeout; if (id == CANP_ID_SET) { tx_commands++; } return 0; }
int canif_receive(canif_frame_t *out, uint32_t timeout)
{ (void)timeout; if (!rx_ready) { return 0; } *out = rx; rx_ready = 0; return 1; }
static void status(uint8_t st, uint16_t v, uint16_t i)
{
  memset(&rx, 0, sizeof(rx));
  rx.id = CANP_ID_MEASURED; rx.dlc = 8;
  canp_put_u16(&rx.data[CANP_MS_V_MV], v);
  canp_put_u16(&rx.data[CANP_MS_I_MA], i);
  rx.data[CANP_MS_CH] = CANP_CH1;
  rx.data[CANP_MS_STATE] = st;
  rx_ready = 1;
  can_proto_poll(0);
  ui_main_update();
}
static void raw_finger(uint8_t count, uint16_t x, uint16_t y)
{
  regs[2] = count;
  regs[3] = (uint8_t)(0x80u | (x >> 8)); regs[4] = (uint8_t)x;
  regs[5] = (uint8_t)(y >> 8); regs[6] = (uint8_t)y;
}
/* Simulate a click at a known LANDSCAPE LCD position. The bus still reports
 * the native portrait position, independently of the UI mapping function. */
static void finger(uint8_t count, uint16_t x, uint16_t y)
{
  raw_finger(count, (uint16_t)(239u - y), x);
}
static void reset_ui(uint32_t tick)
{
  now = tick; log_n = 0; read_fail = 0; rx_ready = 0; tx_commands = 0;
  memset(regs, 0, sizeof(regs));
  ui_model_init(); can_proto_init(); ui_main_init(); ui_main_draw_all();
}
static int red_square(uint16_t x, uint16_t y)
{
  for (unsigned yy = y - 2u; yy <= y + 2u; yy++) {
    for (unsigned xx = x - 2u; xx <= x + 2u; xx++) {
      if (fb[yy][xx] != LCD_RED) { return 0; }
    }
  }
  return 1;
}
static void snapshot(const char *path)
{
  FILE *f = fopen(path, "wb");
  if (!f) { return; }
  fprintf(f, "P6\n%d %d\n255\n", LCD_W, LCD_H);
  for (unsigned y = 0; y < LCD_H; y++) {
    for (unsigned x = 0; x < LCD_W; x++) {
      uint16_t p = fb[y][x];
      uint8_t rgb[3] = {(uint8_t)(((p >> 11) & 31u) * 255u / 31u),
                        (uint8_t)(((p >> 5) & 63u) * 255u / 63u),
                        (uint8_t)((p & 31u) * 255u / 31u)};
      fwrite(rgb, 1, 3, f);
    }
  }
  fclose(f);
}
int main(void)
{
  uint8_t id = 0;
  touch_state_t ts;
  printf("H750 UI/touch framebuffer and protocol checks (host)\n");
  reset_ui(1000u);
  regs[0xA3] = 0x64u;
  check("module initialises at HAL address 0x70", touch_init() == HAL_OK);
  check("full byte chip ID retained", touch_read_id(&id) == HAL_OK && id == 0x64u);
  check("initialisation keeps module configuration defaults", write_count == 0u);
  regs[0xA3] = 0x12u;
  check("unexpected ID is diagnostic, not fatal", touch_init() == HAL_OK && has_log("unexpected chip_id=0x12"));
  raw_finger(2, 239, 319); regs[9] = 0x80u; regs[10] = 25; regs[11] = 0x01u; regs[12] = 44;
  check("both points decoded with status bits masked", touch_read(&ts) == HAL_OK &&
        ts.count == 2 && ts.x[0] == 239 && ts.y[0] == 319 && ts.x[1] == 25 && ts.y[1] == 300);
  finger(3, 0, 0);
  check("invalid point count rejected", touch_read(&ts) == HAL_ERROR && ts.count == 0);
  check("no double-shifted I2C address used", bad_address == 0);
  responding_address = 0x90u;
  check("documented alternate module address is detected", touch_init() == HAL_OK && touch_address() == 0x90u);
  responding_address = 0x70u;
  check("primary address is reprobed on reset", touch_init() == HAL_OK && touch_address() == 0x70u);
  read_fail = 1;
  check("failed read preserves HAL diagnostic", touch_read(&ts) == HAL_ERROR && touch_last_error() == 4u && touch_last_status() == HAL_ERROR);
  read_fail = 0;

  reset_ui(10000u);
  memcpy(baseline, fb, sizeof(fb));
  for (unsigned t = 0; t < CANP_LINK_LOST_MS; t += 20u) {
    now = 10000u + t; can_proto_poll(0); ui_main_update();
  }
  check("boot never invents an online device", !ui_model()->link_up && !ui_model()->ch1.installed);
  check("offline UI stays at zero", !ui_model()->ch1.v_out_mv && !ui_model()->ch1.i_out_ma);
  check("offline polling leaves the initial screen unchanged", memcmp(baseline, fb, sizeof(fb)) == 0);
  check("no capability request without telemetry", tx_commands == 0);

  reset_ui(20000u);
  finger(1, 100, 55); read_count = 0; ui_main_update();
  check("one status read and one XY read per pressed GUI update", read_count == 2);
  check("touch paints exactly 5x5 red pixels", red_square(100, 55) && fb[52][100] != LCD_RED);
  check("coordinates rotate and area is logged", has_log("raw=(184,100) lcd=(100,55) center=(100,55) area=VOLTAGE"));
  unsigned presses = log_n;
  now += 100u; finger(1, 150, 60); ui_main_update();
  check("held finger does not repeat the action or move the marker", log_n == presses && fb[60][150] != LCD_RED);
  finger(0, 0, 0); ui_main_update();
  check("release is logged", has_log("released"));
  now = 21000u; finger(1, 220, 60); ui_main_update();
  check("second press gets a separate mark", red_square(220, 60) && red_square(100, 55));
  check("right measurement selects current", has_log("area=CURRENT"));
  finger(0, 0, 0); ui_main_update();
  ui_main_draw_all();
  check("active marks survive a complete UI redraw", red_square(220, 60) && red_square(100, 55));
  now = 24999u; ui_main_update();
  check("first mark is present just before 5 seconds", red_square(100, 55));
  now = 25000u; ui_main_update();
  check("first mark expires at 5 seconds", fb[55][100] != LCD_RED);
  check("later mark keeps its own lifetime", red_square(220, 60));
  now = 26000u; ui_main_update();
  check("second mark also expires", fb[60][220] != LCD_RED);

  reset_ui(30000u);
  memcpy(baseline, fb, sizeof(fb));
  finger(1, 10, 44); ui_main_update(); finger(0, 0, 0); ui_main_update();
  uint32_t before_pixels = lcd_pixels, before_fills = full_fills;
  now += 5000u; ui_main_update();
  check("expiry restores all underlying text and background", memcmp(baseline, fb, sizeof(fb)) == 0);
  check("expiry writes only the covered 25 pixels", lcd_pixels - before_pixels == 25u);
  check("expiry never clears the full screen", full_fills == before_fills);
  snapshot("offline_ui.ppm");
  finger(1, 0, 0); ui_main_update();
  check("edge tap keeps full 5x5 marker in bounds", red_square(2, 2));
  check("edge log includes actual shifted centre", has_log("raw=(239,0) lcd=(0,0) center=(2,2)"));
  finger(0, 0, 0); ui_main_update(); finger(1, 319, 239); ui_main_update();
  check("bottom-right edge is handled", red_square(317, 237));
  check("bottom-right centre logged", has_log("center=(317,237)"));
  finger(0, 0, 0); ui_main_update(); raw_finger(1, 4095, 4095); ui_main_update();
  check("out-of-range touch ignored and logged", has_log("outside LCD; ignored"));
  snapshot("touch_ui.ppm");

  reset_ui(40000u); finger(1, 20, 196); ui_main_update();
  check("offline START touch sends nothing", tx_commands == 0 && has_log("area=START"));
  status(CANP_STATE_CV, 15000u, 750u);
  check("connected readings retained", ui_model()->link_up && ui_model()->ch1.v_out_mv == 15000u);
  now += CANP_LINK_LOST_MS + 1u; can_proto_poll(0); ui_main_update();
  check("disconnect clears measured values", !ui_model()->link_up && !ui_model()->ch1.v_out_mv && !ui_model()->ch1.i_out_ma);
  status(CANP_STATE_ABSENT, 12345u, 2000u);
  check("absent CH1 always displays zero", ui_model()->link_up && !ui_model()->ch1.installed && !ui_model()->ch1.v_out_mv && !ui_model()->ch1.i_out_ma);
  int sent = tx_commands; can_proto_request_set(5000u, 500u);
  check("absent channel blocks SET", tx_commands == sent);
  finger(0, 0, 0); ui_main_update(); finger(1, 20, 196); ui_main_update();
  check("absent channel blocks START", tx_commands == sent);

  bus_fail = 1; reset_ui(50000u); memcpy(baseline, fb, sizeof(fb));
  can_proto_poll(0); ui_main_update();
  check("CAN init failure still allows the UI", !ui_model()->link_up && memcmp(baseline, fb, sizeof(fb)) == 0);
  finger(1, 80, 80); ui_main_update();
  check("CAN failure does not disable touch", red_square(80, 80));
  bus_fail = 0;

  reset_ui(0xFFFFFF00u); finger(1, 90, 90); ui_main_update();
  finger(0, 0, 0); now += 4999u; ui_main_update();
  check("marker lifetime survives tick wrap", red_square(90, 90));
  now++; ui_main_update();
  check("marker expires across tick wrap", fb[90][90] != LCD_RED);
  check("no LCD request goes out of bounds", bad_rect == 0);
  check("encoder/key handler remains installed", handler != NULL);
  lcd_fill(LCD_BLACK);
  check("large bitmap paints replicated rows", gfx_text_draw(0, 0, "8", 4, LCD_WHITE, LCD_BLACK) == 0 &&
        fb[16][4] == LCD_WHITE && fb[19][7] == LCD_WHITE && fb[0][0] == LCD_BLACK);
  check("changing a digit clears its old pixels", gfx_text_draw(0, 0, "1", 4, LCD_WHITE, LCD_BLACK) == 0 &&
        fb[20][4] == LCD_BLACK && fb[16][12] == LCD_WHITE);
  check("maximum-scale glyph clips safely at screen edge",
        gfx_text_draw(LCD_W - 3u, LCD_H - 3u, "88", 8, LCD_WHITE, LCD_BLACK) == 0 && bad_rect == 0);
  check("landscape geometry is 320x240", LCD_W == 320 && LCD_H == 240);
  reset_ui(60000u);
  finger(1, 108, 196); ui_main_update();
  check("button gap does not execute an action", tx_commands == 0 && has_log("area=NONE"));
  finger(0, 0, 0); ui_main_update(); finger(1, 220, 130); ui_main_update();
  handler(IN_EVT_ENCODER, IN_ACT_PRESSED, 1);
  check("right setpoint selects current editing", ui_model()->ch1.i_limit_ma == 1010u && ui_model()->ch1.v_set_mv == 5000u);
  finger(0, 0, 0); ui_main_update(); finger(1, 40, 130); ui_main_update();
  handler(IN_EVT_ENCODER, IN_ACT_PRESSED, 1);
  check("left setpoint selects voltage editing", ui_model()->ch1.v_set_mv == 5010u);
  finger(0, 0, 0); ui_main_update();
  handler(IN_EVT_KEY2, IN_ACT_PRESSED, 0);
  handler(IN_EVT_ENCODER, IN_ACT_PRESSED, 1);
  check("K2 still selects V/A after physical pin swap", ui_model()->ch1.i_limit_ma == 1020u);
  int previous_tx = tx_commands;
  handler(IN_EVT_KEY1, IN_ACT_PRESSED, 0);
  check("K1 is confirmation and remains blocked offline", tx_commands == previous_tx);
  read_fail = 1; ui_main_update();
  check("touch failure log includes error details", has_log("read_failed a7=0x38 st=1 err=0x00000004"));
  read_fail = 0; ui_main_update();
  check("good empty read uses only the status register", touch_last_count() == 0u && touch_last_status() == HAL_OK);

  /* Compare partial restoration with an independent full render of the same
   * UI state across every kind of layer, including text and border pixels. */
  const uint16_t spots[][2] = {
    {16,14}, {160,24}, {160,60}, {20,40}, {190,40}, {25,68}, {205,68},
    {25,108}, {200,108}, {40,134}, {210,134}, {25,168}, {45,198},
    {153,198}, {255,198}, {40,230}, {0,0}, {319,239}
  };
  for (unsigned p = 0u; p < sizeof(spots) / sizeof(spots[0]); p++) {
    reset_ui(70000u + p * 6000u);
    finger(1, spots[p][0], spots[p][1]); ui_main_update();
    finger(0, 0, 0); ui_main_update();
    before_pixels = lcd_pixels; before_fills = full_fills;
    now += 5000u; ui_main_update();
    check("layer restore is limited to its 5x5 rectangle", lcd_pixels - before_pixels == 25u && full_fills == before_fills);
    memcpy(baseline, fb, sizeof(fb));
    ui_main_draw_all();
    check("partial restore matches fresh scene rendering", memcmp(baseline, fb, sizeof(fb)) == 0);
  }
  reset_ui(200000u);
  finger(1, 100, 55); ui_main_update(); finger(0, 0, 0); ui_main_update();
  now += 1000u;
  finger(1, 102, 55); ui_main_update(); finger(0, 0, 0); ui_main_update();
  now += 4000u; ui_main_update();
  check("expiring an overlapping mark preserves the live mark", red_square(102,55) && fb[55][98] != LCD_RED);
  now += 1000u; ui_main_update();
  memcpy(baseline, fb, sizeof(fb)); ui_main_draw_all();
  check("overlapping marks leave no red residue", memcmp(baseline, fb, sizeof(fb)) == 0);
  reset_ui(220000u);
  status(CANP_STATE_CV, 5000u, 100u);
  finger(1, 30, 68); ui_main_update(); finger(0, 0, 0); ui_main_update();
  now += 1000u; status(CANP_STATE_CV, 12340u, 345u);
  check("measurement updates preserve the live overlay", red_square(30,68));
  before_pixels = lcd_pixels; before_fills = full_fills;
  now += 4000u; ui_main_update();
  check("expiry restores updated data without a full refresh", lcd_pixels - before_pixels == 25u && full_fills == before_fills);
  memcpy(baseline, fb, sizeof(fb)); ui_main_draw_all();
  check("restored measurement matches the latest render", memcmp(baseline, fb, sizeof(fb)) == 0);
  reset_ui(300000u);
  finger(1, 30, 12); ui_main_update(); finger(0,0,0); ui_main_update();
  check("header opens temperature page offline", has_log("page=TEMPERATURE"));
  snapshot("temperature_offline.ppm");
  uint16_t saved_v = ui_model()->ch1.v_set_mv;
  handler(IN_EVT_ENCODER, IN_ACT_PRESSED, 1);
  check("temperature page does not edit hidden setpoints", ui_model()->ch1.v_set_mv == saved_v);
  status(CANP_STATE_CV, 5000u, 100u);
  ui_channel_t *tc = &ui_model()->ch1;
  tc->temp_seen = 1; tc->temp_valid = 3; tc->temp_rx_ms = now;
  tc->tmos_cdeg = 2500; tc->tl_cdeg = -500;
  ui_main_update(); snapshot("temperature_live.ppm");
  finger(1, 30, 68); ui_main_update(); finger(0,0,0); ui_main_update();
  before_fills = full_fills;
  now += 1000; tc->tmos_cdeg = 4567; tc->temp_rx_ms = now; ui_main_update();
  check("temperature update keeps red touch overlay", red_square(30,68));
  now += 4000; tc->temp_rx_ms = now; ui_main_update();
  check("temperature refresh and mark expiry never fill screen", full_fills == before_fills);
  memcpy(baseline, fb, sizeof(fb)); ui_main_draw_all();
  check("temperature partial restore matches fresh render", memcmp(baseline,fb,sizeof(fb)) == 0);
  tc->temp_valid = 1; ui_main_update(); snapshot("temperature_sensor_error.ppm");
  memcpy(baseline,fb,sizeof(fb)); now += CANP_TEMP_STALE_MS + 1; ui_main_update();
  check("stale temperature visibly removes old reading", memcmp(baseline,fb,sizeof(fb)) != 0);
  snapshot("temperature_stale.ppm");
  finger(1, 250, 196); ui_main_update(); finger(0,0,0); ui_main_update();
  check("BACK returns to power without sending setpoint", has_log("page=POWER") && tx_commands == 1);
  check("temperature page rectangles stay on screen", bad_rect == 0);
  printf("%d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
