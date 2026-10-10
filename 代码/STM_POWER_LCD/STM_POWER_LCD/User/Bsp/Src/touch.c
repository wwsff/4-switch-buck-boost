/**
  ******************************************************************************
  * @file    touch.c
  * @brief   FT6336U capacitive touch driver.
  *
  *          Register map is the one shared by the FT6236/FT6336 family and is
  *          what the reference demo reads. Confirm against the panel datasheet
  *          that ships with the module before trusting the coordinates.
  *
  *          I2C1 runs at ~100 kHz (see the .ioc). Raise it to 400 kHz only
  *          after touch is verified working.
  ******************************************************************************
  */
#include "touch.h"
#include "i2c.h"
#include "bsp_log.h"

/* --------------------------------------------------------------------------
 * FT6336 register map
 * -------------------------------------------------------------------------- */
#define FT_REG_DEV_MODE      0x00u
#define FT_REG_GEST_ID       0x01u
#define FT_REG_TD_STATUS     0x02u
#define FT_REG_P1_XH         0x03u   /* 4 bytes: XH, XL, YH, YL */
#define FT_REG_P1_XY         0x03u
#define FT_REG_P2_XH         0x09u
#define FT_REG_P2_XY         0x09u
#define FT_REG_ID_G_THGROUP  0x80u
#define FT_REG_ID_G_CTRL     0x86u
#define FT_REG_ID_G_CIPHER   0xA3u   /* full-byte chip code */
#define FT_REG_ID_G_MODE     0xA4u

/* --------------------------------------------------------------------------
 * Low level I2C
 * -------------------------------------------------------------------------- */
static uint16_t s_address = FT6336_I2C_ADDR;
static uint32_t s_last_error;
static HAL_StatusTypeDef s_last_status = HAL_OK;
static uint8_t s_last_count = 0xFFu;

uint16_t touch_address(void) { return s_address; }
uint32_t touch_last_error(void) { return s_last_error; }
HAL_StatusTypeDef touch_last_status(void) { return s_last_status; }
uint8_t touch_last_count(void) { return s_last_count; }

static HAL_StatusTypeDef ft_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
  s_last_status = HAL_I2C_Mem_Read(&hi2c1, s_address, reg,
                          I2C_MEMADD_SIZE_8BIT, buf, len, FT6336_I2C_TIMEOUT_MS);
  s_last_error = HAL_I2C_GetError(&hi2c1);
  return s_last_status;
}

/* --------------------------------------------------------------------------
 * Public
 * -------------------------------------------------------------------------- */
HAL_StatusTypeDef touch_read_id(uint8_t *id)
{
  if (id == NULL) {
    return HAL_ERROR;
  }
  uint8_t raw = 0u;
  if (ft_read(FT_REG_ID_G_CIPHER, &raw, 1u) != HAL_OK) {
    return HAL_ERROR;
  }
  *id = raw;
  return HAL_OK;
}

HAL_StatusTypeDef touch_init(void)
{
  /* Use the supplied module demo's reset timing. */
  TOUCH_RST_LOW();
  HAL_Delay(20u);
  TOUCH_RST_HIGH();
  HAL_Delay(300u);              /* FT6336 needs ~300 ms after reset */

  s_address = FT6336_I2C_ADDR;
  s_last_count = 0xFFu;
  /* Both addresses are documented FT6336 module variants. Probe once at boot;
   * do not reset/probe in the GUI loop or accept arbitrary bus addresses. */
  if (HAL_I2C_IsDeviceReady(&hi2c1, s_address, 2u, 20u) != HAL_OK) {
    uint16_t alternate = (uint16_t)(0x48u << 1);
    if (HAL_I2C_IsDeviceReady(&hi2c1, alternate, 2u, 20u) != HAL_OK) {
      s_last_status = HAL_ERROR;
      s_last_error = HAL_I2C_GetError(&hi2c1);
      LOGE("TOUCH", "no_ack at 7bit 0x38/0x48 err=0x%08lX", (unsigned long)s_last_error);
      return HAL_ERROR;
    }
    s_address = alternate;
  }
  LOGI("TOUCH", "device_ack addr7=0x%02X", (unsigned)(s_address >> 1));

  uint8_t id = 0u;
  if (touch_read_id(&id) != HAL_OK) {
    return HAL_ERROR;
  }
  if ((id != FT6336_ID_6336) && (id != FT6336_ID_6336M)) {
    /* Module firmware revisions may differ. Keep the full ID in the log;
     * an answering module may still be used for coordinate bring-up. */
    LOGW("TOUCH", "unexpected chip_id=0x%02X; testing coordinates", (unsigned)id);
  }

  /* The supplied demo queries the power-on default state without writing
   * configuration registers. GUI polling does not depend on INT mode. */
  return HAL_OK;
}

HAL_StatusTypeDef touch_read(touch_state_t *state)
{
  if (state == NULL) {
    return HAL_ERROR;
  }
  state->count = 0u;
  state->x[0] = 0u; state->y[0] = 0u;
  state->x[1] = 0u; state->y[1] = 0u;

  /* Follow the supplied module demo: status at 0x02, then each active point
   * as four bytes at 0x03/0x09. No reads across unused weight/misc registers. */
  uint8_t status = 0u;
  s_last_count = 0xFFu;
  if (ft_read(FT_REG_TD_STATUS, &status, 1u) != HAL_OK) {
    return HAL_ERROR;
  }

  uint8_t count = (uint8_t)(status & 0x0Fu);
  s_last_count = status;
  if (count > TOUCH_MAX_POINTS) { s_last_status = HAL_ERROR; return HAL_ERROR; }
  if (count == 0u) {
    return HAL_OK;
  }

  for (uint8_t point = 0u; point < count; point++) {
    uint8_t raw[4];
    uint8_t reg = (point == 0u) ? FT_REG_P1_XY : FT_REG_P2_XY;
    if (ft_read(reg, raw, sizeof(raw)) != HAL_OK) { return HAL_ERROR; }
    state->x[point] = (uint16_t)((((uint16_t)raw[0] & 0x0Fu) << 8) | raw[1]);
    state->y[point] = (uint16_t)((((uint16_t)raw[2] & 0x0Fu) << 8) | raw[3]);
  }
  state->count = count; /* publish only once all requested point reads succeeded */
  return HAL_OK;
}

uint8_t touch_irq_asserted(void)
{
  return TOUCH_INT_ASSERTED() ? 1u : 0u;
}

void touch_scan_bus(void)
{
  for (uint16_t addr = 0x02u; addr < 0xFEu; addr += 2u) {
    if (HAL_I2C_IsDeviceReady(&hi2c1, addr, 1u, 2u) == HAL_OK) {
      /* Logged by the caller via the platform log macro. */
      bsp_log_scan_hit(addr);
    }
  }
}
