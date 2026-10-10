/* Minimal HAL stand-in so GCC's static analyzer can see the H750 driver code.
 *
 * WHY: lcd.c, touch.c and gfx_text.c hold the most hand-written bounds and buffer
 * arithmetic in this project, and none of them has run on hardware yet - there is
 * no display to plug in. -fanalyzer can follow those paths without a board, so it
 * is the strongest check available for that code today. It needs the HAL types to
 * exist or it stops at the first parse error.
 *
 * SCOPE: types, pin identifiers and peripheral pointers ONLY. The pin LABELS
 * (LCD_CS_Pin, TOUCH_RST_Pin, ...) deliberately come from the real generated
 * Core/Inc/main.h - redefining them here would analyse a different program from
 * the one that gets flashed.
 *
 * This is NOT a functional fake. It is never compiled into the firmware and is
 * not used by the host protocol test (that uses User/HostTest/shim).
 */
#ifndef ANALYZE_HAL_STUB_H
#define ANALYZE_HAL_STUB_H

#include <stdint.h>

typedef enum { HAL_OK = 0, HAL_ERROR = 1, HAL_BUSY = 2, HAL_TIMEOUT = 3 } HAL_StatusTypeDef;

typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET = 1 } GPIO_PinState;

typedef struct { uint32_t dummy; } GPIO_TypeDef;
typedef struct { uint32_t dummy; } SPI_HandleTypeDef;
typedef struct { uint32_t dummy; } I2C_HandleTypeDef;

/* Peripheral instances */
extern GPIO_TypeDef *const GPIOA_stub;
extern GPIO_TypeDef *const GPIOB_stub;
extern GPIO_TypeDef *const GPIOC_stub;
extern GPIO_TypeDef *const GPIOD_stub;
extern GPIO_TypeDef *const GPIOE_stub;
#define GPIOA GPIOA_stub
#define GPIOB GPIOB_stub
#define GPIOC GPIOC_stub
#define GPIOD GPIOD_stub
#define GPIOE GPIOE_stub

/* Pin masks */
#define GPIO_PIN_0  ((uint16_t)0x0001)
#define GPIO_PIN_1  ((uint16_t)0x0002)
#define GPIO_PIN_2  ((uint16_t)0x0004)
#define GPIO_PIN_3  ((uint16_t)0x0008)
#define GPIO_PIN_4  ((uint16_t)0x0010)
#define GPIO_PIN_5  ((uint16_t)0x0020)
#define GPIO_PIN_6  ((uint16_t)0x0040)
#define GPIO_PIN_7  ((uint16_t)0x0080)
#define GPIO_PIN_8  ((uint16_t)0x0100)
#define GPIO_PIN_9  ((uint16_t)0x0200)

#define I2C_MEMADD_SIZE_8BIT 1u

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin);
void HAL_Delay(uint32_t ms);
uint32_t HAL_GetTick(void);

HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *h, const uint8_t *p, uint16_t n, uint32_t t);
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *h, uint16_t addr, uint16_t mem,
                                   uint16_t memsize, uint8_t *buf, uint16_t n, uint32_t t);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *h, uint16_t addr, uint16_t mem,
                                    uint16_t memsize, uint8_t *buf, uint16_t n, uint32_t t);
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *h, uint16_t addr, uint32_t trials,
                                        uint32_t t);

extern SPI_HandleTypeDef hspi1;
extern I2C_HandleTypeDef hi2c1;

#endif /* ANALYZE_HAL_STUB_H */
