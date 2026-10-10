/* Host shim for main.h.
 *
 * can_proto.c reaches main.h transitively (bsp_log.h -> bsp_config.h -> main.h),
 * and main.h pulls in stm32h7xx_hal.h, which cannot be compiled on a PC. The
 * protocol state machine only ever wants HAL_GetTick() from all of that, so this
 * shim declares exactly that and nothing else.
 *
 * This directory goes FIRST on the include path for the host build only.
 */
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum { HAL_OK = 0, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET } GPIO_PinState;
#define TOUCH_RST_GPIO_Port ((void *)0)
#define TOUCH_RST_Pin 1u
#define TOUCH_INT_GPIO_Port ((void *)0)
#define TOUCH_INT_Pin 2u
void HAL_Delay(uint32_t ms);
void HAL_GPIO_WritePin(void *port, uint16_t pin, GPIO_PinState state);
GPIO_PinState HAL_GPIO_ReadPin(void *port, uint16_t pin);

/* Milliseconds since boot. The host test advances this explicitly. */
uint32_t HAL_GetTick(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
