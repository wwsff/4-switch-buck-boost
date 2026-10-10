#ifndef HOST_I2C_H
#define HOST_I2C_H
#include "main.h"
typedef struct { uint32_t unused; } I2C_HandleTypeDef;
extern I2C_HandleTypeDef hi2c1;
uint32_t HAL_I2C_GetError(I2C_HandleTypeDef *);
#define I2C_MEMADD_SIZE_8BIT 1u
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *, uint16_t, uint16_t,
                                  uint16_t, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *, uint16_t, uint16_t,
                                   uint16_t, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *, uint16_t, uint32_t, uint32_t);
#endif
