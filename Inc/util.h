#ifndef UTIL_H
#define UTIL_H

#include "stm32f1xx_hal.h"

uint8_t  uread8(I2C_HandleTypeDef *i2c, uint8_t address, uint8_t dir);
int8_t 	 read8(I2C_HandleTypeDef *i2c, uint8_t address, uint8_t dir);

uint16_t uread16(I2C_HandleTypeDef *i2c, uint8_t address, uint8_t dir);
int16_t  read16(I2C_HandleTypeDef *i2c, uint8_t address, uint8_t dir);

uint32_t uread32(I2C_HandleTypeDef *i2c, uint8_t address, uint8_t dir);
int32_t  read32(I2C_HandleTypeDef *i2c, uint8_t address, uint8_t dir);


#endif
