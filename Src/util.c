#include "util.h"

uint8_t uread8(I2C_HandleTypeDef *i2c, uint8_t address, uint8_t dir){
	uint8_t val;

	HAL_I2C_Master_Transmit(i2c,address,&dir,1,100);
	HAL_I2C_Master_Receive(i2c,address,&val,1,100);

	return val;
}

int8_t read8(I2C_HandleTypeDef *i2c, uint8_t address, uint8_t dir){
	return (int8_t)uread8(i2c,address,dir);
}

uint16_t uread16(I2C_HandleTypeDef *i2c, uint8_t address, uint8_t dir){
	uint16_t val;
	unsigned char buffer[2];

	if(HAL_I2C_Master_Transmit(i2c,address,&dir,1,100) != HAL_OK) return 0;
	if(HAL_I2C_Master_Receive(i2c,address,buffer,2,100) != HAL_OK) return 0;
	val = buffer[0]<<8 | buffer[1];

	return val;
}

int16_t  read16(I2C_HandleTypeDef *i2c, uint8_t address, uint8_t dir){
	return (int16_t)uread16(i2c,address,dir);
}

uint32_t uread32(I2C_HandleTypeDef *i2c, uint8_t address, uint8_t dir){
	uint32_t val;
	unsigned char buffer[3];

	HAL_I2C_Master_Transmit(i2c,address,&dir,1,100);
	HAL_I2C_Master_Receive(i2c,address,buffer,3,100);
	val = buffer[0]<<16 | buffer[1] << 8 | buffer[2];

	return val;
}

int32_t  read32(I2C_HandleTypeDef *i2c, uint8_t address, uint8_t dir){
	return (int32_t)uread32(i2c,address,dir);
}
