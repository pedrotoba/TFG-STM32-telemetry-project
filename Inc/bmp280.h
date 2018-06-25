/**
  ******************************************************************************
  * @file    bmp280.h
  * @author  Pedro Torres
  * @version V1.0
  * @date    30-January-2018
  * @brief   Header file for bmp280.c file
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BMP280_H
#define BMP280_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "util.h"


#define BMP_ADDRESS 0x76<<1

/**
  * @brief  BMP280 compesation registers 
  */
#define DIG_T1 		0x88
#define DIG_T2 		0x8A
#define DIG_T3 		0x8C
#define DIG_P1 		0x8E
#define DIG_P2 		0x90
#define DIG_P3 		0x92
#define DIG_P4 		0x94
#define DIG_P5 		0x96
#define DIG_P6 		0x98
#define DIG_P7 		0x9A
#define DIG_P8 		0x9C
#define DIG_P9 		0x9E

/**
  * @brief  BMP280 temperature registers 
  */
#define TEMP_MSB 	0xFA
#define TEMP_LSB 	0xFB
#define TEMP_XLSB	0xFC

/**
  * @brief  BMP280 pressure registers 
  */
#define PRESS_MSB 	0xF7
#define PRESS_LSB 	0xF8
#define PRESS_XLSB	0xF9

/**
  * @brief  BMP280 configuration registers 
  */
#define CONFIG 		0xF5
#define CTRL_MEAS	0xF4
#define STATUS 		0xF3
#define RESET 		0xE0
#define ID 				0xD0

/* Private variables ---------------------------------------------------------*/
/**
* @brief Private struct variable to store all the calibration registers from the BMP280.
*/
struct BMP_calibration_registers{
	uint16_t 	dig_t1;
	int16_t 	dig_t2;
	int16_t		dig_t3;

	uint16_t	dig_p1;
	int16_t		dig_p2;
	int16_t		dig_p3;
	int16_t		dig_p4;
	int16_t		dig_p5;
	int16_t		dig_p6;
	int16_t		dig_p7;
	int16_t		dig_p8;
	int16_t		dig_p9;
};	
typedef struct BMP_calibration_registers BMP_calib;

/**
* @brief Private struct variable to store all the data from the BMP280.
*/
struct BMP_data{
	BMP_calib calibration;
	int16_t temperature;
	int16_t pressure;
	int32_t t_fine;
	float altitude;
};
typedef struct BMP_data bmp280;

/** @defgroup BMP280_FunctionsPrototype
  * @{
  */
int bmp280_get_id(I2C_HandleTypeDef *i2c, uint8_t *id); //0x58
int bmp280_init(I2C_HandleTypeDef *i2c, bmp280 * bmp);
int bmp280_ctrl_meas(I2C_HandleTypeDef *i2c, int temp_o, int press_o, int mode); //Data oversampling
void bmp280_read_calib_reg(I2C_HandleTypeDef *i2c, bmp280 * bmp);
float bmp280_read_pressure(I2C_HandleTypeDef *i2c, bmp280 * bmp);
float bmp280_read_temperature(I2C_HandleTypeDef *i2c, bmp280 * bmp, int32_t *t_fine);


#endif
