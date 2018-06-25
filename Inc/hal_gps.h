/**
  ******************************************************************************
  * @file    hal_gps.h
  * @author  Pedro Torres
  * @version V1.0
  * @date    30-January-2018
  * @brief   Header file for hal_gps.c file
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HAL_GPS_H
#define HAL_GPS_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

/* Private variables ---------------------------------------------------------*/
/**
* @brief Private struct variable to store all data from the GPS.
*/
struct GPS_var{
	float speed;
	float latitude;
	float longitude;
	int32_t time;
	int32_t date;
	int8_t state;
	char direction;
	int8_t checksum;
};

typedef struct GPS_var gps_hal;

/** @defgroup GPS_FunctionsPrototype
  * @{
  */
int get_gps_data(UART_HandleTypeDef *huart, UART_HandleTypeDef *ftdi, gps_hal * gps);
void print_data();
void print_raw_data(UART_HandleTypeDef *huart, UART_HandleTypeDef *ftdi);
void format_position(char * l, char * pos, float *f_position);
int calculate_checksum(char data[], int checksum_value);

#endif
