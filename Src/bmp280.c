/* Includes ------------------------------------------------------------------*/
#include "bmp280.h"

/**
* @brief  Get the id from the module
*         Checks if the mpu6050 is connected and power it on.
* @param  i2c_var: variable that is the i2c handler for HAL libray.
* @param  id: reference variable that contains the id.
* @retval if there is some error in the connection.
*/
int bmp280_get_id(I2C_HandleTypeDef *i2c, uint8_t *id){
	uint8_t temp;
	uint8_t address = (uint8_t)BMP_ADDRESS;
	uint8_t get_id = (uint8_t)ID;

	//Check if the MPU is connected
	if(HAL_I2C_Master_Transmit(i2c,address,&get_id,1,100)!=HAL_OK) return 1;
	if(HAL_I2C_Master_Receive(i2c,address,&temp,1,100) != HAL_OK){ return 2; }
	else *id = temp;

	return 0;
}

/**
* @brief  init
*         Checks if the bmp280 is connected and power it on with the desired configuration.
* @param  i2c_var: variable that is the i2c handler for HAL libray.
* @retval if there is some error in the initialization.
*/
int bmp280_init(I2C_HandleTypeDef *i2c,bmp280 * bmp){
	bmp280_ctrl_meas(i2c,1,16,2);
	bmp280_read_calib_reg(i2c,bmp);
}

/**
* @brief  Configure measurements
*         Confugure the measurements in the device.
* @param  i2c_var: variable that is the i2c handler for HAL libray.
* @param  temp_o: temperature oversampling (see documentation) 
* @param  press_o: pressure oversampling (see documentation) 
* @param  mode: power mode
* @retval if there is some error in the initialization.
*/
int bmp280_ctrl_meas(I2C_HandleTypeDef *i2c, int temp_o, int press_o, int mode){
	uint8_t address = (uint8_t)BMP_ADDRESS;
	unsigned char buffer[2];
	uint8_t meas_o=0x00,aux;

	switch(temp_o){
		case 0:
			meas_o |= 0x00;
			break;
		case 1:
			aux = (1 << 5);
			meas_o |= aux;
			break;
		case 2:
			aux = (1 << 6);
			meas_o |= aux;
			break;
		case 4:
			aux = (1 << 6);
			aux |= (1 << 5);
			meas_o |= aux;
			break;
		case 8:
			aux = (1 << 7);
			meas_o |= aux;
			break;
		case 16:
			aux = (1 << 7);
			aux |= (1 << 6);
			meas_o |= aux;
			break;
	}
	
	switch(press_o){
		case 0:
			meas_o |= 0x00;
			break;
		case 1:
			aux = (1 << 2);
			meas_o |= aux;
			break;
		case 2:
			aux = (1 << 3);
			meas_o |= aux;
			break;
		case 4:
			aux = (1 << 3);
			aux |= (1 << 2);
			meas_o |= aux;
			break;
		case 8:
			aux = (1 << 4);
			meas_o |= aux;
			break;
		case 16:
			aux = (1 << 4);
			aux |= (1 << 2);
			meas_o |= aux;
			break;
	}
	
	switch(mode){
		case 0:
			meas_o |= 0x00;
			break;
		case 1:
			aux = (1 << 0);
			meas_o |= aux;
			break;
		case 2:
			aux = (1 << 1);
			aux |= (1 << 0);
			meas_o |= aux; 
		break;
	}

	buffer[0] = (uint8_t)CTRL_MEAS;
	buffer[1] = (uint8_t)meas_o;

	if(HAL_I2C_Master_Transmit(i2c,address,buffer,2,100)!=HAL_OK) return 1;
	return 0;
}

/**
* @brief  Read calibration registers
*         Read the calibration registers and store the data.
* @param  i2c_var: variable that is the i2c handler for HAL libray.
* @param  bmp: reference variable with the bmp280 data.
* @retval if there is some error in the initialization.
*/
void bmp280_read_calib_reg(I2C_HandleTypeDef *i2c, bmp280 * bmp){
	uint8_t address = (uint8_t)BMP_ADDRESS;
	
	bmp->calibration.dig_t1 = uread16(i2c,address,(uint8_t)DIG_T1);
	bmp->calibration.dig_t2 = read16(i2c,address,(uint8_t)DIG_T2);
	bmp->calibration.dig_t3 = read16(i2c,address,(uint8_t)DIG_T3);
	bmp->calibration.dig_p1 = uread16(i2c,address,(uint8_t)DIG_P1);
	bmp->calibration.dig_p2 = read16(i2c,address,(uint8_t)DIG_P2);
	bmp->calibration.dig_p3 = read16(i2c,address,(uint8_t)DIG_P3);
	bmp->calibration.dig_p4 = read16(i2c,address,(uint8_t)DIG_P4);
	bmp->calibration.dig_p5 = read16(i2c,address,(uint8_t)DIG_P5);
	bmp->calibration.dig_p6 = read16(i2c,address,(uint8_t)DIG_P6);
	bmp->calibration.dig_p7 = read16(i2c,address,(uint8_t)DIG_P7);
	bmp->calibration.dig_p8 = read16(i2c,address,(uint8_t)DIG_P8);
	bmp->calibration.dig_p9 = read16(i2c,address,(uint8_t)DIG_P9);

}

/**
* @brief  Read pressure
*         Read the calibration registers and store the data.
* @param  i2c_var: variable that is the i2c handler for HAL libray.
* @param  bmp: reference variable with the bmp280 data.
* @retval if there is some error in the initialization.
*/
float bmp280_read_pressure(I2C_HandleTypeDef *i2c, bmp280 * bmp){
	uint8_t address = (uint8_t)BMP_ADDRESS;
	unsigned char buffer[3];
	uint32_t adc_P;
	int32_t t_fine;
	int64_t var1,var2,p;

	bmp280_read_temperature(i2c,bmp,&t_fine);

	buffer[0] = (uint8_t)PRESS_MSB;

	if(HAL_I2C_Master_Transmit(i2c,address,buffer,1,100)!=HAL_OK){ return 1; }
	if(HAL_I2C_Master_Receive(i2c,address,buffer,3,100) != HAL_OK){ return 2; }

	/*press_raw_value = (uint32_t)(buffer[0]);
    press_raw_value = press_raw_value << 8;
    press_raw_value |= (uint32_t)(buffer[1]);
    press_raw_value = press_raw_value << 8;
    press_raw_value |= (uint32_t)(buffer[2]);
    press_raw_value = press_raw_value << 8;
    press_raw_value |= (uint32_t)(buffer[3]);*/

    //adc_P = (uint32_t)(((buffer[0] << 16 | buffer[1] << 8 | buffer[2])<<8) | buffer[3]);
		adc_P = ((uint32_t)buffer[0] << 12) | ((uint32_t)buffer[1] << 4) | ((buffer[2] >> 4));
	
		var1 = ((int64_t)bmp->t_fine) - 128000;
		var2 = var1 * var1 * (int64_t)bmp->calibration.dig_p6;
		var2 = var2 + ((var1*(int64_t)bmp->calibration.dig_p5)<<17);
		var2 = var2 + (((int64_t)bmp->calibration.dig_p4)<<35);
		var1 = ((var1 * var1 * (int64_t)bmp->calibration.dig_p3)>>8) + ((var1 * (int64_t)bmp->calibration.dig_p2)<<12); 
		var1 = (((((int64_t)1)<<47)+var1))*((int64_t)bmp->calibration.dig_p1)>>33;
		if (var1 == 0) {
			return 0; // avoid exception caused by division by zero 
		}
		p = 1048576-adc_P;
		p = (((p<<31)-var2)*3125)/var1;
		var1 = (((int64_t)bmp->calibration.dig_p9)*(p>>13)*(p>>13)) >> 25; 
		var2 = (((int64_t)bmp->calibration.dig_p8)*p) >> 19;
		p = ((p + var1 + var2) >> 8) + (((int64_t)bmp->calibration.dig_p7)<<4); 

	return (float)p/256.0;
}

/**
* @brief  Read temperature
*         Read the calibration registers and store the data.
* @param  i2c_var: variable that is the i2c handler for HAL libray.
* @param  bmp: reference variable with the bmp280 data.
* @retval if there is some error in the initialization.
*/
float bmp280_read_temperature(I2C_HandleTypeDef *i2c, bmp280 * bmp, int32_t *t_fine){
	uint8_t address = (uint8_t)BMP_ADDRESS;
	uint8_t temp_addr = (uint8_t)TEMP_MSB;
	unsigned char buffer[3];
	int32_t adc_T,var1,var2;
	

	if(HAL_I2C_Master_Transmit(i2c,address,&temp_addr,1,100)!=HAL_OK){ return 1; }
	if(HAL_I2C_Master_Receive(i2c,address,buffer,3,100) != HAL_OK){ return 2; }

  //adc_T = (int32_t)(((buffer[0] << 16 | buffer[1] << 8 | buffer[2])<<8) | buffer[3]);
	adc_T = ((uint32_t)buffer[0] << 12) | ((uint32_t)buffer[1] << 4) | ((buffer[2] >> 4) & 0x0F);
	//Como viene en la documentación
	var1 = ((((adc_T>>3) - ((int32_t)bmp->calibration.dig_t1<<1)))*((int32_t)bmp->calibration.dig_t2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)bmp->calibration.dig_t1))*((adc_T>>4) - ((int32_t)bmp->calibration.dig_t1))) >> 12)*((int32_t)bmp->calibration.dig_t3)) >> 14;
	bmp->t_fine = var1 + var2;
	*t_fine = bmp->t_fine;
	float T =(bmp->t_fine*5+128)>>8;
	T = T/100;
	
	return T;
}
