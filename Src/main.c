/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "math.h"

/* USER CODE BEGIN Includes */
#include "string.h"
#include "mpu6050.h"
#include "hal_gps.h"
#include "bmp280.h"
/* USER CODE END Includes */
#define M_PI 3.14159265358979323846
/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint32_t tickstart;
uint32_t tickstart_print;
uint32_t tickstart_gps;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void UART_send_i(int val);
void UART_send_f(float val);
void UART_send_s(char val[]);

float ac_get_rotation_x(float ac_x, float ac_y, float ac_z);
float ac_get_rotation_y(float ac_x, float ac_y, float ac_z);

int calculate_checksum_s(char str[]);
void add_checksum_array(char arr[]);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
float temp;
uint8_t id;
mpu6050 mpu; //hi2c2
gps_hal gps;
bmp280 bmp; //hi2c1
int aux = 0, aux2=0, aux3=0;
uint8_t temp_id;


char *calib_msg="Calibrating gyro...";

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
	init(&hi2c2);
	HAL_GPIO_WritePin(LED_INFO_GPIO_Port, LED_INFO_Pin, GPIO_PIN_SET);
	gyro_configuration(&hi2c2, 250);
	accel_configuration(&hi2c2, 8);
	HAL_UART_Transmit(&huart1,(uint8_t*)calib_msg,strlen(calib_msg),10);
	calibration(&hi2c2, &mpu);
	HAL_GPIO_WritePin(LED_INFO_GPIO_Port, LED_INFO_Pin, GPIO_PIN_RESET);
	bmp280_init(&hi2c1,&bmp);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	int b = 0;
	uint8_t id;
	char mpu_data[30]="";
	char gps_data[30]="";
	char gps_data_dt[30]="";
	char bmp_data[30]="";
	char temp_data[30]="";
	float temperatura,press;
	int mostrado = 0;
	
	int16_t acc_x, acc_y, acc_z, acc_total_vector;
	int32_t gyro_x_cal=0,gyro_y_cal=0,gyro_z_cal=0; // gyro calibration value
	float angle_pitch, angle_roll, angle_pitch_acc,angle_roll_acc;
	float pitch =0,roll = 0, gyro_x_delta, gyro_y_delta, gyro_x_total, gyro_y_total, last_x = 0.0, last_y = 0.0, rotation_x, rotation_y;
	int gps_s = 0;
	int32_t tfine_value;
	//END VARIABLES
	
	gps.state = 0;
	tickstart = HAL_GetTick();
	tickstart_print = HAL_GetTick();
	tickstart_gps = HAL_GetTick();
	
  while (1)
  {
		
		if((HAL_GetTick() - tickstart_gps) >= 1000){
			get_gps_data(&huart2,&huart1,&gps);
			tickstart_gps = HAL_GetTick();
		}

		if((HAL_GetTick() - tickstart) >= 10){

			if(read_gyro(&hi2c2, &mpu)!=0 || read_acc(&hi2c2, &mpu)!=0 || read_temperature(&hi2c2, &mpu)!=0) HAL_UART_Transmit(&huart1,(uint8_t*)"ERROR MPU",10,100);
			else {
				float g_x_f = (mpu.gyro_x / 131.0) - (mpu.gyro_x_cal/131.0);
				float g_y_f = (mpu.gyro_y / 131.0) - (mpu.gyro_y_cal/131.0);
				float g_z_f = (mpu.gyro_z / 131.0) - (mpu.gyro_z_cal/131.0);
				
				float a_x_f = mpu.accel_x / 4096.0;
				float a_y_f = mpu.accel_y / 4096.0;
				float a_z_f = mpu.accel_z / 4096.0;
				
				gyro_x_delta = g_x_f * 0.001;
				gyro_y_delta = g_y_f * 0.001;
				
				gyro_x_total = gyro_x_total + gyro_x_delta;
				gyro_y_total = gyro_y_total + gyro_y_delta;
				
				rotation_x = atan2(a_x_f, a_z_f) * 180/M_PI;
				rotation_y = atan2(a_x_f, sqrt(a_y_f*a_y_f + a_z_f*a_z_f)) * 180/M_PI;	
				
				last_x = 0.95 * (last_x + gyro_x_delta) + (0.05 * rotation_x);
				last_y = 0.95 * (last_y + gyro_y_delta) + (0.05 * rotation_y);

			}
			
			if(bmp280_read_temperature(&hi2c1,&bmp,&tfine_value) != 0 ) HAL_UART_Transmit(&huart1,(uint8_t*)"ERROR BMP",3,100);
			if(bmp280_read_pressure(&hi2c1,&bmp) != 0) HAL_UART_Transmit(&huart1,(uint8_t*)"ERROR BMP",3,100);
			
			tickstart = HAL_GetTick();
		}

		if(HAL_GetTick() - tickstart_print >= 200){
			
			sprintf(mpu_data, "P:%f,R:%f",pitch,roll);
			add_checksum_array(mpu_data);
			HAL_UART_Transmit(&huart1,(uint8_t*)mpu_data,30,10);
			HAL_Delay(30);
			
			sprintf(temp_data, "T:%f,S:%f",mpu.temperature,gps.speed);
			add_checksum_array(temp_data);
			HAL_UART_Transmit(&huart1,(uint8_t*)temp_data,30,10);
			HAL_Delay(30);
			
			sprintf(bmp_data, "Pr:%f",bmp.pressure);
			add_checksum_array(bmp_data);
			HAL_UART_Transmit(&huart1,(uint8_t*)bmp_data,30,10);
			HAL_Delay(30);
			
			if(gps.state == 0) HAL_UART_Transmit(&huart1,(uint8_t*)"L:-1,-1\n",10,10);
			else{
				sprintf(gps_data, "La:%f,Lo:%f",gps.latitude,gps.longitude);
				add_checksum_array(gps_data);
				HAL_UART_Transmit(&huart1,(uint8_t*)gps_data,30,10);
				HAL_Delay(30);
				sprintf(gps_data_dt, "D:%d,Ti:%d",gps.date,gps.time);
				add_checksum_array(gps_data_dt);
				HAL_UART_Transmit(&huart1,(uint8_t*)gps_data_dt,30,10);
				HAL_Delay(30);
			}
			
			tickstart_print = HAL_GetTick();
		}
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C2 init function */
static void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through 
        * the Code Generation settings)
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_DATA_Pin|LED_INFO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA4 PA5 
                           PA8 PA11 PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_DATA_Pin LED_INFO_Pin */
  GPIO_InitStruct.Pin = LED_DATA_Pin|LED_INFO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB12 
                           PB13 PB14 PB15 PB3 
                           PB4 PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_12 
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void UART_send_i(int val){
	char aux_msg[10]="";
	snprintf(aux_msg,10,"%d\n",val);
	HAL_GPIO_WritePin(LED_DATA_GPIO_Port, LED_DATA_Pin, GPIO_PIN_SET);
	HAL_UART_Transmit(&huart1,(uint8_t*)aux_msg,5,10);
	HAL_GPIO_WritePin(LED_DATA_GPIO_Port, LED_DATA_Pin, GPIO_PIN_RESET);
	//HAL_Delay(50);
}

void UART_send_f(float val){
	
	char aux_msg[20]="";
	snprintf(aux_msg,20,"%f\n",val);
	HAL_GPIO_WritePin(LED_DATA_GPIO_Port, LED_DATA_Pin, GPIO_PIN_SET);
	HAL_UART_Transmit(&huart1,(uint8_t*)aux_msg,10,10);
	HAL_GPIO_WritePin(LED_DATA_GPIO_Port, LED_DATA_Pin, GPIO_PIN_RESET);
	//HAL_Delay(50);
}

void UART_send_s(char val[]){

}

float ac_get_rotation_x(float ac_x, float ac_y, float ac_z){
	float roll;
	roll = atan2(ac_y, ac_z) * 180/M_PI;
	return roll;
}

float ac_get_rotation_y(float ac_x, float ac_y, float ac_z){
	float pitch;
	pitch = atan2(ac_x, sqrt(ac_y*ac_y + ac_z*ac_z)) * 180/M_PI;	
	return pitch;
}

int calculate_checksum_s(char str[]){
    int checksum = 0;
    for(int i=0; i < strlen(str); i++){
        checksum = checksum ^ str[i];
    }
    return checksum;
}

void add_checksum_array(char arr[]){
    char c_val[50] = "";
    int checksum;
    checksum = calculate_checksum_s(arr);
    sprintf(c_val,"*%x\n",checksum);
    strcat(arr,c_val);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
