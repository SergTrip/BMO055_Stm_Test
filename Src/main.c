/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 23/05/2015 00:41:39
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
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
#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN 0 */

#include "usbd_cdc_if.h"

// D:\MyPROG\LSD_SLAM\STM\BNO055_driver
#include "bno055.h"

void BNO055_Error( void );


// Массив тестовых данных
int16_t testDataToSend[] = {0, 0, 0, 0, 0, 0, 0, 0};

uint8_t IDData[] = {0, 0, 0, 0};

// Резльтат попытки подключния к драфверу
HAL_StatusTypeDef i2cRes;

HAL_I2C_StateTypeDef i2cState;

uint32_t i2cError;

#define BNO055_DEVICE_ADDRESS 0x28

uint16_t devAddress; 
	
/* USER CODE END 0 */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* System interrupt init*/
  /* Sets the priority grouping field */
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C3_Init();
  MX_TIM7_Init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */
	
	// Включить перефирию		
	devAddress = BNO055_DEVICE_ADDRESS; 
	
	testDataToSend[0] = 0xAA;
	
	// Попробовать запустить акселерометр
	// i2cRes = HAL_I2C_IsDeviceReady( &hi2c3, devAddress, 5, 1000 );
	
//	HAL_GPIO_WritePin ( GPIOC, GPIO_PIN_9, GPIO_PIN_RESET );	
//	HAL_GPIO_WritePin ( GPIOA, GPIO_PIN_8, GPIO_PIN_RESET );
//	
		__I2C3_CLK_DISABLE();
		
		HAL_Delay(1000);	
		
		__I2C3_CLK_ENABLE();
//	
//	HAL_GPIO_WritePin ( GPIOC, GPIO_PIN_9, GPIO_PIN_SET );
//	HAL_GPIO_WritePin ( GPIOA, GPIO_PIN_8, GPIO_PIN_SET );   

	
	i2cState = HAL_I2C_GetState(&hi2c3);
	
	if( HAL_I2C_STATE_READY != i2cState )
	{
		// Уходим в бесконечный цикл
		while(1) {}
	}
	
	// Пробуем читать ID устройства
	// i2cRes = HAL_I2C_Master_Receive ( &hi2c3, (uint16_t)BNO055_I2C_ADDR1<<1, (uint8_t*)testDataToSend, 1, 1000 );
	uint16_t reg = 0x01;
	i2cRes = HAL_I2C_Mem_Read ( 					&hi2c3, 
															(uint16_t)BNO055_I2C_ADDR1<<1,		// Адрес устройства
																				0x2, 										// Адрес регистра
																				I2C_MEMADD_SIZE_8BIT,		// Размерность данных
																				IDData, 								// Буфер для получения данных
																				4, 											// Сколько данных нужно получить
																				100);
	
	if( HAL_OK != i2cRes )
	{
		BNO055_Error();
	}
	

	IDData[0] = OPERATION_MODE_NDOF;
	// Пробуем поменять режим устройства	
	i2cRes = HAL_I2C_Mem_Write( 					&hi2c3, 
															(uint16_t)BNO055_I2C_ADDR1<<1,		// Адрес устройства
																				0x3D, 									// Адрес регистра
																				I2C_MEMADD_SIZE_8BIT, 	// Размерность данных
																				IDData, 								// Буфер для получения данных
																				1, 											// Сколько данных нужно отправить
																				100);
	
	if( HAL_OK != i2cRes )
	{
		BNO055_Error();
	}
	
	// Дадим время на переключение
	HAL_Delay(3000);
	
/*
	// Пробуем поменять режим устройства	
	i2cRes = HAL_I2C_Mem_Read( 					&hi2c3, 
															(uint16_t)BNO055_I2C_ADDR1<<1,		// Адрес устройства
																				BNO055_OPR_MODE_ADDR, 	// Адрес регистра
																				8, 											// Размерность данных
																				&IDData[1], 						// Буфер для получения данных
																				1, 											// Сколько данных нужно отправить
																				1000);
	
	if( HAL_OK != i2cRes )
	{
		BNO055_Error();
	}
	*/
	// BNO055_GRAVITY_DATA_X_LSB_ADDR
	// BNO055_QUATERNION_DATA_W_LSB_ADDR
	
		// Пробуем читать ID устройства
		i2cRes = HAL_I2C_Mem_Read ( 				&hi2c3, 
															(uint16_t)BNO055_I2C_ADDR1<<1,									// Адрес устройства
															(uint16_t)BNO055_GRAVITY_DATA_X_LSB_ADDR, 			// Адрес регистра
																				I2C_MEMADD_SIZE_8BIT, 								// Размерность данных
																				(uint8_t*)(&testDataToSend[1]), 			// Буфер для получения данных
																				2, 																		// Сколько данных нужно получить
																				1000);
	
	if( HAL_OK != i2cRes )
	{
		BNO055_Error();
	}
	
	// Запустить таймер
	HAL_TIM_Base_Start_IT( &htim7 );

  /* USER CODE END 2 */
	

  /* USER CODE BEGIN 3 */
  /* Infinite loop */
  while (1)
  {

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// Меняем состояние на противоположное
	HAL_GPIO_TogglePin ( GPIOD, GPIO_PIN_13 );
	
	/* Read Acceleration*/
	// BSP_ACCELERO_GetXYZ( (testDataToSend+1) );
	
	// Пробуем читать ID устройства
		i2cRes = HAL_I2C_Mem_Read ( 				&hi2c3, 
															(uint16_t)BNO055_I2C_ADDR1<<1,									// Адрес устройства
															(uint16_t)BNO055_QUATERNION_DATA_W_LSB_ADDR, 		// Адрес регистра
																				I2C_MEMADD_SIZE_8BIT, 								// Размерность данных
																				(uint8_t*)(&testDataToSend[1]), 			// Буфер для получения данных
																				4*2, 																	// Сколько данных нужно получить
																				1000);
	
	if( HAL_OK != i2cRes )
	{
		{	
		// Уходим в бесконечный цикл
		i2cError = HAL_I2C_GetError ( &hi2c3 );
		while(1){}
}
	}
	
		// Пробуем читать ID устройства
		i2cRes = HAL_I2C_Mem_Read ( 				&hi2c3, 
															(uint16_t)BNO055_I2C_ADDR1<<1,									// Адрес устройства
															(uint16_t)BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR,	// Адрес регистра
																				I2C_MEMADD_SIZE_8BIT, 								// Размерность данных
																				(uint8_t*)(&testDataToSend[5]), 			// Буфер для получения данных
																				3*2, 																	// Сколько данных нужно получить
																				1000);
	
	if( HAL_OK != i2cRes )
	{
		{	
		// Уходим в бесконечный цикл
		i2cError = HAL_I2C_GetError ( &hi2c3 );
		while(1){}
}
	}
	
	// Отправить пакет по USB
	int8_t res = CDC_Transmit_FS( (uint8_t*)testDataToSend, 2*8 );
}

void BNO055_Error()
{	
	// Уходим в бесконечный цикл
	i2cError = HAL_I2C_GetError ( &hi2c3 );
	while(1){}
}

/* USER CODE END 4 */

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
