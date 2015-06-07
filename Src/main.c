/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 06/06/2015 19:36:42
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
#include "crc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN 0 */

//------------------------------ Includes -------------------------------------
// Для работы с устройством USB
#include "usbd_cdc_if.h"
// Определены адреса регистров сенсора
// D:\MyPROG\LSD_SLAM\STM\BNO055_driver
#include "bno055.h"
// Определены адреса регистров видео камеры
// D:\MyPROG\STM32\STM32Cube_FW_F4_V1.6.0\Drivers\BSP\Components\ov2640
//#include "ov2640.h"

//------------------------------ Defines -------------------------------------
// Управление питанием видеокамеры
#define  CAMERA_PWR_EN_PIN		GPIO_PIN_6
#define  CAMERA_PWR_EN_PORT		GPIOD
// Зеленый светодиод и сброс камеры
#define  CAMERA_RST_PIN			  GPIO_PIN_12	
#define  CAMERA_RST_PORT		  GPIOD
// Оранжевый сетодиод
#define  ORANG_LED_PIN		GPIO_PIN_13	
#define  ORANG_LED_PORT		GPIOD
// Красный сетодиод
#define  RED_LED_PIN			GPIO_PIN_14	
#define  RED_LED_PORT		  GPIOD
// Синий сетодиод
#define  BLUE_LED_PIN			GPIO_PIN_14	
#define  BLUE_LED_PORT		GPIOD

// Адресс датчика 
#define BNO055_DEVICE_ADDRESS 0x28
// Адресс камеры 
#define CAMERA_DEVICE_ADDRESS 0x21
//------------------------------ Globals Variables ----------------------------
// Массив тестовых данных
int16_t testDataToSend[] = {0, 0, 0, 0, 0, 0, 0, 0};

uint8_t IDData[] = {0, 0, 0, 0};

// Резльтат попытки подключния к драфверу
HAL_StatusTypeDef i2cRes;

HAL_I2C_StateTypeDef i2cState;

uint32_t i2cError;



uint16_t devAddress; 

uint32_t 						dcmiError = 0;
HAL_StatusTypeDef		dcmiStatus; 

uint16_t imageBuffer[1280]; 
uint16_t lineCounter = 0;

//------------------------- Private function prototypes ------------------------
// Обработка ошибок датчика
void BNO055_Error( void );
	
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
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_2);
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CRC_Init();
  MX_I2C1_Init();
  MX_TIM7_Init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */
		/*Reset camera*/
  HAL_GPIO_WritePin(CAMERA_RST_PORT, CAMERA_RST_PIN, GPIO_PIN_RESET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(CAMERA_RST_PORT, CAMERA_RST_PIN, GPIO_PIN_SET);
	
	/* camera PWR EN pin configuration */
  HAL_GPIO_WritePin(CAMERA_PWR_EN_PORT, CAMERA_PWR_EN_PIN, GPIO_PIN_RESET);
	
	// Включить перефирию		
	devAddress = BNO055_DEVICE_ADDRESS; 
	
	testDataToSend[0] = 0xAA;
	
	// Попробовать запустить акселерометр
	// i2cRes = HAL_I2C_IsDeviceReady( &hi2c3, (uint16_t)BNO055_I2C_ADDR1<<1, 5, 1000 );

	i2cState = HAL_I2C_GetState(&hi2c1);
	
	if( HAL_I2C_STATE_READY != i2cState )
	{
		// Уходим в бесконечный цикл
		while(1) {}
	}
	
	// Пробуем читать ID устройства ( Это рабочая фукция !!! ) *********************************
	i2cRes = HAL_I2C_Mem_Read ( 					&hi2c1, 
															(uint16_t)BNO055_I2C_ADDR1<<1,		// Адрес устройства
																				0x2, 										// Адрес регистра
																				I2C_MEMADD_SIZE_8BIT,		// Размерность данных
																				IDData, 								// Буфер для получения данных
																				4, 											// Сколько данных нужно получить
																				100);
	//********************************************************************************************
	
//	i2cRes = HAL_I2C_Mem_Read_DMA ( 					&hi2c1, 
//															(uint16_t)BNO055_I2C_ADDR1<<1,		// Адрес устройства
//																				0x2, 										// Адрес регистра
//																				I2C_MEMADD_SIZE_8BIT,		// Размерность данных
//																				IDData, 								// Буфер для получения данных
//																				4 	);									// Сколько данных нужно получить
																				
	
	if( HAL_OK != i2cRes )
	{
		while(1){}
		// BNO055_Error();
	}
	
	while( HAL_I2C_STATE_READY != HAL_I2C_GetState ( &hi2c1 ))
	{
		
	}
	
	//while(1){}
	
	// ******************* Попытки работы с камерой ***********************************************
	// Пробуем прочитать номер камеры
//	i2cRes = HAL_I2C_Mem_Read ( 				&hi2c1, 
//														(uint16_t)CAMERA_DEVICE_ADDRESS,  // Адрес устройства
//																			0x0A, 									// Адрес регистра
//																			I2C_MEMADD_SIZE_8BIT,		// Размерность данных
//																			IDData, 								// Буфер для получения данных
//																			1, 											// Сколько данных нужно получить
//																			100);
	
	//uint8_t Data = 0x0A;
//	i2cRes = HAL_I2C_Mem_Write_DMA ( 			&hi2c1, 
//															(uint16_t)0x43,												// Адрес устройства
//																				0x0A, 											// Адрес регистра
//																				I2C_MEMADD_SIZE_8BIT,				// Размерность данных
//																				IDData, 										// Буфер для получения данных
//																				2 	);											// Сколько данных нужно получить

//	
//	if( HAL_OK != i2cRes )
//	{
//		while(1){}
//		// BNO055_Error();
//	}
//	
//	while( HAL_I2C_STATE_READY != HAL_I2C_GetState ( &hi2c1 ))
//	{
//		
//	}
//	
//	while(1){}
	
	//*********************************************************************************************
	
	IDData[0] = OPERATION_MODE_NDOF;
	// Пробуем поменять режим устройства	
	i2cRes = HAL_I2C_Mem_Write( 					&hi2c1, 
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
	//  HAL_Delay(3000);
	

	// Пробуем поменять режим устройства	
//	i2cRes = HAL_I2C_Mem_Read( 					&hi2c3, 
//															(uint16_t)BNO055_I2C_ADDR1<<1,		// Адрес устройства
//																				BNO055_OPR_MODE_ADDR, 	// Адрес регистра
//																				8, 											// Размерность данных
//																				&IDData[1], 						// Буфер для получения данных
//																				1, 											// Сколько данных нужно отправить
//																				1000);
//	
//	if( HAL_OK != i2cRes )
//	{
//		BNO055_Error();
//	}
	
	// BNO055_GRAVITY_DATA_X_LSB_ADDR
	// BNO055_QUATERNION_DATA_W_LSB_ADDR
	
		// Пробуем читать ID устройства
		i2cRes = HAL_I2C_Mem_Read ( 				&hi2c1, 
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
		i2cRes = HAL_I2C_Mem_Read ( 				&hi2c1, 
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
		i2cError = HAL_I2C_GetError ( &hi2c1 );
		while(1){}
}
	}
	
		// Пробуем читать ID устройства
		i2cRes = HAL_I2C_Mem_Read ( 				&hi2c1, 
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
		i2cError = HAL_I2C_GetError ( &hi2c1 );
		while(1){}
}
	}
	
	// Отправить пакет по USB
	int8_t res = CDC_Transmit_FS( (uint8_t*)testDataToSend, 2*8 );
}

void BNO055_Error()
{	
	// Уходим в бесконечный цикл
	i2cError = HAL_I2C_GetError ( &hi2c1 );
	while(1){}
}

// Прерывание по окончантю передачи
void HAL_I2C_MemTxCpltCallback ( I2C_HandleTypeDef *hi2c )
{

}

// Прерывание по окончантю приема
void HAL_I2C_MemRxCpltCallback ( I2C_HandleTypeDef *hi2c )
{

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
