/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	HAL_Init();
  SystemClock_Config();	
	
	
	// Enabling the timer, enable peripheral clock of GPIOB
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	// Enabling the timer, enable peripheral clock of GPIOC
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	
	// Enabling the timer, enable peripheral clock of I2C2
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	
	
	
	
	// Set PB11 to alternate function mode(I2C2_SDA), open-drain output type
	GPIOB->MODER &= ~(1 << 22);
	GPIOB->MODER |= (1 << 23);
	
	GPIOB->OTYPER |= (1 << 11);
	
	GPIOB->AFR[1] |= (1 << 12);
	GPIOB->AFR[1] &= ~((1 << 13) | (1 << 14) | (1 << 15));
	
	
	// Set PB13 to alternate function mode(I2C2_SCL), open-drain output type
	GPIOB->MODER &= ~(1 << 26);
	GPIOB->MODER |= (1 << 27);
	
	GPIOB->OTYPER |= (1 << 13);
	
	GPIOB->AFR[1] |= ((1 << 20) | (1 << 22));
	GPIOB->AFR[1] &= ~((1 << 21) | (1 << 23));
	
	
	// Set PB14 to output mode, push-pull output type, and initialize/set the pin high
	GPIOB->MODER |= (1 << 28);
	GPIOB->MODER &= ~(1 << 29);
	
	GPIOB->OTYPER &= ~(1 << 14);
	
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	
	
	// Set PC0 to output mode, push-pull output type, and initialize/set the pin high
	GPIOC->MODER |= (1 << 0);
	GPIOC->MODER &= ~(1 << 1);
	
	GPIOC->OTYPER &= ~(1 << 0);
	
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
	
	
	// Initializing GPIO Pins for LED: PC6-PC9
	// General-purpose output mode using the MODER register
	GPIOC->MODER |= (1 << 12) | (1 << 14) | (1 << 16) | (1 << 18);
	GPIOC->MODER &= ~((1 << 13) | (1 << 15) | (1 << 17) | (1 << 19));
	
	// Push-pull output type using the OTYPER register
	GPIOC->OTYPER &= ~((1 << 6) | (1 << 7) | (1 << 8) | (1 << 9));
	
	// Low speed using the OSPEEDR register
	GPIOC->OSPEEDR &= ~((1 << 12) | (1 << 14) | (1 << 16) | (1 << 18));

	// No pull-up/down resistors using the PUPDR register
	GPIOC->PUPDR &= ~((1 << 12) | (1 << 13) | (1 << 14) | (1 << 15) | (1 << 16) | (1 << 17) | (1 << 18) | (1 << 19));
	
	
	
	
	// Set TIMINGR register to use 100kHz standard-mode I2C for I2C2
	// PRESC = 1
	I2C2->TIMINGR |= (1 << 28);
	// SCLL = 0x13
	I2C2->TIMINGR |= (0x13 << 0);
	// SCLH = 0xF
	I2C2->TIMINGR |= (0xF << 8);
	// SDADEL = 0x2
	I2C2->TIMINGR |= (0x2 << 16);
	// SCLDEL = 0x4
	I2C2->TIMINGR |= (0x4 << 20);
	
	
	// Enable I2C2 peripheral
	I2C2->CR1 |= (1 << 0);
	
	
	/*
	//First checkoff START
	
	
	// Set the transaction parameters in the CR2 register
	// Set the L3GD20 slave address = 0x69
	I2C2->CR2 |= (0x69 << 1);
	
	// Set the number of bytes to transmit, 1
	I2C2->CR2 |= (1 << 16);
	
	// Set the RD_WRN bit to indicate a "write" operation
	I2C2->CR2 &= ~(1 << 10);
	
	// Set the START bit
	I2C2->CR2 |= (1 << 13);

	
	// TXIS: Transmit Register Empty/Ready
	// NACKF: Slave NotAcknowledge
	// Wait until I2C2 "TXIS" is set(= 1)
	// Red LED will toggle when NACKF is set, indicating an error
	while (1) {
		// Blink Green LED if TXIS is set & break
		if (I2C2->ISR & I2C_ISR_TXIS){
			GPIOC->ODR |= (1 << 9);
			HAL_Delay(500);
			GPIOC->ODR &= ~(1 << 9);
			HAL_Delay(500);
			break;	// continue
		}
		
		// Blink Red LED if NACKF is set
		if (((I2C2->ISR & I2C_ISR_NACKF) == 0)) {
			GPIOC->ODR |= (1 << 6);
			HAL_Delay(500);
			GPIOC->ODR &= ~(1 << 6);
			HAL_Delay(500);
		}
	} 

	
	
	
	// Set the address for "WHO_AM_I" register into the I2C transmit register using TXDR
	//I2C2->TXDR |= (0x0F << 0);
	I2C2->TXDR = 0x0F;
	
	
	// TC: Transfer Complete
	// Wait until I2C2 "TC" flag is set(= 1), TC = Transfer Complete
	// Red LED will toggle when NACKF is set, indicating an error
	while (1) {
		// Blink Green LED if TC is set & break
		if (I2C2->ISR & I2C_ISR_TC){
			GPIOC->ODR |= (1 << 9);
			HAL_Delay(500);
			GPIOC->ODR &= ~(1 << 9);
			HAL_Delay(500);
			break;	// continue
		}
		
		// Blink Red LED if NACKF is set
		if (((I2C2->ISR & I2C_ISR_NACKF) == 0)) {
			GPIOC->ODR |= (1 << 6);
			HAL_Delay(500);
			GPIOC->ODR &= ~(1 << 6);
			HAL_Delay(500);
		}
	}
	
	
	
	
	// Reload CR2 register with same parameters, but set RD_WRN bit to indicate a read operation
	// Set the L3GD20 slave address = 0x69
	I2C2->CR2 |= (0x69 << 1);
	
	// Set the number of bytes to transmit, 1
	I2C2->CR2 |= (1 << 16);
	
	// Set the RD_WRN bit to indicate a "read" operation
	I2C2->CR2 |= (1 << 10);
	
	// Set the START bit *** needed for to perform a I2C restart condition!
	I2C2->CR2 |= (1 << 13);
	
	
	// RXNE: Receive Register Not Empty
	// Wait until I2C2 "RXNE" flag is set(= 1)
	// Red LED will toggle when NACKF is set, indicating an error
	while (1) {
		// Blink Green LED if RXNE is set & break
		if (I2C2->ISR & I2C_ISR_RXNE){
			GPIOC->ODR |= (1 << 9);
			HAL_Delay(500);
			GPIOC->ODR &= ~(1 << 9);
			HAL_Delay(500);
			break;	// continue
		}
		
		// Blink Red LED if NACKF is set
		if (((I2C2->ISR & I2C_ISR_NACKF) == 0)) {
			GPIOC->ODR |= (1 << 6);
			HAL_Delay(500);
			GPIOC->ODR &= ~(1 << 6);
			HAL_Delay(500);
		}
	}
	// Wait until I2C2 "TC" flag is set(= 1)
	// Red LED will toggle when NACKF is set, indicating an error
	while (1) {
		// Blink Green LED if TC is set & break
		if (I2C2->ISR & I2C_ISR_TC){
			GPIOC->ODR |= (1 << 9);
			HAL_Delay(500);
			GPIOC->ODR &= ~(1 << 9);
			HAL_Delay(500);
			break;	// continue
		}
		
		// Blink Red LED if NACKF is set
		if (((I2C2->ISR & I2C_ISR_NACKF) == 0)) {
			GPIOC->ODR |= (1 << 6);
			HAL_Delay(500);
			GPIOC->ODR &= ~(1 << 6);
			HAL_Delay(500);
		}
	}
	
	
	// Check contents of RXDR register to see if it matches 0xD3(=value of the "WHO_AM_I" register)
	// Blue LED will toggle when 
	if (I2C2->RXDR == 0xD3) {
			GPIOC->ODR |= (1 << 7);
			HAL_Delay(500);
			GPIOC->ODR &= ~(1 << 7);
			HAL_Delay(500);
	}
	
	
	// Set the STOP bit
	I2C2->CR2 |= (1 << 14);
	
	
	// First checkoff END
	*/
	
	
	
	
	//Second checkoff START
	// See Figure 5.7 for steps for second checkoff, implementing a rotation indicator using the Gyroscope
	
	
	// 1. Register Write - this step enables the Gyroscope
	// Setup "write" to L3GD20 address
	// Set the L3GD20 slave address = 0x69
	I2C2->CR2 |= (0x69 << 1);
	
	// Set the number of bytes to transmit, 2
	I2C2->CR2 &= ~(1 << 16);
	I2C2->CR2 |= (1 << 17);
	
	// Set the RD_WRN bit to indicate a "write" operation
	I2C2->CR2 &= ~(1 << 10);
	
	// Set the START bit
	I2C2->CR2 |= (1 << 13);
	
	
	// Wait until I2C2 "TXIS" is set(= 1)
	while (1) {
		if (I2C2->ISR & I2C_ISR_TXIS){
			break;	// continue
		}
	}
	
	
	// Set the address for "CTRL_REG1" register into the I2C transmit register using TXDR
	I2C2->TXDR = 0x20;
	
	
	// Wait until I2C2 "TXIS" is set(= 1)
	while (1) {
		if (I2C2->ISR & I2C_ISR_TXIS){
			break;	// continue
		}
	}
	
	
	// Transfer rest of the address
	I2C2->TXDR = 0xB;
	
	
	// Wait until I2C2 "TC" flag is set(= 1), TC = Transfer Complete
	while (1) {
		if (I2C2->ISR & I2C_ISR_TC){
			break;	// continue
		}
	}	
	
	
	// Set the STOP bit
	I2C2->CR2 |= (1 << 14);
	
	
	
	
	// 2. Register Read - this step reads/saves the X, Y axis data every 100ms
	// Since the board needs to update data every 100ms, main infinite loop is used
	// Declare variables for x, y axis datas
	int8_t x_1;
	int8_t x_2;
	int8_t y_1;
	int8_t y_2;
	int16_t x_axis;
	int16_t y_axis;
	
	
	

  while (1)
  {
		HAL_Delay(100); // Give delay every 100ms
		
		
		// X axis data --------------------
		
		// Set the number of bytes to transmit, 1 ***(Send both high, low data at one time)***
		I2C2->CR2 |= (1 << 16);
		I2C2->CR2 &= ~(1 << 17);
		// Set the RD_WRN bit to indicate a "write" operation
		I2C2->CR2 &= ~(1 << 10);
		// Set the START bit
		I2C2->CR2 |= (1 << 13);
		
		// Wait until TXIS flag is set(= 1)
		while (1) {
		if (I2C2->ISR & I2C_ISR_TXIS){
			break;	// continue
			}
		}
		
		// First, write both x axis datas in one transaction
		I2C2->TXDR = 0xA8;
		
		// Wait until TC flag is set(= 1), transaction to be completed
		while (1) {
		if (I2C2->ISR & I2C_ISR_TC){
			break;	// continue
			}
		}
		
		// Read the x-axis datas: Data Low Bytes & Data High Bytes
		// Set the number of bytes to transmit, 2 ***(Recieve high, low data separately)***
		I2C2->CR2 &= ~(1 << 16);
		I2C2->CR2 |= (1 << 17);
		// Set the RD_WRN bit to indicate a "read" operation
		I2C2->CR2 |= (1 << 10);
		// Set the START bit
		I2C2->CR2 |= (1 << 13);
		
		// Wait until RXNE flag is set(= 1), indicating recieve register not empty
		while (1) {
		if (I2C2->ISR & I2C_ISR_RXNE){
			break;	// continue
			}
		}
		
		// save first data to x_1 variable (LOW)
		x_1 = I2C2->RXDR;
		
		// Wait until RXNE flag is set(= 1), indicating recieve register not empty
		while (1) {
		if (I2C2->ISR & I2C_ISR_RXNE){
			break;	// continue
			}
		}		
		
		// save second data to x_2 variable (HIGH)
		x_2 = I2C2->RXDR;
		
		// Wait until TC flag is set(= 1), transaction to be completed
		while (1) {
		if (I2C2->ISR & I2C_ISR_TC){
			break;	// continue
			}
		}
		
		// Set the STOP bit, finished updating x_axis data
		I2C2->CR2 |= (1 << 14);
		
		
		
		
		// Y axis data --------------------
		
		// Set the number of bytes to transmit, 1 ***(Send both high, low data at one time)***
		I2C2->CR2 |= (1 << 16);
		I2C2->CR2 &= ~(1 << 17);
		// Set the RD_WRN bit to indicate a "write" operation
		I2C2->CR2 &= ~(1 << 10);
		// Set the START bit
		I2C2->CR2 |= (1 << 13);
		
		// Wait until TXIS flag is set(= 1)
		while (1) {
		if (I2C2->ISR & I2C_ISR_TXIS){
			break;	// continue
			}
		}
		
		// First, write both x axis datas in one transaction
		I2C2->TXDR = 0xAA;
		
		// Wait until TC flag is set(= 1), transaction to be completed
		while (1) {
		if (I2C2->ISR & I2C_ISR_TC){
			break;	// continue
			}
		}
		
		// Read the x-axis datas: Data Low Bytes & Data High Bytes
		// Set the number of bytes to transmit, 2 ***(Recieve high, low data separately)***
		I2C2->CR2 &= ~(1 << 16);
		I2C2->CR2 |= (1 << 17);
		// Set the RD_WRN bit to indicate a "read" operation
		I2C2->CR2 |= (1 << 10);
		// Set the START bit
		I2C2->CR2 |= (1 << 13);
		
		// Wait until RXNE flag is set(= 1), indicating recieve register not empty
		while (1) {
		if (I2C2->ISR & I2C_ISR_RXNE){
			break;	// continue
			}
		}
		
		// save first data to y_1 variable (LOW)
		y_1 = I2C2->RXDR;
		
		// Wait until RXNE flag is set(= 1), indicating recieve register not empty
		while (1) {
		if (I2C2->ISR & I2C_ISR_RXNE){
			break;	// continue
			}
		}
		
		// save second data to y_2 variable (HIGH)
		y_2 = I2C2->RXDR;
		
		// Wait until TC flag is set(= 1), transaction to be completed
		while (1) {
		if (I2C2->ISR & I2C_ISR_TC){
			break;	// continue
			}
		}
		
		// Set the STOP bit, finished updating y_axis data
		I2C2->CR2 |= (1 << 14);
		
		
		
		
		// Save partial data into one variable: x_axis, y_axis
		x_axis = (uint16_t)((x_2 << 8) | x_1);
		y_axis = (uint16_t)((y_2 << 8) | y_1);
		
		// Set a threshold for tilting the device
		int32_t threshold = 5000;
		
		
		
		
		// LEDs will indicate the direction of the inclination of device
		if (x_axis > threshold) {
				GPIOC->ODR |= (1 << 9);
				GPIOC->ODR &= ~(1 << 8);
		}
		
		else if (x_axis < -threshold){
				GPIOC->ODR |= (1 << 8);
				GPIOC->ODR &= ~(1 << 9);
		}
		
		else if( y_axis >threshold){
				GPIOC->ODR |= (1 << 6);
				GPIOC->ODR &= ~(1 << 7);
		}
		
		else if( y_axis < -threshold){
				GPIOC->ODR |= (1 << 7);
				GPIOC->ODR &= ~(1 << 6);
		}
			
	}
	
	
	/* Infinite loop END */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
