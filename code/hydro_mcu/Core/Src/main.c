/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  * watermelon sugar 
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
uint32_t is_pump_on;

/* USER CODE BEGIN Includes */
void GPIO_init();
void ADC_init();
void Turn_On_Pump();
void Turn_Off_Pump();
void Check_Water_Level();
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

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();
	
	GPIO_init();

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	uint32_t debouncer = 0;
	is_pump_on = 0;
  while (1){
		debouncer = (debouncer << 1); // Always shift every loop iteration
		if (GPIOA->IDR & 1) {     // If input signal is set/high
				debouncer |= 0x01;  // Set lowest bit of bit-vector
    }
		
		if (debouncer == 0x7FFFFFFF) {
			if (is_pump_on) {
				Turn_Off_Pump();
			}
			else {
				Turn_On_Pump();
			}
		}
		
		HAL_Delay(10);
		
		Check_Water_Level();
  }
  /* USER CODE END 3 */
}

// Helper to init GPIOs
void GPIO_init() {
	// Enable GPIOA, GPIOB, and GPIOC clocks
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; 
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	
	// Initialize PA0 for button input
	GPIOA->MODER &= ~(GPIO_MODER_MODER0_0 | GPIO_MODER_MODER0_1);               // Set PA0 to input
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR0_0 | GPIO_OSPEEDR_OSPEEDR0_1);     // Set to low speed
	GPIOC->PUPDR |= GPIO_PUPDR_PUPDR0_1;                                        // Set to pull-down
	
	// Set GPIOB and GPIOC to output mode
	GPIOB->MODER |= (1<<14)|(1<<12)| (1<<10)| (1<<8);
	GPIOC->MODER |= (1<<12) | (1<<14) | (1<<16) | (1<<18);
}

// A helper method to set up the ADC to pin PC0
void ADC_init (void) {
	// Set up ADC clock
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
	
  // Set up the MODER register to Analog mode
  GPIOC->MODER |= ((1 << 1) | (1 << 0));

  // Set PUPDR register to no pull-up no pull-down
  GPIOC->PUPDR &= ~((1 << 1) | (1 << 0));

  // Set ADC configure register to 8-bit resolution, continuous conversion mode, and hardware
	// triggers diabled.
	ADC1->CFGR1 |= ((1 << 13) | (1 << 4));
	ADC1->CFGR1 &= ~((1 << 11) | (1 << 10) | (1 << 3));
	
	// Set ADC channel selection register to ADC_IN10 ie PC0
	ADC1->CHSELR |= (1 << 10);
		
	// Calibrate the ADC
	ADC1->CR |= (1 << 31);
	
	// Wait for calibration to finish
	while (ADC1->CR & ADC_CR_ADCAL)
	{		
	}
	
	// Enable the ADC
	ADC1->CR |= (1 << 0);
	
	// Wait for the ADC ready flag
	while (!(ADC1->ISR & ADC_ISR_ADRDY))
	{
	}		
	
	// Start the ADC
	ADC1->CR |= (1 << 2);
} 

// Helper to turn on pump
void Turn_On_Pump(){
	//this is the GPIO pin used here.
	GPIOB->ODR &= ~(1<<4);
	
	//this is to see the code working.
	//GPIOC->ODR &= ~(1<<6);
	
	// Set pump value true.
	is_pump_on = 1;
}

// Helper to turn off pump
void Turn_Off_Pump(){
	//this is the GPIO pin used.
	GPIOB->ODR |= (1<<4);
	
	//this is to see the code working.
	//GPIOC->ODR |= (1<<6);
	
	// Set pump value false;
	is_pump_on = 0;
}

void Check_Water_Level() {
	if (ADC1->DR > 1) {
		GPIOC->ODR |= (1<<6);
		GPIOC->ODR &= ~(1<<7);
	}
	
	if (ADC1->DR < 1) {
		GPIOC->ODR |= (1<<7);
		GPIOC->ODR &= ~(1<<6);
	}
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
