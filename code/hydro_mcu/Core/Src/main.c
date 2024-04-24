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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

//-------------------------------------
//---------Current Pins Used-----------
// ADC -> PC1
// Push Button -> PA0
// Pump -> PB4
// Lighting -> PB5
// LEDS -> PC9, PC8, PC7, PC6
// Unused but enabled -> PB6, PB7
//-------------------------------------
//-------------------------------------


// Static Variables
static char is_pump_on;
static char timer_state;

// Function Prototypes
void SystemClock_Config(void);
void Next_State();
void Transmit_String(char* input);
void Transmit_Char(char input);
void GPIO_init();
void LED_init();
void USART_init();
void Timer_init(uint16_t reload_val);
void ADC_init();
void Turn_On_Pump();
void Turn_Off_Pump();
void Check_Water_Level();

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
	// ----Begin Init----
  HAL_Init();
  SystemClock_Config();
	GPIO_init(); // USING PINS ---
	ADC_init(); // USING PINS ---
	
	// Enable the interupt for Timer2
	NVIC_EnableIRQ(TIM2_IRQn);

	uint32_t debouncer = 0;
	is_pump_on = 0;
	timer_state = 0;
	Turn_Off_Pump();
	
	// Begin inifite loop
  while (1){
		
		//GPIOB->ODR |= (1 << 5);   
		// Debouncer for toggling lighting state.
		debouncer = (debouncer << 1); // Always shift every loop iteration
		if (GPIOA->IDR & 1) {     // If input signal is set/high
				debouncer |= 0x01;  // Set lowest bit of bit-vector
    }
		
		if (debouncer == 0x7FFFFFFF) {
			Next_State();
			/*
			if (is_pump_on) {
				Turn_Off_Pump();
			}
			else {
				Turn_On_Pump();
			}
			*/
		}
		
  }
  /* USER CODE END 3 */
}

// Method that will handle state changes. Based on the value in timer_state. It will increment
// state until they are cycled through coming back to state 0. State 0 is lights off, and state
// 4 is lights always on. Will also turn on pump whenever lighting is enabled. Pump will shut 
// off when lighting is not enabled.
void Next_State() {
		switch (timer_state) {
			// Case 0 everything is off
			case 0:
				Turn_On_Pump();
				timer_state = 1;
				Timer_init(1000);
				break;
			// Case 1 light timer will toggle every 5 sec
			case 1:
				timer_state = 2;
				GPIOB->ODR |= (1 << 5);
				//TIM2->ARR = 0;
				//TIM2->CR1 &= ~TIM_CR1_UDIS;
				//TIM2->EGR = TIM_EGR_UG;
				//TIM1->CR1 |= TIM_CR1_UDIS;
			
				TIM2->ARR = 0;
				TIM2->CR1 |= TIM_CR1_UDIS;

				TIM2->EGR = TIM_EGR_UG;
				TIM2->CR1 &= ~TIM_CR1_UDIS;
			TIM2->CR1 |= TIM_CR1_URS;
			
			
			
				Timer_init(2000);
				break;
			// Case 2 light timer will toggle every 10 sec
			case 2:
				timer_state = 3;
				GPIOB->ODR |= (1 << 5);
				Timer_init(3000);
				break;
			// Case 3 light timer will toggle every 15 sec
			case 3:
				// This will be different. Timer needs to be disabled and LED turned on.
				timer_state = 4;
				TIM2->CR1 &= ~(1 << 0);
				GPIOB->ODR |= (1 << 5);
				break;
			// Case 4 lights will stay on
			case 4:
				// Only difference will be that LED turns off.
				Turn_Off_Pump();
				timer_state = 0;
				GPIOB->ODR &= ~(1 << 5);
				break;
		}
}

// Helper function for transmitting one character through USART
void Transmit_Char(char input) {
	// An empty while loop that breaks through when the status register
	// says the transmit data register is empty.
	while(!(USART3->ISR & (1 << 7))) {
			
	}
	
	// Write the input to the transfer data register
	USART3->TDR = input;
}

// Helper function to transmit a string through USART
void Transmit_String(char* input) {
	int counter = 0;
	
	// Loop through and transmit the string.
	while(*(input + counter) != 0) {
		Transmit_Char(*(input + counter));
		counter = counter + 1;
	}
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

// Helper for LED initialization
void LED_init() {
	// First enable clock for GPIOC
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	
	// Set up MODER registers to general purpose output for PC6, PC7,
	// PC8, and PC9, the LEDs
	GPIOC->MODER |= ((1 << 18) | (1 << 16) | (1 << 14) | (1 << 12));
	GPIOC->MODER &= ~((1 << 19) | (1 << 17) | (1 << 15) | (1 << 13));
	
	// Set up OTYPER registers to be in push-pull mode
	GPIOC->OTYPER &= ~((1 << 9) | (1 << 8) | (1 << 7) | (1 << 6));
	
	// Set up OSPEEDR registers to low speed mode
	GPIOC->OSPEEDR &= ~((1 << 18) | (1 << 16) | (1 << 14) | (1 << 12));
	
	// Set up PUPDR registers to no pull up / no pull down resistors
	GPIOC->PUPDR &= ~((1 << 19) | (1 << 18) | (1 << 17) | (1 << 16));
	GPIOC->PUPDR &= ~((1 << 15) | (1 << 14) | (1 << 13) | (1 << 12));
}

// Helper for USART initilization PC4 = TX and PC5 = RX
void USART_init(void) {
	// First enable clock for GPIOC
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	
	// Set both PC4 and PC5 to Alternate Function Mode 
	GPIOC->MODER |= ((1 << 11) | (1 << 9));
	GPIOC->MODER &= ~((1 << 10) | (1 << 8));
	
	// Select the AF1 alternate function for PC4 and PC5
	GPIOC->AFR[0] |= ((1 << 20) | (1 << 16));
	
	// Enable clock for USART3
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	
	// Set the baud rate to 115200 bits / second. Clock is 8Mhz.
	USART3->BRR = 8000000 / 115200;
	
	// Enable Transmitter and Reciever hardware
	USART3->CR1 |= ((1 << 3) | (1 << 2));
	
	// Enable the USART
	USART3->CR1 |= (1 << 0);
	
	// Enable interupts
	USART3->CR1 |= (1 << 5);
	
	// Enable USART interupt in the NVIC
	NVIC_EnableIRQ(USART3_4_IRQn);
}

// Heleper for Timer2 init
void Timer_init(uint16_t reload_val) {
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	
	// Set the count back to 0.
	TIM2->CNT = 0;
	
	// Enable PSC register to 39,999 which will give us 5ms ticks
	TIM2->PSC = 39999;
	
	// Values for this code are 1k = 5 sec, 2k = 10 sec, 3k = 15 sec.
	TIM2->ARR = reload_val;
	
	
	// Enable DIER register to UIE(Update interrupt enable)
	TIM2->DIER |= 1;
	
	// Enable the CNT or configuration register of the timer
	TIM2->CR1 &= ~((1 << 9) | (1 << 8) | (1 << 6) | (1 << 5) | (1 << 4) | (1 << 3) | (1	<< 1));
	TIM2->CR1 |= ((1 << 7) | (1 << 2) | (1 << 0));
}

// A helper method to set up the ADC to pin PC1
void ADC_init (){
    // Enabling clock
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
	
    //Setting Mode to Analog mode this is PC1(3:2) to value 11 Analog mode
		GPIOC->MODER |= (1<<3) | (1<<2);
	
    // Setting PUDR to no pull-up/down this is for PC1(3:2) value of 00 in the reg.
    GPIOC->PUPDR &= ~((1<<3)|(1<<2));
	
    //ADC CFGR1 still used ADC Bit 13 (1 enables Continous), BIT (4:3) (sets resoultion)
    ADC1->CFGR1 |= (1<<13) | (1<<4);
	
    //bits 11:10 are the address the values should be 00 not to have Triggers.
    ADC1->CFGR1 &= ~((1<<10) | (1<<11));
	
    //Enabling ADC11 with bit 11 (1 value enables)
    ADC1->CHSELR |= (1 << 11);
	
    //Start ADC calibration this should be bit 31 of the CR1 regiester
    ADC1->CR |= (1<<31);
		
    //Wait for calibration
    while((ADC1->CR & (1<<31))){}
			
    //Enable ADC everything using bit 0 enable in the CR 1 is what sets it
    ADC1->CR |= (1<<0);
			
    while(!(ADC1->ISR & (1<<0))){}
    //this starts the conversion
    //this is in bit 2
    // and this 1 enables it 0 disables
    ADC1->CR |= (1<<2);
    //getting the val from the ADC
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
	GPIOB->ODR |=(1<<4);
	
	//this is to see the code working.
	//GPIOC->ODR |= (1<<6);
	
	// Set pump value false;
	is_pump_on = 0;
}

// TODO: Needs much work and is not correct
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
