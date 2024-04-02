/**
  *
  * Brandon Mouser & Tyler Evans
  * U0962682, u1313811
  *
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
#include "stm32f0xx_hal.h"
void _Error_Handler(char * file, int line);
/*
 KEY Commands for terminal
 */

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/**
 @function turnOffPump1
 @brief turns off pump 1 using PB4.
 */
void turnOffPump1(){
    //this is the GPIO pin used.
    GPIOB->ODR |= (1<<4);
    //this is to see the code working.
    GPIOC->ODR |= (1<<6);
}
/**
 @function turnOnPump1
 @brief turns on pump 1 using PB4.
 */
void turnOnPump1(){
    //this is the GPIO pin used here.
    GPIOB->ODR &= ~(1<<4);
    //this is to see the code working.
    GPIOC->ODR &= ~(1<<6);
}

/**
 @function turnOffPump2
 @brief turns off pump 2 using PB5.
 */
void turnOffPump2(){
    //this is the GPIO pin used.
    GPIOB->ODR |= (1<<5);
    //this is to see the code working.
    GPIOC->ODR |= (1<<7);
}
/**
 @function turnOnPump2
 @brief turns on pump 2 using PB5.
 */
void turnOnPump2(){
    //this is the GPIO pin used here.
    GPIOB->ODR &= ~(1<<5);
    //this is to see the code working.
    GPIOC->ODR &= ~(1<<7);
}
/**
 @function turnOffPump3
 @brief turns off pump 3 using PB6.
 */
void turnOffPump3(){
    //this is the GPIO pin used.
    GPIOB->ODR |= (1<<6);
    //this is to see the code working.
    GPIOC->ODR |= (1<<8);
}
/**
 @function turnOnPump3
 @brief turns on pump 3 using PB6.
 */
void turnOnPump3(){
    //this is the GPIO pin used here.
    GPIOB->ODR &= ~(1<<6);
    //this is to see the code working.
    GPIOC->ODR &= ~(1<<8);
}

/**
 @function turnOffPump4
 @brief turns off pump 4 using PB7.
 */
void turnOffPump4(){
    //this is the GPIO pin used.
    GPIOB->ODR |= (1<<7);
    //this is to see the code working.
    GPIOC->ODR |= (1<<9);
}
/**
 @function turnOnPump4
 @brief turns on pump 4 using PB7.
 */
void turnOnPump4(){
    //this is the GPIO pin used here.
    GPIOB->ODR &= ~(1<<7);
    //this is to see the code working.
    GPIOC->ODR &= ~(1<<9);
}
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
//

/*
 useful things
 bit follows for the LED
 RED = 6
 BLUE = 7
 ORANGE = 8
 GREEN = 9
 */
int main(void)
{
    HAL_Init(); // Reset of all peripherals, init the Flash and Systick
    SystemClock_Config(); //Configure the system clock
    
    
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // done to find the clock
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // Enables GPIOB
    
    /*
     GPIO setup section.
     */
    // The following are for LED mode.
    //good for debug.
    //output mode of 01   PC9(19:18) PC8(16:17)     PC7(15:14)   PC6(13:12)
    GPIOC->MODER |= (1<<12) | (1<<14) | (1<<16) | (1<<18);
    // GPIO B output enable section
    //the following pins are used for pump contorl.
    //output mode of 01   PB7(15:14)    PB6(13:12)    PB5(11:10)    PB4(9:8)
    GPIOB->MODER |= (1<<14)|(1<<12)| (1<<10)| (1<<8);
    while(1) {
     //now this is where one will run the code repeatedly
            //turnOnPump1();
            //HAL_Delay(100);
            turnOffPump1();
            HAL_Delay(1000);
        turnOffPump2();
        
     }

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

/* USER CODE BEGIN 4 */

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
