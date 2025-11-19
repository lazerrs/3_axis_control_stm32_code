/* USER CODE BEGIN Header */
/**3axis step motor  control v 1.1
 Created on:mehr,1404
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
    * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.

  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "lcd.h"

#include "motor_control.h"
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
volatile uint32_t adcValues[4];
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  HAL_Delay(200);


  char lcd0[5];
  char lcd2[20];
  uint32_t lastLCDUpdate = 0;


  // [0] = PA4 , [1] = PA7
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  /*-[ I2C Bus Scanning ]-*/
  HAL_TIM_Base_Start_IT(&htim1);

  HAL_TIM_Base_Start_IT(&htim2);
  LCD_Init();
        LCD_Send_Command(0x01); // پاک کردن LCD
        LCD_Set_Cursor(0, 0);
        LCD_Send_String( "K.S.A 3axis control ");






      HAL_Delay(2000);
      LCD_Send_Command(0x01);
      LCD_Set_Cursor(0, 0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


     // HAL_ADC_Start(&hadc1); // شروع ADC

			  LCD_Send_Command(0x01); // پاک کردن LCD
			  LCD_Set_Cursor(0, 0);
			  LCD_Send_String("HOMING .......");
			  MotorControl_StepMotor_homing(1);



			  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			  HAL_ADCEx_Calibration_Start(&hadc1);
			  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcValues, 4);
  while (1)
  {
	 // Read_ADC();
		HAL_Delay(100);
	  LCD_Send_Command(0x01); // پاک کردن LCD
	  	         float voltage_pa4 = adcValues[0];
	  	       float voltage_pa5 = adcValues[1];
	  	     float voltage_pa6 = adcValues[2];
	  	     float voltage_pa7 = adcValues[3];
	  	   uint8_t msg[] = "***";
	  	   HAL_UART_Transmit(&huart1, msg, sizeof(msg), 10000);
	  MotorControl_UpdateSpeed(voltage_pa7,voltage_pa6);

	         if(HAL_GetTick() - lastLCDUpdate > 100)
	         {
	            lastLCDUpdate = HAL_GetTick();
	             MotorControl_LCDUpdate();
	        }

	         sprintf(lcd0,"%.0f", voltage_pa4);
	         LCD_Set_Cursor(0, 0);
	         LCD_Send_String(lcd0);
	         HAL_UART_Transmit(&huart1,(uint8_t*) lcd0,strlen(lcd0), 10000);
	         HAL_UART_Transmit(&huart1, msg, sizeof(msg), 10000);
	         sprintf(lcd2,"%.0f", voltage_pa5);

	         	         LCD_Set_Cursor(1, 1);
	         	         LCD_Send_String(lcd2);
	         	        HAL_UART_Transmit(&huart1,(uint8_t*) lcd2, strlen(lcd0), 10000);
	         	       HAL_UART_Transmit(&huart1, msg, sizeof(msg), 10000);
	         	      HAL_UART_Transmit(&huart1, "a6= ", sizeof("a6= "), 10000);
	         	        sprintf(lcd2,"%.0f", voltage_pa6);
	         	       LCD_Set_Cursor(2, 2);
	         	       	         	         LCD_Send_String(lcd2);
	         	       	         	    HAL_UART_Transmit(&huart1,(uint8_t*) lcd2, strlen(lcd2), 10000);
	         	       	         	 HAL_UART_Transmit(&huart1, msg, sizeof(msg), 10000);
	         	       	         HAL_UART_Transmit(&huart1, "a7= ", sizeof("a7= "), 10000);
	         	       	         	    sprintf(lcd2,"%.0f", voltage_pa7);
	         	       	         	   	         	       LCD_Set_Cursor(3, 3);
	         	       	         	   	         	       	         	         LCD_Send_String(lcd2);
	         	       	         	   	         	       	         HAL_UART_Transmit(&huart1,(uint8_t*) lcd2, strlen(lcd2), 10000);
	         	       	         	   	         	        HAL_UART_Transmit(&huart1, msg, sizeof(msg), 10000);
	         	       	         	   	         	        HAL_UART_Transmit(&huart1, msg, sizeof(msg), 10000);






          HAL_Delay(500);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if(htim->Instance == TIM2) {

    	 if( motorEnable==1)
    	 {if(HAL_GPIO_ReadPin(LIMIT_UP_GPIO_Port, LIMIT_UP_Pin) == GPIO_PIN_RESET &&
    	           HAL_GPIO_ReadPin(DIR_GPIO_Port, DIR_Pin) == GPIO_PIN_SET)
    	        {
    	            return;
    	        }

    	        if(HAL_GPIO_ReadPin(LIMIT_DOWN_GPIO_Port, LIMIT_DOWN_Pin) == GPIO_PIN_RESET &&
    	           HAL_GPIO_ReadPin(DIR_GPIO_Port, DIR_Pin) == GPIO_PIN_RESET)
    	        {
    	            return;
    	        }
    	MotorControl_TIMCallback();
    	 }
    }
    if(htim->Instance == TIM1) {

       	 if( motorEnable_2==1)
       	 {if(HAL_GPIO_ReadPin(LIMIT_UP_GPIO_Port_2, LIMIT_UP_Pin_2) == GPIO_PIN_RESET &&
       	           HAL_GPIO_ReadPin(DIR_GPIO_Port_2, DIR_Pin_2) == GPIO_PIN_SET)
       	        {
       	            return;
       	        }

       	        if(HAL_GPIO_ReadPin(LIMIT_DOWN_GPIO_Port_2, LIMIT_DOWN_Pin_2) == GPIO_PIN_RESET &&
       	           HAL_GPIO_ReadPin(DIR_GPIO_Port_2, DIR_Pin_2) == GPIO_PIN_RESET)
       	        {
       	            return;
       	        }
       	MotorControl_TIMCallback_2();
       	 }
       }
}





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
