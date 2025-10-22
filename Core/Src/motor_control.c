#include "motor_control.h"
#include "lcd.h"
#include "motor.h"
#include "main.h"
#include "adc.h"
#include <stdio.h>
#include "usart.h"
volatile int32_t stepCount = 0;

volatile uint8_t motorEnable = 0;
uint32_t adcValue = 0;
char lcd0[20],lcd3[20];
// خواندن جویستیک و تنظیم سرعت و جهت
void MotorControl_UpdateSpeed(uint32_t adc_val)
{



    adcValue =adc_val;

    if(adcValue > ADC_CENTER + DEAD_ZONE)
    {
        if(HAL_GPIO_ReadPin(LIMIT_UP_GPIO_Port, LIMIT_UP_Pin) == GPIO_PIN_RESET)
        {
            motorEnable = 0;
            HAL_TIM_Base_Stop_IT(&htim2);
            HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, GPIO_PIN_RESET);
            return;
        }

        motorEnable = 1;
        HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
        uint32_t period = MAX_PERIOD - ((adcValue - (ADC_CENTER + DEAD_ZONE)) * (MAX_PERIOD - MIN_PERIOD) / (4095 - (ADC_CENTER + DEAD_ZONE)));
        __HAL_TIM_SET_AUTORELOAD(&htim2, period);
        __HAL_TIM_SET_COUNTER(&htim2, 0);
        HAL_TIM_Base_Start_IT(&htim2);

    }
    else if(adcValue < ADC_CENTER - DEAD_ZONE)
    {
        if(HAL_GPIO_ReadPin(LIMIT_DOWN_GPIO_Port, LIMIT_DOWN_Pin) ==GPIO_PIN_RESET)
        {
            motorEnable = 0;
            HAL_TIM_Base_Stop_IT(&htim2);
            HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, GPIO_PIN_RESET);
            return;
        }

        motorEnable = 1;
        HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
        uint32_t period = MAX_PERIOD - (((ADC_CENTER - DEAD_ZONE) - adcValue) * (MAX_PERIOD - MIN_PERIOD) / (ADC_CENTER - DEAD_ZONE));
        __HAL_TIM_SET_AUTORELOAD(&htim2, period);
        __HAL_TIM_SET_COUNTER(&htim2, 0);
        HAL_TIM_Base_Start_IT(&htim2);

    }
    else
    {
        motorEnable = 0;
        HAL_TIM_Base_Stop_IT(&htim2);
        HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, GPIO_PIN_RESET);
    }



   // sprintf(lcd0,"%.3f",(float)period);
   // LCD_Send_String(lcd0);
}

// Timer interrupt → تولید پالس STEP و شمارش

 void MotorControl_TIMCallback()
{



    	HAL_GPIO_TogglePin(STEP_GPIO_Port, STEP_Pin);


        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);


        if(HAL_GPIO_ReadPin(STEP_GPIO_Port, STEP_Pin) == GPIO_PIN_SET)
        {
            if(HAL_GPIO_ReadPin(DIR_GPIO_Port, DIR_Pin) == GPIO_PIN_SET)
                stepCount++;
            else
                stepCount--;
        }
    }


// بروزرسانی LCD
void MotorControl_LCDUpdate(void)
{
	sprintf(lcd0,"%.3f",(float)stepCount);
				//  LCD_Set_Cursor(0, 0);
		 // LCD_Send_String("Position:");

				  LCD_Set_Cursor(1, 0);
    LCD_Send_String(lcd0);

               //  HAL_UART_Transmit(&huart1, lcd0, sizeof(lcd0), 10000);
}
void MotorControl_StepMotor_homing(uint32_t speed)
{
	uint8_t confirmed = 0;
	HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
	while (!confirmed){
		HAL_GPIO_TogglePin(STEP_GPIO_Port, STEP_Pin);

		if (HAL_GPIO_ReadPin(LIMIT_DOWN_GPIO_Port, LIMIT_DOWN_Pin) == GPIO_PIN_RESET)  confirmed = 1;

		HAL_Delay(5);
	}

	HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
	HAL_Delay(20);








}
