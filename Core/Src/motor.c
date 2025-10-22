/*
 * motor.c
 *
 *  Created on: Jun 17, 2025
 *      Author: ms
 */

#include "motor.h"
#include "main.h"
#include <stdint.h>
#include "lcd.h"

#define STEP_PIN GPIO_PIN_0
#define STEP_PORT GPIOB
#define DIR_PIN GPIO_PIN_1
#define DIR_PORT GPIOB
#define UP_MS_PIN GPIO_PIN_9
#define UP_MS_PORT GPIOB
#define DW_MS_PIN GPIO_PIN_8
#define DW_MS_PORT GPIOB
void StepMotor_SendPulses(uint32_t steps)
{
	char buf[4];
    for (uint32_t i = 0; i < steps; i++) {
    	LCD_Set_Cursor(1, 0);
    buf[0]='0'+i/1000;
    buf[1]='0'+(i/100)%10;
   buf[2]='0'+(i/10)%10;
    buf[3]='0'+(i)%10;

    	LCD_Send_String(buf);
        HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_SET);
        HAL_Delay(1); // یا use microsecond delay if needed

        HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_RESET);
        HAL_Delay(1);
    }

}
void StepMotor_homing(uint32_t speed)
{
	uint8_t confirmed = 0;
	HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, GPIO_PIN_SET);
	while (!confirmed){
		if (HAL_GPIO_ReadPin(DW_MS_PORT,DW_MS_PIN) == GPIO_PIN_RESET)  confirmed = 1;


	}

	HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, GPIO_PIN_RESET);
	HAL_Delay(20);








}
