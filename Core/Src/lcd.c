#include "lcd.h"
#include "main.h"
#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdlib.h>

void LCD_Enable(void) {
    HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
}

void LCD_Send_4Bits(uint8_t data) {
    HAL_GPIO_WritePin(LCD_D4_GPIO_Port, LCD_D4_Pin, (data >> 0) & 0x01);
    HAL_GPIO_WritePin(LCD_D5_GPIO_Port, LCD_D5_Pin, (data >> 1) & 0x01);
    HAL_GPIO_WritePin(LCD_D6_GPIO_Port, LCD_D6_Pin, (data >> 2) & 0x01);
    HAL_GPIO_WritePin(LCD_D7_GPIO_Port, LCD_D7_Pin, (data >> 3) & 0x01);
    LCD_Enable();
}

void LCD_Send_Command(uint8_t cmd) {
    HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET);
    LCD_Send_4Bits(cmd >> 4);
    LCD_Send_4Bits(cmd & 0x0F);
    HAL_Delay(2);
}

void LCD_Send_Data(uint8_t data) {
    HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);
    LCD_Send_4Bits(data >> 4);
    LCD_Send_4Bits(data & 0x0F);
    HAL_Delay(1);
}

void LCD_Send_String(char *str) {
    while (*str) {
        LCD_Send_Data((uint8_t)(*str));
        str++;
    }
}

void LCD_Set_Cursor(uint8_t row, uint8_t col) {
    uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
    LCD_Send_Command(0x80 | (col + row_offsets[row]));
}

void LCD_Init(void) {
    HAL_Delay(50);
    LCD_Send_4Bits(0x03); HAL_Delay(5);
    LCD_Send_4Bits(0x03); HAL_Delay(1);
    LCD_Send_4Bits(0x03); HAL_Delay(1);
    LCD_Send_4Bits(0x02); // 4-bit mode

    LCD_Send_Command(0x28); // 4-bit, 2 line, 5x8
    LCD_Send_Command(0x0C); // Display on, cursor off
    LCD_Send_Command(0x06); // Entry mode
    LCD_Send_Command(0x01); // Clear display
    HAL_Delay(2);
}
