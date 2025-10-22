#ifndef INC_LCD_H_
#define INC_LCD_H_

#include "main.h"

// پین‌ها را بر اساس اتصالات خودتان تغییر دهید
#define LCD_RS_Pin GPIO_PIN_7
#define LCD_RS_GPIO_Port GPIOB
#define LCD_E_Pin  GPIO_PIN_6
#define LCD_E_GPIO_Port GPIOB
#define LCD_D4_Pin GPIO_PIN_3
#define LCD_D4_GPIO_Port GPIOA
#define LCD_D5_Pin GPIO_PIN_2
#define LCD_D5_GPIO_Port GPIOA
#define LCD_D6_Pin GPIO_PIN_1
#define LCD_D6_GPIO_Port GPIOA
#define LCD_D7_Pin GPIO_PIN_0
#define LCD_D7_GPIO_Port GPIOA

void LCD_Init(void);
void LCD_Send_Command(uint8_t cmd);
void LCD_Send_Data(uint8_t data);
void LCD_Send_String(char *str);
void LCD_Set_Cursor(uint8_t row, uint8_t col);

#endif /* INC_LCD_H_ */
