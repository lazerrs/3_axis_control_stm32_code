#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "main.h"
#include "adc.h"
#include "tim.h"

// تنظیمات جویستیک و سرعت
#define ADC_CENTER      2050
#define DEAD_ZONE       100

#define MIN_PERIOD      250
#define MAX_PERIOD      3500

#define MIN_PERIOD_2      1000
#define MAX_PERIOD_2     30000

#define MIN_PERIOD_3      1000
#define MAX_PERIOD_3    30000
// پایه‌های موتور و لیمیت‌سوییچ‌ها
//استپ موتور اول
#define STEP_Pin        GPIO_PIN_3
#define STEP_GPIO_Port  GPIOB
#define DIR_Pin         GPIO_PIN_4
#define DIR_GPIO_Port   GPIOB
#define LIMIT_UP_Pin    GPIO_PIN_9
#define LIMIT_UP_GPIO_Port  GPIOB
#define LIMIT_DOWN_Pin  GPIO_PIN_8
#define LIMIT_DOWN_GPIO_Port GPIOB
//استپ موتور دوم
#define STEP_Pin_2     GPIO_PIN_0
#define STEP_GPIO_Port_2   GPIOB
#define DIR_Pin_2          GPIO_PIN_1
#define DIR_GPIO_Port_2    GPIOB
#define LIMIT_UP_Pin_2     GPIO_PIN_12
#define LIMIT_UP_GPIO_Port_2   GPIOB
#define LIMIT_DOWN_Pin_2   GPIO_PIN_11
#define LIMIT_DOWN_GPIO_Port_2  GPIOB
//استپ موتور سوم
#define STEP_Pin_3        GPIO_PIN_4
#define STEP_GPIO_Port_3  GPIOB
#define DIR_Pin_3         GPIO_PIN_3
#define DIR_GPIO_Port_3   GPIOB
#define LIMIT_UP_Pin_3    GPIO_PIN_15
#define LIMIT_UP_GPIO_Port_3  GPIOA
#define LIMIT_DOWN_Pin_3  GPIO_PIN_13
#define LIMIT_DOWN_GPIO_Port_3 GPIOB
// متغیرهای موتور
extern volatile int32_t stepCount;
extern volatile uint8_t motorEnable;
extern volatile uint8_t motorEnable_2;
extern uint32_t adcValue;
extern char lcd0[20],lcd3[20];
// توابع
void MotorControl_UpdateSpeed(uint32_t adc_val,uint32_t adc_val2);
void MotorControl_TIMCallback();
void MotorControl_TIMCallback_2();
void MotorControl_LCDUpdate(void);
void MotorControl_StepMotor_homing(uint32_t speed);

#endif
